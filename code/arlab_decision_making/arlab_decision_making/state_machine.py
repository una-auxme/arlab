"""Three-region hierarchical supervisor with central Recovery outside Operational.

This intentionally models the UML statechart with explicit Python transitions,
not a py_trees Parallel. All methods use one lock so events and BT ticks cannot
mutate execution state concurrently. Separate ROS callbacks/timers provide
asynchronous observations independent of the BT tick timer.

Operational has these orthogonal regions:
  1. Heartbeat: Monitoring -> PauseByHeartbeat
  2. BT:        Starting -> Running -> Paused / Succeeded / Failed

ONLY the BT region's transition into Paused exits Operational to Recovery.
Heartbeat and E-stop observations continue outside Operational, but their
region states are inactive until re-entry and then start at Monitoring.
"""

from enum import Enum
from threading import RLock
from typing import Callable

from .heartbeat_monitor import HeartbeatMonitor
from .recovery import RecoveryManager
from .bt_controller import BehaviorTreeController


class Phase(str, Enum):
    INIT = "INIT"
    OPERATIONAL = "OPERATIONAL"
    RECOVERY = "RECOVERY"
    FINISHED = "FINISHED"
    FAILED = "FAILED"


class HBRegion(str, Enum):
    INACTIVE = "INACTIVE"
    MONITORING = "MONITORING"
    PAUSE_BY_HEARTBEAT = "PAUSE_BY_HEARTBEAT"




class BTRegion(str, Enum):
    INACTIVE = "INACTIVE"
    STARTING = "STARTING"
    RUNNING = "RUNNING"
    PAUSED = "PAUSED"
    SUCCEEDED = "SUCCEEDED"
    FAILED = "FAILED"


class DecisionStateMachine:
    """Supervisor for one selected task; hardware control is not included."""

    def __init__(
        self,
        heartbeat: HeartbeatMonitor,
        bt_controller: BehaviorTreeController,
        recovery: RecoveryManager,
        *,
        max_recoveries: int = 3,
        require_restart_authorization: bool = True,
        log: Callable[[str], None] = print,
    ) -> None:
        if max_recoveries < 0:
            raise ValueError("max_recoveries must be nonnegative")
        self.heartbeat = heartbeat
        self.bt = bt_controller
        self.recovery = recovery
        self.max_recoveries = max_recoveries
        self.require_restart_authorization = require_restart_authorization
        self.log = log
        self._lock = RLock()
        self.phase = Phase.INIT
        self.hb_region = HBRegion.INACTIVE
        self.bt_region = BTRegion.INACTIVE
        self.bt_history = BTRegion.STARTING
        self.recovery_count = 0
        self.last_fault = ""

    def monitor_tick(self) -> None:
        """Called independently of the BT tick, e.g. from a 20 Hz timer."""
        with self._lock:
            if self.phase == Phase.INIT:
                # Never start a task before the expected live inputs are seen.
                if self.heartbeat.healthy:
                    self._enter_operational(first_time=True)
            elif self.phase == Phase.OPERATIONAL:
                self._react_to_monitor_faults()
            elif self.phase == Phase.RECOVERY:
                if self.recovery.ready(
                    heartbeat_ok=self.heartbeat.healthy,
                    require_authorization=self.require_restart_authorization,
                ):
                    self._enter_operational(first_time=False)

    def heartbeat_event(self, module_id: int, module_state: int) -> None:
        """Triggered by ROS subscriber; interrupt without waiting for BT tick."""
        with self._lock:
            if self.phase == Phase.OPERATIONAL:
                self._react_to_monitor_faults()

    def tick_bt(self) -> None:
        """Called from BT timer, only while Operational and monitors OK."""
        with self._lock:
            # Guard against freshness changes since the last monitor event.
            self._react_to_monitor_faults()
            if self.phase != Phase.OPERATIONAL:
                return
            if self.bt_region == BTRegion.STARTING:
                self.bt.start()
                self.bt_region = BTRegion.RUNNING
                self.log("BT: Starting -> Running")
                return
            if self.bt_region == BTRegion.PAUSED:
                # UML history returns to Paused; this transition resumes it.
                self.bt.pause()
                self.bt_region = BTRegion.RUNNING
                self.log("BT: Paused -> Running (checkpoint resume)")
                return
            if self.bt_region != BTRegion.RUNNING:
                return
            try:
                outcome = self.bt.tick()
            except Exception as exc:
                self.log(f"BT exception: {exc}")
                outcome = "FAILURE"
            if outcome == "SUCCESS":
                self.bt_region = BTRegion.SUCCEEDED
                self.phase = Phase.FINISHED
                self.log("BT Succeeded -> Finished")
            elif outcome == "FAILURE":
                self.bt_region = BTRegion.FAILED
                self.log("BT Failed: requesting recovery")
                self._enter_retry("BT failure")
            elif outcome != "RUNNING":
                self.bt_region = BTRegion.FAILED
                self._enter_retry(f"Unexpected BT status: {outcome}")

    def _react_to_monitor_faults(self) -> None:
        if self.phase != Phase.OPERATIONAL:
            return
        if not self.heartbeat.healthy:
            self.hb_region = HBRegion.PAUSE_BY_HEARTBEAT
            self.log("Heartbeat: Monitoring -> Pause by Heartbeat")
            self.bt_region = BTRegion.PAUSED
            self.

    def _pause_bt_and_exit(self, reason: str) -> None:
        self.bt_region = BTRegion.PAUSED
        self.bt_history = BTRegion.PAUSED
        self.log("BT: Running/Starting -> Paused")
        # In a real action, `terminate(INVALID)` MUST request cancellation.
        self.bt.pause()
        self._enter_retry(reason)

    def _enter_retry(self, reason: str) -> None:
        # A failed BT action must also be canceled before recovery.
        self.bt.pause()
        self.bt_history = BTRegion.PAUSED
        self.phase = Phase.RECOVERY
        # The three orthogonal REGION states are exited, but the ROS input
        # observers keep receiving current health signals outside Operational.
        self.hb_region = HBRegion.INACTIVE
        self.bt_region = BTRegion.INACTIVE
        self.log("BT region -> retry")

    def _enter_operational(self, *, first_time: bool) -> None:
        self.phase = Phase.OPERATIONAL
        # Always use initial monitoring states for the first two regions.
        self.hb_region = HBRegion.MONITORING
        # Only the BT region has (explicitly implemented) shallow history.
        self.bt_region = BTRegion.STARTING if first_time else self.bt_history
        self.log(
            "Enter Operational: HB=Monitoring, "
            f"BT={self.bt_region.value}"
        )

    def confirm_environment_checked(self) -> None:
        with self._lock:
            if self.phase == Phase.RECOVERY:
                self.recovery.mark_environment_verified()
                self.log("Recovery: external environment check acknowledged")

    def authorize_restart(self) -> None:
        with self._lock:
            if self.phase == Phase.RECOVERY:
                self.recovery.authorize_restart()
                self.log("Recovery: operator restart authorization received")

    def status(self) -> dict:
        with self._lock:
            return {
                "phase": self.phase.value,
                "heartbeat_region": self.hb_region.value,
                "bt_region": self.bt_region.value,
                "bt_history": self.bt_history.value,
                "heartbeat_ok": self.heartbeat.healthy,
                "recoveries": self.recovery_count,
                "last_fault": self.last_fault,
            }
