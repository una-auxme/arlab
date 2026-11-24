"""Non-blocking py_trees leaf for 291-move-to-entity.

Create one client per ROS node and reuse it across behavior-tree rebuilds:

    client = MovementActionClient.connect(node)
    behavior = NavigateTo("Navigate to washer", "laundry.enter",
                          "laundry.enter.navigate_to_washer", 42, client)

The client requires MovementAction rebuilt with the ``entityid`` goal field.
It expects the server to set ``MovementAction.Result.success`` to true only
when the destination has been reached.
The server's cancel callback can stop the entire navigation stack.
"""

from concurrent.futures import Future
from dataclasses import dataclass, field
from math import isfinite
from threading import RLock
from time import monotonic

import py_trees
from py_trees.common import Access, Status


def validate_entity_id(entity_id: int) -> None:
    if type(entity_id) is not int or not 0 < entity_id < 2**63:
        raise ValueError("navigation_entity_id must be a positive Knowledge Base ID (ROS int64)")


@dataclass
class _Attempt:
    result: Future = field(default_factory=Future)
    handle: object = None
    cancel_requested: bool = False


class MovementActionClient:
    """Own at most one goal, including across BT pause/rebuild and timeouts."""

    def __init__(self, action_client, goal_type, goal_status, logger):
        self._client = action_client
        self._goal_type = goal_type
        self._status = goal_status
        self._logger = logger
        self._attempt = None
        self._closed = False
        self._lock = RLock()

    @classmethod
    def connect(cls, node, action_name="/movement/action"):
        # Keep the tree, demo and unit tests importable without a ROS installation.
        from action_msgs.msg import GoalStatus
        from arlab_common_interfaces.action import MovementAction
        from rclpy.action import ActionClient

        if "entityid" not in MovementAction.Goal.get_fields_and_field_types():
            raise RuntimeError("MovementAction lacks entityid; rebuild and source the 291-move-to-entity interfaces")
        return cls(ActionClient(node, MovementAction, action_name), MovementAction.Goal, GoalStatus, node.get_logger())

    @property
    def pending(self):
        with self._lock:
            return self._attempt is not None

    def try_start(self, entity_id):
        with self._lock:
            if self._closed or self._attempt is not None or not self._client.server_is_ready():
                return None
            goal = self._goal_type(cmd="move_to_entity", entityid=entity_id)
            attempt = self._attempt = _Attempt()
            try:
                goal_response = self._client.send_goal_async(goal)
                goal_response.add_done_callback(lambda future: self._on_goal(attempt, future))
            except Exception as exc:
                self._uncertain(attempt, exc)
            return attempt.result

    def cancel(self, result):
        with self._lock:
            attempt = self._attempt
            if attempt is None or attempt.result is not result or attempt.cancel_requested:
                return
            attempt.cancel_requested = True
            if attempt.handle is not None:
                self._send_cancel(attempt)

    def close(self):
        with self._lock:
            self._closed = True
            if self._attempt is not None:
                self.cancel(self._attempt.result)

    def _on_goal(self, attempt, future):
        with self._lock:
            try:
                handle = future.result()
                if not handle.accepted:
                    self._attempt = None
                    attempt.result.set_result((False, "MovementAction server rejected the goal"))
                    return
                attempt.handle = handle
                handle.get_result_async().add_done_callback(lambda result: self._on_result(attempt, result))
            except Exception as exc:
                self._uncertain(attempt, exc)
            if self._attempt is attempt and attempt.cancel_requested and attempt.handle is not None:
                self._send_cancel(attempt)

    def _send_cancel(self, attempt):
        try:
            # A cancel reply alone is not a terminal MovementAction result.
            attempt.handle.cancel_goal_async()
        except Exception as exc:
            self._logger.error(f"Could not request MovementAction goal cancellation: {exc}")

    def _on_result(self, attempt, future):
        with self._lock:
            try:
                response = future.result()
                if response.status not in (
                    self._status.STATUS_SUCCEEDED,
                    self._status.STATUS_ABORTED,
                    self._status.STATUS_CANCELED,
                ):
                    raise RuntimeError(f"non-terminal MovementAction result status {response.status}")
                result = response.result
                # Acceptance and the ROS terminal status are not arrival confirmations.
                # Only the final result payload reports that the destination was reached.
                reached = (
                    not attempt.cancel_requested
                    and response.status == self._status.STATUS_SUCCEEDED
                    and result.success is True
                )
                detail = f"MovementAction status={response.status}, error_code={result.error_code}"
                if result.message:
                    detail += f": {result.message}"
                if attempt.cancel_requested:
                    detail += " (result arrived after cancellation was requested)"
                self._attempt = None
                attempt.result.set_result((reached, detail))
            except Exception as exc:
                self._uncertain(attempt, exc)

    def _uncertain(self, attempt, exc):
        # A lost response does not prove that the goal never ran. Do not
        # release the single-goal guard without a terminal MovementAction result.
        self._logger.error(f"MovementAction goal state could not be established: {exc}")
        if not attempt.result.done():
            attempt.result.set_result((False, str(exc)))


class NavigateTo(py_trees.behaviour.Behaviour):
    """Send one goal per activation and write the task's resume checkpoint."""

    def __init__(
        self, name, process_id, action_id, entity_id, client, *,
        server_timeout_sec=10.0, goal_timeout_sec=120.0, clock=monotonic,
    ):
        super().__init__(name=name)
        if not all(isfinite(t) and t > 0 for t in (server_timeout_sec, goal_timeout_sec)):
            raise ValueError("Navigation timeouts must be finite and positive")
        self.PROCESS_ID = process_id
        self.ACTION_ID = action_id
        validate_entity_id(entity_id)
        self._entity_id = entity_id
        self._client = client
        self._server_timeout = server_timeout_sec
        self._goal_timeout = goal_timeout_sec
        self._clock = clock
        self._result = None
        self.bb = self.attach_blackboard_client(name=action_id)
        self.bb.register_key("memory_process_name", access=Access.WRITE)
        self.bb.register_key("memory_action_name", access=Access.WRITE)

    def initialise(self):
        self._result = None
        self._deadline = self._clock() + self._server_timeout

    def update(self):
        self.bb.memory_process_name = self.PROCESS_ID
        self.bb.memory_action_name = self.ACTION_ID
        now = self._clock()

        if self._result is None:
            if now >= self._deadline:
                return self._fail("Timed out waiting for MovementAction or for the previous goal to end")
            self._result = self._client.try_start(self._entity_id)
            if self._result is None:
                self.feedback_message = "Waiting for MovementAction or for the previous goal to end"
                return Status.RUNNING
            self._deadline = now + self._goal_timeout

        if self._result.done():
            reached, message = self._result.result()
            if not reached:
                return self._fail(message)
            self.feedback_message = f"Entity {self._entity_id} reached: {message}"
            return Status.SUCCESS
        if now >= self._deadline:
            return self._fail("MovementAction goal timed out; requesting cancellation")
        self.feedback_message = "Waiting for the MovementAction goal result"
        return Status.RUNNING

    def _fail(self, message):
        self.feedback_message = message
        self.logger.warning(f"{self.ACTION_ID}: {message}")
        return Status.FAILURE

    def terminate(self, new_status):
        if new_status != Status.SUCCESS and self._result is not None:
            self._client.cancel(self._result)