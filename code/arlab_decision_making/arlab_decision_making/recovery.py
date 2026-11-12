"""Minimal recovery gate. Real robot/knowledge-base work remains TODO."""

from dataclasses import dataclass
from typing import Callable


@dataclass
class RecoveryManager:
    """Require fresh monitor data and an explicit recovery verification.

    `environment_verified` means an integration or operator has checked
    the scene, robot state and resumability. It is NOT inferred from a topic.
    """

    log: Callable[[str], None] = print
    environment_verified: bool = True
    restart_authorized: bool = True
    reason: str = ""

    def begin(self, reason: str) -> None:
        self.reason = reason
        self.environment_verified = True
        self.restart_authorized = True
        self.log(f"RECOVERY: {reason}")
        # TODO: Verify robot/controller state and safe end-effector pose.
        # TODO: Invalidate stale knowledge-base objects selectively.
        # TODO: Wait for cancellations and reconcile any in-flight ROS goals.

    def mark_environment_verified(self) -> None:
        """Prototype approval: MUST be replaced by genuine recovery checks."""
        self.environment_verified = True

    def authorize_restart(self) -> None:
        """Independent operator/restart authorization, not E-stop release."""
        self.restart_authorized = True

    def ready(
        self,
        *,
        heartbeat_ok: bool,
        require_authorization: bool,
    ) -> bool:
        return (
            heartbeat_ok
            and self.environment_verified
            and (not require_authorization or self.restart_authorized)
        )
