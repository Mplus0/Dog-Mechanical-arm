"""Pure Stage-4C command/session state for read-only target localization."""

from dataclasses import dataclass
from typing import Any, Dict, Optional, Tuple

from apriltag_block_grasp.core.target_lock import StableTargetLock


def valid_task_id(value: Any) -> bool:
    """Accept integer or non-empty string IDs, but reject bool and containers."""
    return (isinstance(value, int) and not isinstance(value, bool)) or (
        isinstance(value, str) and bool(value.strip())
    )


@dataclass(frozen=True)
class CommandDecision:
    action: str
    reason: str
    task_id: Any


class LocalizationTaskSession:
    """Own one active pick-localization attempt without any motion authority."""

    def __init__(self, target_lock: StableTargetLock) -> None:
        self.target_lock = target_lock
        self.active_task_id: Any = None
        self.active_cmd: Optional[str] = None
        self.state = "idle"
        self.last_reason = "ready"
        self.duplicate_command_count = 0
        self.last_command_event: Optional[Dict[str, Any]] = None

    @property
    def active(self) -> bool:
        return self.active_cmd is not None

    def accept_command(self, data: Dict[str, Any], now_s: float) -> CommandDecision:
        task_id = data.get("task_id")
        cmd = data.get("cmd")
        if not valid_task_id(task_id):
            return CommandDecision("reject", "invalid_task_id", task_id)
        if not isinstance(cmd, str) or cmd.strip() != "pick":
            return CommandDecision("reject", "unsupported_command", task_id)

        if self.active:
            if task_id == self.active_task_id:
                self.duplicate_command_count += 1
                self.last_command_event = {
                    "event": "duplicate_active_pick",
                    "task_id": task_id,
                    "action": "ignored",
                }
                return CommandDecision("ignore", "duplicate_active_pick", task_id)
            return CommandDecision("busy", "arm_busy", task_id)

        self.target_lock.reset()
        self.target_lock.start(now_s)
        self.active_task_id = task_id
        self.active_cmd = "pick"
        self.state = "localizing"
        self.last_reason = "pick_accepted"
        self.duplicate_command_count = 0
        self.last_command_event = None
        return CommandDecision("accepted", "pick_accepted", task_id)

    def update_candidates(
        self, payload: Dict[str, Any], now_s: float
    ) -> Optional[Dict[str, Any]]:
        if not self.active:
            return None
        result = self.target_lock.update(payload, now_s)
        self._apply_localization_result(result)
        return result

    def check_timeout(self, now_s: float) -> Optional[Dict[str, Any]]:
        if not self.active:
            return None
        result = self.target_lock.check_timeout(now_s)
        if result is not None:
            self._apply_localization_result(result)
        return result

    def _apply_localization_result(self, result: Dict[str, Any]) -> None:
        status = result.get("status")
        self.last_reason = str(result.get("reason", ""))
        if status == "stable":
            self.state = "snapshot_ready"
        elif status == "failed":
            self.state = "localization_failed"
        else:
            self.state = "localizing"

    def finish_terminal(self) -> Tuple[Any, Optional[str]]:
        """Clear the active command after its terminal messages are published."""
        finished_task_id = self.active_task_id
        finished_cmd = self.active_cmd
        self.active_task_id = None
        self.active_cmd = None
        return finished_task_id, finished_cmd

    def state_payload(self) -> Dict[str, Any]:
        payload = {
            "state": self.state,
            "reason": self.last_reason,
            "active_task_id": self.active_task_id,
            "active_cmd": self.active_cmd,
            "locked_id": self.target_lock.locked_id,
            "collected_frame_count": len(self.target_lock.samples),
            "required_frame_count": self.target_lock.config.stable_frame_count,
            "duplicate_command_count": self.duplicate_command_count,
        }
        if self.last_command_event is not None:
            payload["last_command_event"] = dict(self.last_command_event)
        return payload
