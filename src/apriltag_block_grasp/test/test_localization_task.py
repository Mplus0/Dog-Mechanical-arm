"""Unit tests for the command-driven Stage-4C localization session."""

from apriltag_block_grasp.core.localization_task import LocalizationTaskSession
from apriltag_block_grasp.core.target_lock import StableTargetLock, TargetStabilityConfig


def session(frame_count=3, timeout=1.0):
    config = TargetStabilityConfig(
        selection_order=(0, 1),
        stable_frame_count=frame_count,
        xyz_peak_to_peak_threshold_mm=(1.0, 1.0, 1.0),
        stable_timeout_s=timeout,
        max_pnp_arm_stamp_delta_s=0.2,
        max_arm_state_reported_age_s=0.1,
    )
    return LocalizationTaskSession(StableTargetLock(config))


def payload(tag_id=0, xyz=(10.0, 20.0, 30.0), stamp=1.0):
    return {
        "valid": True,
        "pnp_stamp": stamp,
        "pnp_arm_stamp_delta_s": 0.05,
        "arm_state_reported_age_s": 0.02,
        "candidates": [
            {
                "tag_id": tag_id,
                "base_object_mm": {"x": xyz[0], "y": xyz[1], "z": xyz[2]},
            }
        ],
    }


def test_only_valid_pick_command_is_accepted():
    task = session()
    assert task.accept_command({"task_id": 1, "cmd": "pick"}, 0.0).action == "accepted"

    fresh = session()
    assert fresh.accept_command({"task_id": None, "cmd": "pick"}, 0.0).reason == "invalid_task_id"
    assert fresh.accept_command({"task_id": True, "cmd": "pick"}, 0.0).reason == "invalid_task_id"
    assert fresh.accept_command({"task_id": "", "cmd": "pick"}, 0.0).reason == "invalid_task_id"
    assert fresh.accept_command({"task_id": 2, "cmd": "place"}, 0.0).reason == "unsupported_command"


def test_duplicate_active_pick_is_ignored_and_different_task_is_busy():
    task = session()
    task.accept_command({"task_id": 101, "cmd": "pick"}, 0.0)
    duplicate = task.accept_command({"task_id": 101, "cmd": "pick"}, 0.1)
    other = task.accept_command({"task_id": 102, "cmd": "pick"}, 0.2)
    assert duplicate.action == "ignore"
    assert duplicate.reason == "duplicate_active_pick"
    assert other.action == "busy"
    assert other.reason == "arm_busy"
    assert task.active_task_id == 101


def test_localization_only_finishes_with_snapshot_ready_state():
    task = session()
    task.accept_command({"task_id": "pick-a", "cmd": "pick"}, 0.0)
    task.update_candidates(payload(xyz=(10.0, 20.0, 30.0), stamp=1.0), 0.1)
    task.update_candidates(payload(xyz=(10.2, 20.1, 29.9), stamp=1.1), 0.2)
    result = task.update_candidates(payload(xyz=(9.9, 19.9, 30.1), stamp=1.2), 0.3)
    assert result["status"] == "stable"
    assert task.state == "snapshot_ready"
    assert task.active_task_id == "pick-a"
    assert task.active is True
    assert task.finish_terminal() == ("pick-a", "pick")
    assert task.active is False


def test_timer_reports_target_not_found_without_candidate_messages():
    task = session(timeout=1.0)
    task.accept_command({"task_id": 1, "cmd": "pick"}, 10.0)
    assert task.check_timeout(10.9) is None
    result = task.check_timeout(11.0)
    assert result["status"] == "failed"
    assert result["reason"] == "target_not_found"
    assert result["failure_detail"] == "candidate_message_timeout"
    assert task.state == "localization_failed"


def test_new_pick_after_terminal_publication_starts_clean_attempt():
    task = session(frame_count=1)
    task.accept_command({"task_id": 1, "cmd": "pick"}, 0.0)
    result = task.update_candidates(payload(tag_id=0), 0.1)
    assert result["locked_id"] == 0
    task.finish_terminal()

    decision = task.accept_command({"task_id": 2, "cmd": "pick"}, 1.0)
    assert decision.action == "accepted"
    assert task.target_lock.locked_id is None
    result = task.update_candidates(payload(tag_id=1), 1.1)
    assert result["locked_id"] == 1
