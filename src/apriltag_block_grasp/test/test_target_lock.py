"""Unit tests for Stage-4B selection, locking and stability logic."""

from apriltag_block_grasp.core.target_lock import StableTargetLock, TargetStabilityConfig


def config(frame_count=3, threshold=(1.0, 1.0, 1.0), timeout=3.0):
    return TargetStabilityConfig(
        selection_order=(0, 1),
        stable_frame_count=frame_count,
        xyz_peak_to_peak_threshold_mm=threshold,
        stable_timeout_s=timeout,
        max_pnp_arm_stamp_delta_s=0.2,
        max_arm_state_reported_age_s=0.1,
    )


def payload(*candidates, delta=0.05, age=0.02, valid=True, stamp=1.0):
    return {
        "valid": valid,
        "pnp_stamp": stamp,
        "pnp_arm_stamp_delta_s": delta,
        "arm_state_reported_age_s": age,
        "candidates": [
            {
                "tag_id": tag_id,
                "base_object_mm": {"x": xyz[0], "y": xyz[1], "z": xyz[2]},
            }
            for tag_id, xyz in candidates
        ],
    }


def test_selects_id_zero_first_when_both_are_visible():
    lock = StableTargetLock(config())
    result = lock.update(payload((1, (20.0, 0.0, 0.0)), (0, (10.0, 0.0, 0.0))), 0.0)
    assert result["locked_id"] == 0
    assert result["collected_frame_count"] == 1


def test_selects_id_one_when_id_zero_is_not_visible():
    lock = StableTargetLock(config())
    result = lock.update(payload((1, (20.0, 0.0, 0.0))), 0.0)
    assert result["locked_id"] == 1


def test_locked_id_does_not_switch_and_missing_frame_clears_window():
    lock = StableTargetLock(config())
    lock.update(payload((0, (10.0, 0.0, 0.0))), 0.0)
    result = lock.update(payload((1, (20.0, 0.0, 0.0))), 0.1)
    assert result["locked_id"] == 0
    assert result["reason"] == "locked_target_missing"
    assert result["collected_frame_count"] == 0
    assert result["window_reset_count"] == 1


def test_stable_snapshot_uses_axis_medians_and_is_latched():
    lock = StableTargetLock(config())
    lock.update(payload((0, (10.0, 20.0, 30.0)), stamp=1.0), 0.0)
    lock.update(payload((0, (10.4, 19.8, 30.3)), stamp=1.1), 0.1)
    result = lock.update(payload((0, (9.8, 20.2, 29.9)), stamp=1.2), 0.2)
    assert result["status"] == "stable"
    assert result["base_object_median_mm"] == {"x": 10.0, "y": 20.0, "z": 30.0}
    assert result["xyz_peak_to_peak_mm"] == {
        "x": 0.5999999999999996,
        "y": 0.3999999999999986,
        "z": 0.40000000000000213,
    }
    latched = lock.update(payload((1, (100.0, 100.0, 100.0))), 1.0)
    assert latched == result


def test_excessive_variation_does_not_create_stable_snapshot():
    lock = StableTargetLock(config())
    lock.update(payload((0, (0.0, 0.0, 0.0))), 0.0)
    lock.update(payload((0, (0.0, 0.0, 0.0))), 0.1)
    result = lock.update(payload((0, (2.0, 0.0, 0.0))), 0.2)
    assert result["status"] == "collecting"
    assert result["reason"] == "xyz_peak_to_peak_exceeded"
    assert result["current_xyz_peak_to_peak_mm"]["x"] == 2.0


def test_timeout_distinguishes_not_found_from_unstable():
    not_found = StableTargetLock(config(timeout=1.0))
    assert not_found.update(payload(), 0.0)["status"] == "waiting"
    result = not_found.update(payload(), 1.0)
    assert result["status"] == "failed"
    assert result["reason"] == "target_not_found"

    unstable = StableTargetLock(config(timeout=1.0))
    unstable.update(payload((0, (0.0, 0.0, 0.0))), 0.0)
    unstable.update(payload(), 0.5)
    result = unstable.update(payload(), 1.0)
    assert result["status"] == "failed"
    assert result["reason"] == "target_unstable"


def test_unstable_timeout_reports_last_and_best_window_diagnostics():
    lock = StableTargetLock(config(frame_count=3, timeout=0.3))
    lock.update(payload((1, (0.0, 0.0, 0.0))), 0.0)
    lock.update(payload((1, (0.0, 0.0, 0.0))), 0.1)
    lock.update(payload((1, (0.0, 0.0, 2.0))), 0.2)
    result = lock.update(payload((1, (0.0, 0.0, 2.5))), 0.3)
    assert result["status"] == "failed"
    assert result["reason"] == "target_unstable"
    assert result["failure_detail"] == "xyz_peak_to_peak_exceeded"
    assert result["last_xyz_peak_to_peak_mm"] == {"x": 0.0, "y": 0.0, "z": 2.5}
    assert result["best_xyz_peak_to_peak_mm"] == {"x": 0.0, "y": 0.0, "z": 2.0}
    assert result["best_max_threshold_ratio"] == 2.0
    assert result["threshold_exceeded_axes"] == ["z"]


def test_stale_or_unsynchronized_input_is_not_sampled():
    lock = StableTargetLock(config())
    result = lock.update(payload((0, (0.0, 0.0, 0.0)), delta=0.21), 0.0)
    assert result["reason"] == "pnp_arm_stamp_delta_exceeded"
    assert result["collected_frame_count"] == 0
    assert result["ever_saw_allowed_target"] is True
    result = lock.update(payload((0, (0.0, 0.0, 0.0)), age=0.11), 0.1)
    assert result["reason"] == "arm_state_stale"
    assert result["collected_frame_count"] == 0


def test_explicit_timeout_check_works_without_any_candidate_update():
    lock = StableTargetLock(config(timeout=1.0))
    lock.start(5.0)
    assert lock.check_timeout(5.9) is None
    result = lock.check_timeout(6.0)
    assert result["status"] == "failed"
    assert result["reason"] == "target_not_found"
    assert result["failure_detail"] == "candidate_message_timeout"
