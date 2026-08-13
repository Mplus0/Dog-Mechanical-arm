"""Tests for manual compensation arithmetic and bounds."""

import pytest

from apriltag_block_grasp.core.manual_compensation import (
    apply_values,
    corrected_offset,
    validate_gripper_angle,
)


def test_corrected_offset_adds_observed_delta_and_manual_trim():
    result = corrected_offset(
        (-24.0, 33.0, 0.5),
        (295.0, -29.0, -8.0),
        (300.0, -37.0, -5.0),
        (1.0, 0.0, -1.0),
    )
    assert result == (-18.0, 25.0, 2.5)


def test_gripper_angle_is_bounded():
    assert validate_gripper_angle(55, "close") == 55.0
    with pytest.raises(ValueError, match="within"):
        validate_gripper_angle(181, "close")


def test_apply_values_updates_only_calibration_fields():
    grasp = {"final_grasp_tcp_offset_base_mm": [0.0, 0.0, 0.0], "keep": 1}
    motion = {
        "gripper_open": {"angle_deg": 110.0, "keep": 2},
        "pick_sequence": {"close_gripper": {"angle_deg": 55.0, "keep": 3}},
    }
    apply_values(grasp, motion, (1, 2, 3), 100, 60)
    assert grasp["final_grasp_tcp_offset_base_mm"] == [1.0, 2.0, 3.0]
    assert motion["gripper_open"]["angle_deg"] == 100.0
    assert motion["pick_sequence"]["close_gripper"]["angle_deg"] == 60.0
    assert grasp["keep"] == 1
