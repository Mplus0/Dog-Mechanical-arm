"""Tests for the fixed, non-overridable gripper-open action."""

import pytest

from apriltag_block_grasp.core.gripper_motion import (
    GripperOpenConfig,
    validate_open_gripper_request,
)


def config():
    return GripperOpenConfig(
        angle_deg=110.0,
        speed_deg_s=25.0,
        acceleration=25.0,
        timed_wait_s=1.0,
    )


def test_open_action_builds_exact_single_joint_6_command():
    assert config().serial_command() == {
        "T": 121,
        "joint": 6,
        "angle": 110.0,
        "spd": 25.0,
        "acc": 25.0,
    }


def test_request_cannot_override_open_parameters():
    assert validate_open_gripper_request(
        {"command_id": "open-1", "type": "open_gripper"}
    ) == "open-1"
    for field, value in (
        ("joint", 6),
        ("angle", 50.0),
        ("speed", 5.0),
        ("acceleration", 5.0),
    ):
        request = {"command_id": "open-1", "type": "open_gripper", field: value}
        with pytest.raises(ValueError, match="cannot override"):
            validate_open_gripper_request(request)


def test_nonpositive_speed_wait_or_acceleration_is_rejected():
    with pytest.raises(ValueError, match="speed and acceleration"):
        GripperOpenConfig(110.0, 0.0, 25.0, 1.0)
    with pytest.raises(ValueError, match="timed wait"):
        GripperOpenConfig(110.0, 25.0, 25.0, 0.0)
