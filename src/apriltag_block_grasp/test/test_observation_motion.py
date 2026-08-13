"""Unit tests for the fixed, non-overridable observation motion contract."""

import pytest

from apriltag_block_grasp.core.observation_motion import (
    ObservationMotionConfig,
    validate_observation_request,
)


def config():
    return ObservationMotionConfig(
        pose_deg={"b": 0.0, "s": 0.0, "e": 70.0, "t": 90.0, "r": -90.0},
        move_order=("b", "t", "r", "s", "e"),
        speed_deg_s=35.0,
        acceleration=35.0,
        command_interval_s=0.1,
        timed_wait_s=3.0,
    )


def test_exact_hardware_validated_sequence_is_planned_without_gripper():
    commands = config().serial_commands()
    assert [item["joint"] for item in commands] == [1, 4, 5, 2, 3]
    assert [item["angle"] for item in commands] == [0.0, 90.0, -90.0, 0.0, 70.0]
    assert all(item["T"] == 121 for item in commands)
    assert all(item["spd"] == 35.0 and item["acc"] == 35.0 for item in commands)
    assert all(item["joint"] != 6 for item in commands)


def test_semantic_request_cannot_override_any_joint_or_motion_field():
    assert validate_observation_request(
        {"command_id": "observation-1", "type": "move_observation_pose"}
    ) == "observation-1"
    for field, value in (
        ("joint", 2),
        ("angle", 10.0),
        ("pose", {}),
        ("speed", 1.0),
        ("acceleration", 1.0),
    ):
        request = {
            "command_id": "observation-1",
            "type": "move_observation_pose",
            field: value,
        }
        with pytest.raises(ValueError, match="cannot override"):
            validate_observation_request(request)


def test_gripper_is_not_a_valid_observation_pose_joint():
    with pytest.raises(ValueError, match="only b, s, e, t and r"):
        ObservationMotionConfig(
            pose_deg={
                "b": 0.0,
                "s": 0.0,
                "e": 70.0,
                "t": 90.0,
                "r": -90.0,
                "g": 0.0,
            },
            move_order=("b", "t", "r", "s", "e"),
            speed_deg_s=35.0,
            acceleration=35.0,
            command_interval_s=0.1,
            timed_wait_s=3.0,
        )
