"""Unit tests for the gated B-only driver command contract."""

import pytest

from apriltag_block_grasp.core.b_joint_command import (
    BJointCommandLimits,
    validate_b_joint_request,
)


def limits():
    return BJointCommandLimits(
        minimum_deg=-20.0,
        maximum_deg=20.0,
        maximum_delta_deg=10.0,
        maximum_speed_deg_s=35.0,
        maximum_acceleration=35.0,
    )


def request(angle=5.0, speed=10.0, acceleration=10.0):
    return {
        "command_id": "test-b-1",
        "type": "move_b_joint",
        "joint": 1,
        "angle": angle,
        "speed": speed,
        "acceleration": acceleration,
    }


def test_valid_request_builds_exact_single_t121_b_command():
    result = validate_b_joint_request(request(), current_b_deg=0.5, limits=limits())
    assert result["requested_delta_deg"] == 4.5
    assert result["serial_command"] == {
        "T": 121,
        "joint": 1,
        "angle": 5.0,
        "spd": 10.0,
        "acc": 10.0,
    }


@pytest.mark.parametrize("joint", [2, 3, 4, 5, 6])
def test_every_non_b_joint_is_rejected(joint):
    data = request()
    data["joint"] = joint
    with pytest.raises(ValueError, match="only B joint 1"):
        validate_b_joint_request(data, current_b_deg=0.0, limits=limits())


def test_absolute_range_and_relative_delta_are_both_enforced():
    with pytest.raises(ValueError, match="outside enabled range"):
        validate_b_joint_request(request(angle=20.1), 15.0, limits())
    with pytest.raises(ValueError, match="exceeds maximum_delta_deg"):
        validate_b_joint_request(request(angle=10.1), 0.0, limits())


def test_speed_and_acceleration_limits_are_enforced():
    with pytest.raises(ValueError, match="speed must be"):
        validate_b_joint_request(request(speed=35.1), 0.0, limits())
    with pytest.raises(ValueError, match="acceleration must be"):
        validate_b_joint_request(request(acceleration=35.1), 0.0, limits())


def test_wrong_command_type_is_rejected():
    data = request()
    data["type"] = "move_joint"
    with pytest.raises(ValueError, match="command type must be move_b_joint"):
        validate_b_joint_request(data, 0.0, limits())
