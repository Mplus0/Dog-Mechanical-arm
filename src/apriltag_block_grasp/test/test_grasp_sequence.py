"""Tests for staged execution gating and remaining pick commands."""

import pytest

from apriltag_block_grasp.core.grasp_sequence import (
    CARTESIAN_STAGE_COMMAND_TYPE,
    CartesianStageConfig,
    CloseGripperConfig,
    load_pick_sequence_config,
    stage_is_enabled,
)


def cartesian_config():
    return CartesianStageConfig(
        speeds={"approach": 0.08, "final_grasp": 0.08, "lift": 0.10},
        maximum_segment_mm=120.0,
        position_tolerance_mm=25.0,
        arrival_stable_samples=3,
        minimum_wait_s=0.8,
        motion_timeout_s=8.0,
        workspace_x_mm=(80.0, 700.0),
        workspace_y_mm=(-450.0, 450.0),
        workspace_z_mm=(-150.0, 380.0),
    )


def test_execution_limit_is_ordered():
    assert stage_is_enabled("approach", "approach")
    assert not stage_is_enabled("approach", "final_grasp")
    assert stage_is_enabled("lift", "close_gripper")
    with pytest.raises(ValueError, match="execution_limit"):
        stage_is_enabled("unknown", "approach")


def test_cartesian_stage_uses_fresh_pose_orientation_and_server_speed():
    result = cartesian_config().validate_request(
        {
            "command_id": "approach-1",
            "type": CARTESIAN_STAGE_COMMAND_TYPE,
            "stage": "approach",
            "x": 300.0,
            "y": -30.0,
            "z": -30.0,
        },
        {"x": 295.0, "y": -29.0, "z": -8.0, "tit": 1.46, "r": 0.01, "g": 2.7},
        "approach",
    )
    assert result["serial_command"]["t"] == 1.46
    assert result["serial_command"]["r"] == 0.01
    assert result["serial_command"]["spd"] == 0.08
    with pytest.raises(ValueError, match="exceeds execution_limit"):
        cartesian_config().validate_request(
            {
                "command_id": "grasp-1",
                "type": CARTESIAN_STAGE_COMMAND_TYPE,
                "stage": "final_grasp",
                "x": 300.0,
                "y": -30.0,
                "z": -110.0,
            },
            {"x": 300.0, "y": -30.0, "z": -30.0, "tit": 1.46, "r": 0.01, "g": 2.7},
            "approach",
        )


def test_close_gripper_is_configuration_only_and_double_gated():
    config = CloseGripperConfig(55.0, 25.0, 25.0, 1.0)
    command = {"command_id": "close-1", "type": "close_gripper"}
    assert config.validate_request(command, "close_gripper") == "close-1"
    assert config.serial_command()["angle"] == 55.0
    with pytest.raises(ValueError, match="execution_limit"):
        config.validate_request(command, "final_grasp")


def test_repository_pick_configuration_loads():
    from pathlib import Path

    root = Path(__file__).resolve().parents[1]
    config = load_pick_sequence_config(
        str(root / "config" / "motion_control.json"),
        str(root / "config" / "grasp_calibration.json"),
    )
    assert config.lift_relative_z_mm == 80.0
    assert config.close_gripper.angle_deg == 55.0
    assert config.grasp_pitch_rad == pytest.approx(1.71345654)
