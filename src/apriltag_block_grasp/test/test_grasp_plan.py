"""Tests for motion-free pre-grasp point planning."""

import pytest

from apriltag_block_grasp.core.grasp_plan import PreGraspPlanConfig


def config():
    return PreGraspPlanConfig(
        final_grasp_tcp_offset_base_mm=(-24.0, 33.0, 0.5),
        pre_grasp_z_offset_mm=120.0,
        status="candidate",
        source="legacy",
    )


def test_plan_applies_fixed_tcp_offset_then_changes_only_z():
    plan = config().build({"x": 273.0, "y": -60.0, "z": -115.0})
    assert plan["final_grasp_tcp_mm"] == {"x": 249.0, "y": -27.0, "z": -114.5}
    assert plan["pre_grasp_tcp_mm"] == {"x": 249.0, "y": -27.0, "z": 5.5}
    assert plan["pre_grasp_and_final_same_xy"] is True
    assert plan["planning_only"] is True
    assert plan["cartesian_motion_command_sent"] is False


def test_invalid_object_or_offset_is_rejected():
    with pytest.raises(ValueError, match="numeric x/y/z"):
        config().build({"x": 1.0, "y": 2.0})
    with pytest.raises(ValueError, match="positive"):
        PreGraspPlanConfig((0.0, 0.0, 0.0), 0.0, "candidate", "legacy")
