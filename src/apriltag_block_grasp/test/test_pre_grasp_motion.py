"""Tests for the gated, legacy-compatible pre-grasp motion protocol."""

import math

import pytest

from apriltag_block_grasp.core.pre_grasp_motion import PreGraspMotionConfig


def config():
    return PreGraspMotionConfig(
        speed=0.18,
        maximum_segment_mm=120.0,
        maximum_segment_count=3,
        position_tolerance_mm=25.0,
        arrival_stable_samples=3,
        minimum_wait_s=0.8,
        motion_timeout_s=8.0,
        workspace_x_mm=(80.0, 700.0),
        workspace_y_mm=(-450.0, 450.0),
        workspace_z_mm=(-150.0, 380.0),
    )


def test_path_segments_reserve_arrival_tolerance_from_command_limit():
    segments = config().build_segments((200.0, 0.0, 100.0), (320.0, 0.0, -6.0))
    assert len(segments) == 2
    assert segments[-1] == {"x": 320.0, "y": 0.0, "z": -6.0}
    assert math.dist((200.0, 0.0, 100.0), tuple(segments[0].values())) <= 95.0
    # Even if the previous segment is accepted 25 mm early, the next request
    # remains within the hard 120 mm driver limit.
    assert math.dist((200.0, 0.0, 100.0), tuple(segments[0].values())) + 25.0 <= 120.0


def test_request_builds_t104_and_cannot_override_motion_fields():
    request = {
        "command_id": "segment-1",
        "type": "move_pre_grasp_segment",
        "x": 300.0,
        "y": -20.0,
        "z": 10.0,
    }
    validated = config().validate_request(
        request,
        {
            "x": 250.0,
            "y": 0.0,
            "z": 80.0,
            "tit": 1.23,
            "r": -0.04,
            "g": 1.9,
        },
    )
    assert validated["serial_command"] == {
        "T": 104,
        "x": 300.0,
        "y": -20.0,
        "z": 10.0,
        "t": 1.23,
        "r": -0.04,
        "g": 1.9,
        "spd": 0.18,
    }
    assert validated["orientation_source"] == "fresh_T1051_tit_and_r"
    with pytest.raises(ValueError, match="cannot override"):
        config().validate_request(
            {**request, "t": 0.0},
            {
                "x": 250.0,
                "y": 0.0,
                "z": 80.0,
                "tit": 1.23,
                "r": -0.04,
                "g": 1.9,
            },
        )


def test_workspace_and_segment_limit_are_enforced():
    with pytest.raises(ValueError, match="outside configured workspace"):
        config().validate_workspace((300.0, 0.0, -151.0))
    with pytest.raises(ValueError, match="exceeds"):
        config().validate_request(
            {
                "command_id": "too-far",
                "type": "move_pre_grasp_segment",
                "x": 400.0,
                "y": 0.0,
                "z": 0.0,
            },
            {
                "x": 200.0,
                "y": 0.0,
                "z": 0.0,
                "tit": 1.23,
                "r": -0.04,
                "g": 1.9,
            },
        )


def test_request_requires_fresh_tool_orientation_feedback():
    with pytest.raises(ValueError, match="current_state.tit"):
        config().validate_request(
            {
                "command_id": "missing-orientation",
                "type": "move_pre_grasp_segment",
                "x": 300.0,
                "y": 0.0,
                "z": 20.0,
            },
            {"x": 250.0, "y": 0.0, "z": 80.0, "r": 0.0, "g": 1.9},
        )
