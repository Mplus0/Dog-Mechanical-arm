"""Unit tests for safe B-search target and transition planning."""

import pytest

from apriltag_block_grasp.core.b_search import BSearchConfig


def config():
    return BSearchConfig(
        offsets_deg=(0.0, -5.0, 5.0, -10.0, 10.0),
        speed_deg_s=10.0,
        acceleration=10.0,
        minimum_absolute_deg=-20.0,
        maximum_absolute_deg=20.0,
        maximum_single_delta_deg=10.0,
        arrival_tolerance_deg=1.5,
        arrival_stable_samples=3,
        motion_timeout_s=8.0,
    )


def test_absolute_targets_are_relative_to_measured_b0():
    assert config().absolute_targets(0.5) == (0.5, -4.5, 5.5, -9.5, 10.5)


def test_approved_search_order_uses_b0_and_five_degree_safety_transitions():
    planner = config()
    b0 = 0.5
    assert planner.route_between(b0, 0, 1) == [-4.5]
    assert planner.route_between(b0, 1, 2) == [0.5, 5.5]
    assert planner.route_between(b0, 2, 3) == [0.5, -4.5, -9.5]
    assert planner.route_between(b0, 3, 4) == [-4.5, 0.5, 5.5, 10.5]


def test_return_from_last_search_angle_uses_safe_transition():
    assert config().route_to_b0(0.5, 4) == [5.5, 0.5]


def test_recovery_route_uses_measured_feedback_instead_of_nominal_target():
    assert config().route_from_actual_to_b0(-0.439453, -8.877) == [-0.439453]


def test_default_field_gate_only_enables_offset_zero():
    assert config().maximum_automatic_search_index == 0


def test_b0_near_limit_is_rejected_before_any_motion():
    with pytest.raises(ValueError, match="exceeds the enabled absolute B range"):
        config().absolute_targets(15.0)


def test_offsets_must_start_at_zero():
    with pytest.raises(ValueError, match="start with 0"):
        BSearchConfig(
            offsets_deg=(-5.0, 5.0),
            speed_deg_s=10.0,
            acceleration=10.0,
            minimum_absolute_deg=-20.0,
            maximum_absolute_deg=20.0,
            maximum_single_delta_deg=10.0,
            arrival_tolerance_deg=1.5,
            arrival_stable_samples=3,
            motion_timeout_s=8.0,
        )


@pytest.mark.parametrize("b0", [-9.5, -3.0, 0.0, 4.0, 9.5])
def test_every_planned_nominal_segment_keeps_arrival_error_margin(b0):
    planner = config()
    targets = planner.absolute_targets(b0)
    safe_delta = planner.maximum_single_delta_deg - planner.arrival_tolerance_deg
    for current_index in range(len(targets) - 1):
        route = planner.route_between(b0, current_index, current_index + 1)
        current = targets[current_index]
        for target in route:
            assert abs(target - current) <= safe_delta
            current = target
    current = targets[-1]
    for target in planner.route_to_b0(b0, len(targets) - 1):
        assert abs(target - current) <= safe_delta
        current = target
