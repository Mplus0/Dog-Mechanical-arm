from pathlib import Path

import pytest

from red_block_grasp_mplus0.core.place_pose_calibration import (
    apply_pose,
    load_document,
    pose_from_document,
    save_document,
    validate_pose,
)


VALID_POSE = {
    "place_x_mm": 260.0,
    "place_y_mm": 180.0,
    "place_z_mm": 120.0,
}


def write_config(path: Path) -> None:
    path.write_text(
        "visual_servo_task_node:\n"
        "  ros__parameters:\n"
        "    place_x_mm: 260.0\n"
        "    place_y_mm: 180.0\n"
        "    place_z_mm: 120.0\n"
        "    place_speed: 0.1\n",
        encoding="utf-8",
    )


def test_loads_fixed_pose(tmp_path: Path) -> None:
    path = tmp_path / "place_pose.yaml"
    write_config(path)
    assert pose_from_document(load_document(path)) == VALID_POSE


@pytest.mark.parametrize(
    "key,value",
    [
        ("place_x_mm", float("nan")),
        ("place_x_mm", 79.9),
        ("place_y_mm", 450.1),
        ("place_z_mm", -30.1),
    ],
)
def test_rejects_nonfinite_or_out_of_workspace(key: str, value: float) -> None:
    pose = dict(VALID_POSE)
    pose[key] = value
    with pytest.raises(ValueError):
        validate_pose(pose)


def test_updates_only_pose_and_saves_backup(tmp_path: Path) -> None:
    path = tmp_path / "place_pose.yaml"
    write_config(path)
    document = load_document(path)
    candidate = {
        "place_x_mm": 300.0,
        "place_y_mm": -25.0,
        "place_z_mm": 140.0,
    }
    apply_pose(document, candidate)
    backup = save_document(path, document)

    assert backup.is_file()
    assert "place_x_mm: 260.0" in backup.read_text(encoding="utf-8")
    saved = load_document(path)
    assert pose_from_document(saved) == candidate
    assert saved["visual_servo_task_node"]["ros__parameters"]["place_speed"] == 0.1


def test_interactive_tool_has_no_command_publisher() -> None:
    tool = (
        Path(__file__).parents[1]
        / "red_block_grasp_mplus0"
        / "tools"
        / "calibrate_place_pose.py"
    ).read_text(encoding="utf-8")
    assert "create_publisher" not in tool
    assert 'create_subscription(String, topic, self._on_state, 10)' in tool
