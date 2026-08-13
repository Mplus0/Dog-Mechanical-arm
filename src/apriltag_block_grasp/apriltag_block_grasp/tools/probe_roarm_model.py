#!/usr/bin/env python3
"""Inspect installed RoArm model and MoveIt files without starting ROS nodes."""

import json
import os
from pathlib import Path
import re
from typing import Any, Dict, List, Optional
import xml.etree.ElementTree as ET

from ament_index_python.packages import (
    PackageNotFoundError,
    get_package_prefix,
    get_package_share_directory,
)


TEXT_SUFFIXES = {".urdf", ".xacro", ".srdf", ".yaml", ".yml", ".json", ".py"}
MODEL_SUFFIXES = {".urdf", ".xacro", ".srdf"}
CONFIG_NAME_PATTERN = re.compile(
    r"(kinematic|joint.?limit|controller|moveit|semantic|srdf|urdf)", re.IGNORECASE
)


def local_tag(element: ET.Element) -> str:
    return str(element.tag).split("}")[-1]


def child_by_tag(element: ET.Element, name: str) -> Optional[ET.Element]:
    for child in element:
        if local_tag(child) == name:
            return child
    return None


def attributes_or_none(element: Optional[ET.Element]) -> Optional[Dict[str, str]]:
    if element is None:
        return None
    return {str(key): str(value) for key, value in element.attrib.items()}


def parse_model_file(path: Path) -> Dict[str, Any]:
    result: Dict[str, Any] = {
        "path": str(path),
        "parse_valid": False,
        "robot_name": None,
        "links": [],
        "joints": [],
    }
    try:
        root = ET.parse(path).getroot()
    except Exception as exc:
        result["parse_error"] = f"{type(exc).__name__}: {exc}"
        return result
    result["parse_valid"] = True
    result["robot_name"] = root.attrib.get("name")
    for element in root.iter():
        tag = local_tag(element)
        if tag == "link":
            name = element.attrib.get("name")
            if name:
                result["links"].append(str(name))
        elif tag == "joint":
            parent = child_by_tag(element, "parent")
            child = child_by_tag(element, "child")
            origin = child_by_tag(element, "origin")
            axis = child_by_tag(element, "axis")
            limit = child_by_tag(element, "limit")
            result["joints"].append(
                {
                    "name": element.attrib.get("name"),
                    "type": element.attrib.get("type"),
                    "parent": None if parent is None else parent.attrib.get("link"),
                    "child": None if child is None else child.attrib.get("link"),
                    "origin": attributes_or_none(origin),
                    "axis": attributes_or_none(axis),
                    "limit": attributes_or_none(limit),
                }
            )
    return result


def relevant_text_lines(path: Path, maximum_matches: int = 80) -> List[Dict[str, Any]]:
    patterns = re.compile(
        r"(joint_limits|kinematics_solver|planning_group|planning_frame|"
        r"base_link|hand_link|eef|end_effector|roarm_m3|group:|controller)",
        re.IGNORECASE,
    )
    matches: List[Dict[str, Any]] = []
    try:
        if path.stat().st_size > 2_000_000:
            return matches
        with path.open("r", encoding="utf-8", errors="replace") as stream:
            for line_number, line in enumerate(stream, start=1):
                text = line.rstrip()
                if patterns.search(text):
                    matches.append({"line": line_number, "text": text[:500]})
                    if len(matches) >= maximum_matches:
                        break
    except OSError:
        pass
    return matches


def inspect_package(package_name: str) -> Dict[str, Any]:
    try:
        prefix = Path(get_package_prefix(package_name)).resolve()
        share = Path(get_package_share_directory(package_name)).resolve()
    except PackageNotFoundError as exc:
        return {
            "package": package_name,
            "available": False,
            "error": f"{type(exc).__name__}: {exc}",
        }
    files = sorted(
        path
        for path in share.rglob("*")
        if path.is_file() and path.suffix.lower() in TEXT_SUFFIXES
    )
    model_files = [path for path in files if path.suffix.lower() in MODEL_SUFFIXES]
    config_files = [
        path
        for path in files
        if path.suffix.lower() in {".yaml", ".yml", ".json", ".srdf"}
        and CONFIG_NAME_PATTERN.search(path.name)
    ]
    return {
        "package": package_name,
        "available": True,
        "prefix": str(prefix),
        "share_directory": str(share),
        "text_file_count": len(files),
        "model_files": [parse_model_file(path) for path in model_files],
        "relevant_config_files": [
            {
                "path": str(path),
                "matching_lines": relevant_text_lines(path),
            }
            for path in config_files
        ],
    }


def main() -> int:
    report: Dict[str, Any] = {
        "tool": "apriltag_block_grasp.probe_roarm_model",
        "read_only": True,
        "ros_node_created": False,
        "serial_opened": False,
        "camera_opened": False,
        "service_clients_created": False,
        "service_requests_sent": 0,
        "motion_commands_enabled": False,
        "motion_command_sent": False,
        "roarm_model_environment": os.environ.get("ROARM_MODEL"),
        "packages": [],
        "summary": {"valid": False},
    }
    try:
        packages = [
            inspect_package("roarm_description"),
            inspect_package("roarm_moveit"),
        ]
        report["packages"] = packages
        available = [item for item in packages if item.get("available")]
        model_file_count = sum(
            len(item.get("model_files", [])) for item in available
        )
        parsed_model_count = sum(
            1
            for item in available
            for model in item.get("model_files", [])
            if model.get("parse_valid")
        )
        report["summary"] = {
            "valid": len(available) == 2 and model_file_count > 0,
            "available_package_count": len(available),
            "model_file_count": model_file_count,
            "parsed_model_file_count": parsed_model_count,
            "finite_numeric_assumptions_added": False,
            "motion_command_sent": False,
        }
    except Exception as exc:
        report["summary"] = {
            "valid": False,
            "error": f"{type(exc).__name__}: {exc}",
            "motion_command_sent": False,
        }
    print(json.dumps(report, ensure_ascii=False, indent=2))
    return 0 if report["summary"].get("valid", False) else 1


if __name__ == "__main__":
    raise SystemExit(main())
