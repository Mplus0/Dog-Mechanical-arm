#!/usr/bin/env python3
"""Read-only capability check for the target RDK X5 environment.

This tool only imports libraries and inspects their APIs. It does not open the
Orbbec device, connect to the RoArm serial port, create a ROS node, or send any
motion command.
"""

import argparse
import importlib
import json
import platform
import sys
from typing import Any, Dict, List, Optional


Result = Dict[str, Any]


def result(
    name: str,
    status: str,
    detail: str,
    *,
    required: bool,
    version: Optional[str] = None,
) -> Result:
    item: Result = {
        "name": name,
        "status": status,
        "required": required,
        "detail": detail,
    }
    if version is not None:
        item["version"] = version
    return item


def import_module(name: str):
    return importlib.import_module(name)


def module_version(module: Any) -> Optional[str]:
    value = getattr(module, "__version__", None)
    return None if value is None else str(value)


def check_python() -> Result:
    version = platform.python_version()
    if sys.version_info < (3, 8):
        return result(
            "python",
            "FAIL",
            "Python 3.8 or newer is required by this package.",
            required=True,
            version=version,
        )
    return result(
        "python",
        "PASS",
        f"executable={sys.executable}; platform={platform.platform()}",
        required=True,
        version=version,
    )


def check_import(module_name: str, *, required: bool = True) -> Result:
    try:
        module = import_module(module_name)
    except Exception as exc:
        return result(
            module_name,
            "FAIL" if required else "WARN",
            f"import failed: {type(exc).__name__}: {exc}",
            required=required,
        )
    return result(
        module_name,
        "PASS",
        "import succeeded",
        required=required,
        version=module_version(module),
    )


def check_opencv() -> List[Result]:
    items: List[Result] = []
    try:
        cv2 = import_module("cv2")
    except Exception as exc:
        detail = f"import failed: {type(exc).__name__}: {exc}"
        items.append(result("opencv", "FAIL", detail, required=True))
        items.append(
            result(
                "opencv_apriltag_25h9",
                "WARN",
                "not checked because OpenCV is unavailable",
                required=False,
            )
        )
        items.append(
            result(
                "opencv_ippe_square",
                "WARN",
                "not checked because OpenCV is unavailable",
                required=False,
            )
        )
        return items

    items.append(
        result(
            "opencv",
            "PASS",
            "import succeeded",
            required=True,
            version=module_version(cv2),
        )
    )

    aruco = getattr(cv2, "aruco", None)
    dictionary_id = getattr(aruco, "DICT_APRILTAG_25h9", None) if aruco is not None else None
    dictionary_loader = getattr(aruco, "getPredefinedDictionary", None) if aruco is not None else None
    if aruco is None or dictionary_id is None or not callable(dictionary_loader):
        items.append(
            result(
                "opencv_apriltag_25h9",
                "WARN",
                "cv2.aruco DICT_APRILTAG_25h9 is unavailable; use the dedicated detector fallback",
                required=False,
            )
        )
    else:
        try:
            dictionary_loader(dictionary_id)
        except Exception as exc:
            items.append(
                result(
                    "opencv_apriltag_25h9",
                    "WARN",
                    f"dictionary creation failed: {type(exc).__name__}: {exc}",
                    required=False,
                )
            )
        else:
            items.append(
                result(
                    "opencv_apriltag_25h9",
                    "PASS",
                    "cv2.aruco DICT_APRILTAG_25h9 is available",
                    required=False,
                )
            )

    if hasattr(cv2, "solvePnP") and hasattr(cv2, "SOLVEPNP_IPPE_SQUARE"):
        items.append(
            result(
                "opencv_ippe_square",
                "PASS",
                "cv2.solvePnP and SOLVEPNP_IPPE_SQUARE are available",
                required=False,
            )
        )
    else:
        items.append(
            result(
                "opencv_ippe_square",
                "WARN",
                "IPPE Square is unavailable; the pose estimator must use its normal PnP fallback",
                required=False,
            )
        )
    return items


def check_pyorbbecsdk() -> Result:
    try:
        sdk = import_module("pyorbbecsdk")
    except Exception as exc:
        return result(
            "pyorbbecsdk",
            "FAIL",
            f"import failed: {type(exc).__name__}: {exc}",
            required=True,
        )

    required_api = ("Pipeline", "Config", "OBSensorType")
    missing = [name for name in required_api if not hasattr(sdk, name)]
    if missing:
        return result(
            "pyorbbecsdk",
            "FAIL",
            "import succeeded but required API is missing: " + ", ".join(missing),
            required=True,
            version=module_version(sdk),
        )
    return result(
        "pyorbbecsdk",
        "PASS",
        "import succeeded; device was not opened; required API is present",
        required=True,
        version=module_version(sdk),
    )


def check_roarm_interface() -> Result:
    try:
        service_module = import_module("roarm_msgs.srv")
    except Exception as exc:
        return result(
            "roarm_msgs.srv",
            "FAIL",
            f"import failed: {type(exc).__name__}: {exc}",
            required=True,
        )

    if not hasattr(service_module, "GetPoseCmd"):
        return result(
            "roarm_msgs.srv.GetPoseCmd",
            "FAIL",
            "roarm_msgs.srv imports, but GetPoseCmd is unavailable",
            required=True,
        )
    return result(
        "roarm_msgs.srv.GetPoseCmd",
        "PASS",
        "service interface is available",
        required=True,
    )


def collect_results() -> List[Result]:
    checks: List[Result] = [check_python()]
    checks.append(check_import("numpy"))
    checks.extend(check_opencv())
    checks.append(check_pyorbbecsdk())
    checks.append(check_import("rclpy"))
    checks.append(check_import("std_msgs.msg"))
    checks.append(check_import("ament_index_python"))
    checks.append(check_roarm_interface())
    return checks


def build_report(checks: List[Result]) -> Result:
    failed_required = [
        item["name"]
        for item in checks
        if item["required"] and item["status"] == "FAIL"
    ]
    warnings = [item["name"] for item in checks if item["status"] == "WARN"]
    return {
        "tool": "apriltag_block_grasp.check_environment",
        "read_only": True,
        "camera_opened": False,
        "arm_connected": False,
        "checks": checks,
        "summary": {
            "ready": not failed_required,
            "failed_required": failed_required,
            "warnings": warnings,
        },
    }


def print_human(report: Result) -> None:
    print("AprilTag block grasp environment check")
    print("Safety: read-only; camera not opened; arm not connected")
    print()
    for item in report["checks"]:
        required = "required" if item["required"] else "fallback allowed"
        version = f" version={item['version']}" if item.get("version") else ""
        print(f"[{item['status']}] {item['name']} ({required}){version}")
        print(f"       {item['detail']}")
    print()
    summary = report["summary"]
    print("READY=" + ("yes" if summary["ready"] else "no"))
    if summary["failed_required"]:
        print("FAILED_REQUIRED=" + ",".join(summary["failed_required"]))
    if summary["warnings"]:
        print("WARNINGS=" + ",".join(summary["warnings"]))


def parse_args(argv: Optional[List[str]] = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Check AprilTag grasp dependencies without opening hardware."
    )
    parser.add_argument(
        "--json",
        action="store_true",
        help="print a machine-readable JSON report",
    )
    return parser.parse_args(argv)


def main(argv: Optional[List[str]] = None) -> int:
    args = parse_args(argv)
    report = build_report(collect_results())
    if args.json:
        print(json.dumps(report, ensure_ascii=False, indent=2))
    else:
        print_human(report)
    return 0 if report["summary"]["ready"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
