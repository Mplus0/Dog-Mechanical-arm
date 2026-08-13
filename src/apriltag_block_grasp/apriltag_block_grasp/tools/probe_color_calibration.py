#!/usr/bin/env python3
"""Probe Orbbec color calibration APIs without depth, PnP or arm access."""

import json
from typing import Any, Dict, Optional, Tuple

import numpy as np

from apriltag_block_grasp.core.camera_color_orbbec import OrbbecColorCamera


ProbeResult = Dict[str, Any]


def finite_float(value: Any) -> float:
    number = float(value)
    if not np.isfinite(number):
        raise ValueError(f"non-finite calibration value: {number}")
    return number


def optional_int(value: Any) -> Optional[int]:
    if value is None:
        return None
    return int(value)


def serialize_intrinsic(intrinsic: Any) -> ProbeResult:
    fx = finite_float(getattr(intrinsic, "fx"))
    fy = finite_float(getattr(intrinsic, "fy"))
    cx = finite_float(getattr(intrinsic, "cx"))
    cy = finite_float(getattr(intrinsic, "cy"))
    if fx <= 0.0 or fy <= 0.0:
        raise ValueError(f"focal length must be positive: fx={fx}, fy={fy}")
    width = optional_int(getattr(intrinsic, "width", None))
    height = optional_int(getattr(intrinsic, "height", None))
    return {
        "width": width,
        "height": height,
        "fx": fx,
        "fy": fy,
        "cx": cx,
        "cy": cy,
        "camera_matrix": [
            [fx, 0.0, cx],
            [0.0, fy, cy],
            [0.0, 0.0, 1.0],
        ],
    }


def serialize_distortion(distortion: Any) -> ProbeResult:
    # OpenCV expects radial/tangential order k1, k2, p1, p2, k3.
    k1 = finite_float(getattr(distortion, "k1"))
    k2 = finite_float(getattr(distortion, "k2"))
    p1 = finite_float(getattr(distortion, "p1"))
    p2 = finite_float(getattr(distortion, "p2"))
    k3 = finite_float(getattr(distortion, "k3"))
    model = getattr(distortion, "model", None)
    return {
        "model": None if model is None else str(model),
        "k1": k1,
        "k2": k2,
        "p1": p1,
        "p2": p2,
        "k3": k3,
        "opencv_coefficients": [k1, k2, p1, p2, k3],
    }


def attempt(name: str, callback) -> Tuple[ProbeResult, Optional[Any]]:
    try:
        value = callback()
    except Exception as exc:
        return {
            "name": name,
            "status": "FAIL",
            "error": f"{type(exc).__name__}: {exc}",
        }, None
    return {"name": name, "status": "PASS"}, value


def probe_profile(camera: OrbbecColorCamera) -> ProbeResult:
    result: ProbeResult = {
        "source": "color_profile",
        "status": "FAIL",
        "attempts": [],
    }
    profile = camera.color_profile
    if profile is None:
        result["error"] = "active color profile is unavailable"
        return result

    video_attempt, video_profile = attempt(
        "color_profile.as_video_stream_profile",
        lambda: profile.as_video_stream_profile(),
    )
    result["attempts"].append(video_attempt)
    if video_profile is None:
        return result

    intrinsic_attempt, intrinsic = attempt(
        "video_profile.get_intrinsic",
        lambda: video_profile.get_intrinsic(),
    )
    result["attempts"].append(intrinsic_attempt)
    if intrinsic is not None:
        try:
            result["intrinsic"] = serialize_intrinsic(intrinsic)
        except Exception as exc:
            intrinsic_attempt["status"] = "FAIL"
            intrinsic_attempt["error"] = f"serialization failed: {type(exc).__name__}: {exc}"

    distortion_attempt, distortion = attempt(
        "video_profile.get_distortion",
        lambda: video_profile.get_distortion(),
    )
    result["attempts"].append(distortion_attempt)
    if distortion is not None:
        try:
            result["distortion"] = serialize_distortion(distortion)
        except Exception as exc:
            distortion_attempt["status"] = "FAIL"
            distortion_attempt["error"] = f"serialization failed: {type(exc).__name__}: {exc}"

    if "intrinsic" in result and "distortion" in result:
        result["status"] = "PASS"
    elif "intrinsic" in result:
        result["status"] = "PARTIAL"
    return result


def probe_pipeline(camera: OrbbecColorCamera) -> ProbeResult:
    result: ProbeResult = {
        "source": "pipeline_camera_param",
        "status": "FAIL",
        "attempts": [],
    }
    if camera.pipeline is None:
        result["error"] = "active pipeline is unavailable"
        return result

    param_attempt, camera_param = attempt(
        "pipeline.get_camera_param",
        lambda: camera.pipeline.get_camera_param(),
    )
    result["attempts"].append(param_attempt)
    if camera_param is None:
        return result

    intrinsic_attempt, intrinsic = attempt(
        "camera_param.rgb_intrinsic",
        lambda: getattr(camera_param, "rgb_intrinsic"),
    )
    result["attempts"].append(intrinsic_attempt)
    if intrinsic is not None:
        try:
            result["intrinsic"] = serialize_intrinsic(intrinsic)
        except Exception as exc:
            intrinsic_attempt["status"] = "FAIL"
            intrinsic_attempt["error"] = f"serialization failed: {type(exc).__name__}: {exc}"

    distortion_attempt, distortion = attempt(
        "camera_param.rgb_distortion",
        lambda: getattr(camera_param, "rgb_distortion"),
    )
    result["attempts"].append(distortion_attempt)
    if distortion is not None:
        try:
            result["distortion"] = serialize_distortion(distortion)
        except Exception as exc:
            distortion_attempt["status"] = "FAIL"
            distortion_attempt["error"] = f"serialization failed: {type(exc).__name__}: {exc}"

    if "intrinsic" in result and "distortion" in result:
        result["status"] = "PASS"
    elif "intrinsic" in result:
        result["status"] = "PARTIAL"
    return result


def resolution_matches(intrinsic: ProbeResult, frame_width: int, frame_height: int) -> Optional[bool]:
    width = intrinsic.get("width")
    height = intrinsic.get("height")
    if width is None or height is None:
        return None
    return int(width) == int(frame_width) and int(height) == int(frame_height)


def main() -> int:
    camera = OrbbecColorCamera()
    report: ProbeResult = {
        "tool": "apriltag_block_grasp.probe_color_calibration",
        "read_only": True,
        "depth_enabled": False,
        "pnp_enabled": False,
        "arm_connected": False,
        "motion_commands_enabled": False,
        "frame": None,
        "sources": [],
        "summary": {"ready_for_pnp": False},
    }
    try:
        camera.start()
        frame = None
        for _ in range(10):
            frame = camera.read(timeout_ms=500)
            if frame is not None:
                break
        if frame is None:
            report["summary"]["error"] = "no valid color frame received"
            print(json.dumps(report, ensure_ascii=False, indent=2))
            return 1

        height, width = frame.bgr.shape[:2]
        report["frame"] = {
            "width": int(width),
            "height": int(height),
            "format": frame.format_name,
        }

        sources = [probe_profile(camera), probe_pipeline(camera)]
        report["sources"] = sources
        usable_sources = []
        for source in sources:
            if source.get("status") != "PASS":
                continue
            source["resolution_matches_frame"] = resolution_matches(
                source["intrinsic"], width, height
            )
            if source["resolution_matches_frame"] is not False:
                usable_sources.append(source["source"])

        report["summary"] = {
            "ready_for_pnp": bool(usable_sources),
            "usable_sources": usable_sources,
            "require_yaml_override": not bool(usable_sources),
        }
        print(json.dumps(report, ensure_ascii=False, indent=2))
        return 0 if usable_sources else 1
    except Exception as exc:
        report["summary"] = {
            "ready_for_pnp": False,
            "require_yaml_override": True,
            "error": f"{type(exc).__name__}: {exc}",
        }
        print(json.dumps(report, ensure_ascii=False, indent=2))
        return 1
    finally:
        try:
            camera.stop()
            print("Orbbec color stream stopped.", flush=True)
        except Exception as exc:
            print(f"Failed to stop camera cleanly: {exc}", flush=True)


if __name__ == "__main__":
    raise SystemExit(main())
