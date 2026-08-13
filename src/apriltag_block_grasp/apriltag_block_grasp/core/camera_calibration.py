"""Color-camera calibration types and Orbbec SDK extraction."""

from dataclasses import dataclass
from typing import Any, Optional

import numpy as np


@dataclass(frozen=True)
class ColorCameraCalibration:
    width: Optional[int]
    height: Optional[int]
    camera_matrix: np.ndarray
    distortion_coefficients: np.ndarray
    source: str

    def matches_frame(self, width: int, height: int) -> bool:
        if self.width is None or self.height is None:
            return True
        return self.width == int(width) and self.height == int(height)


def _finite_float(value: Any) -> float:
    number = float(value)
    if not np.isfinite(number):
        raise ValueError(f"non-finite calibration value: {number}")
    return number


def _optional_int(value: Any) -> Optional[int]:
    return None if value is None else int(value)


def build_color_calibration(intrinsic: Any, distortion: Any, source: str) -> ColorCameraCalibration:
    fx = _finite_float(getattr(intrinsic, "fx"))
    fy = _finite_float(getattr(intrinsic, "fy"))
    cx = _finite_float(getattr(intrinsic, "cx"))
    cy = _finite_float(getattr(intrinsic, "cy"))
    if fx <= 0.0 or fy <= 0.0:
        raise ValueError(f"focal length must be positive: fx={fx}, fy={fy}")

    camera_matrix = np.array(
        [[fx, 0.0, cx], [0.0, fy, cy], [0.0, 0.0, 1.0]],
        dtype=np.float64,
    )
    distortion_coefficients = np.array(
        [
            _finite_float(getattr(distortion, "k1")),
            _finite_float(getattr(distortion, "k2")),
            _finite_float(getattr(distortion, "p1")),
            _finite_float(getattr(distortion, "p2")),
            _finite_float(getattr(distortion, "k3")),
        ],
        dtype=np.float64,
    )
    return ColorCameraCalibration(
        width=_optional_int(getattr(intrinsic, "width", None)),
        height=_optional_int(getattr(intrinsic, "height", None)),
        camera_matrix=camera_matrix,
        distortion_coefficients=distortion_coefficients,
        source=str(source),
    )


def read_orbbec_color_calibration(camera) -> ColorCameraCalibration:
    """Read active-profile calibration, then use camera_param as fallback."""

    errors = []
    profile = getattr(camera, "color_profile", None)
    if profile is not None:
        try:
            video_profile = profile.as_video_stream_profile()
            return build_color_calibration(
                video_profile.get_intrinsic(),
                video_profile.get_distortion(),
                "color_profile",
            )
        except Exception as exc:
            errors.append(f"color_profile: {type(exc).__name__}: {exc}")

    pipeline = getattr(camera, "pipeline", None)
    if pipeline is not None:
        try:
            camera_param = pipeline.get_camera_param()
            return build_color_calibration(
                camera_param.rgb_intrinsic,
                camera_param.rgb_distortion,
                "pipeline_camera_param",
            )
        except Exception as exc:
            errors.append(f"pipeline_camera_param: {type(exc).__name__}: {exc}")

    raise RuntimeError("color calibration unavailable; " + "; ".join(errors))
