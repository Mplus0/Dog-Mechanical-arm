"""Minimal Orbbec RGBD stream wrapper for read-only depth probing."""

from dataclasses import dataclass
from typing import Optional

import numpy as np
from pyorbbecsdk import Config, OBAlignMode, OBSensorType, Pipeline

from apriltag_block_grasp.core.camera_color_orbbec import (
    ColorFrame,
    OrbbecColorCamera,
)


@dataclass(frozen=True)
class RgbdFrame:
    """Decoded, synchronized color and depth frames."""

    color: ColorFrame
    depth_mm: np.ndarray
    depth_scale_mm: float


class OrbbecRgbdCamera:
    """Own one color/depth pipeline with requested software D2C alignment."""

    def __init__(self) -> None:
        self.pipeline: Optional[Pipeline] = None
        self.config: Optional[Config] = None
        self.color_profile = None
        self.depth_profile = None
        self.alignment_requested = False
        self.alignment_error: Optional[str] = None

    @property
    def started(self) -> bool:
        return self.pipeline is not None

    def start(self) -> None:
        if self.started:
            raise RuntimeError("Orbbec RGBD camera is already started")

        pipeline = Pipeline()
        config = Config()
        color_profiles = pipeline.get_stream_profile_list(OBSensorType.COLOR_SENSOR)
        depth_profiles = pipeline.get_stream_profile_list(OBSensorType.DEPTH_SENSOR)
        color_profile = color_profiles.get_default_video_stream_profile()
        depth_profile = depth_profiles.get_default_video_stream_profile()
        config.enable_stream(color_profile)
        config.enable_stream(depth_profile)

        self.alignment_requested = False
        self.alignment_error = None
        try:
            config.set_align_mode(OBAlignMode.SW_MODE)
            self.alignment_requested = True
        except Exception as exc:
            self.alignment_error = f"{type(exc).__name__}: {exc}"

        pipeline.start(config)
        self.pipeline = pipeline
        self.config = config
        self.color_profile = color_profile
        self.depth_profile = depth_profile

    def read(self, timeout_ms: int = 500) -> Optional[RgbdFrame]:
        if self.pipeline is None:
            raise RuntimeError("Orbbec RGBD camera is not started")

        frames = self.pipeline.wait_for_frames(int(timeout_ms))
        if frames is None:
            return None
        color_frame = frames.get_color_frame()
        depth_frame = frames.get_depth_frame()
        if color_frame is None or depth_frame is None:
            return None

        color = OrbbecColorCamera.decode_color_frame(color_frame)
        depth_result = self.decode_depth_frame(depth_frame)
        if color is None or depth_result is None:
            return None
        depth_mm, depth_scale_mm = depth_result
        return RgbdFrame(
            color=color,
            depth_mm=depth_mm,
            depth_scale_mm=depth_scale_mm,
        )

    def stop(self) -> None:
        pipeline = self.pipeline
        self.pipeline = None
        self.config = None
        self.color_profile = None
        self.depth_profile = None
        if pipeline is not None:
            pipeline.stop()

    @staticmethod
    def decode_depth_frame(depth_frame):
        width = int(depth_frame.get_width())
        height = int(depth_frame.get_height())
        data = np.frombuffer(depth_frame.get_data(), dtype=np.uint16)
        if width <= 0 or height <= 0 or data.size != width * height:
            return None

        scale_mm = 1.0
        if hasattr(depth_frame, "get_depth_scale"):
            scale_mm = float(depth_frame.get_depth_scale())
        elif hasattr(depth_frame, "get_value_scale"):
            scale_mm = float(depth_frame.get_value_scale())
        if not np.isfinite(scale_mm) or scale_mm <= 0.0:
            return None

        raw = data.reshape((height, width))
        return raw.astype(np.float32) * scale_mm, scale_mm
