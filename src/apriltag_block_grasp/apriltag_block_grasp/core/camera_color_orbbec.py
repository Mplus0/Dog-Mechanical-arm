"""Minimal Orbbec color stream wrapper used by the stage-1 camera check."""

from dataclasses import dataclass
from typing import Optional

import cv2
import numpy as np
from pyorbbecsdk import Config, OBFormat, OBSensorType, Pipeline


@dataclass(frozen=True)
class ColorFrame:
    """Decoded color image and the SDK format used to produce it."""

    bgr: np.ndarray
    format_name: str


class OrbbecColorCamera:
    """Own one Orbbec color pipeline and decode frames into BGR images."""

    def __init__(self) -> None:
        self.pipeline: Optional[Pipeline] = None
        self.config: Optional[Config] = None
        self.color_profile = None

    @property
    def started(self) -> bool:
        return self.pipeline is not None

    def start(self) -> None:
        if self.started:
            raise RuntimeError("Orbbec color camera is already started")

        pipeline = Pipeline()
        config = Config()
        profiles = pipeline.get_stream_profile_list(OBSensorType.COLOR_SENSOR)
        profile = profiles.get_default_video_stream_profile()
        config.enable_stream(profile)
        pipeline.start(config)

        self.pipeline = pipeline
        self.config = config
        self.color_profile = profile

    def read(self, timeout_ms: int = 300) -> Optional[ColorFrame]:
        if self.pipeline is None:
            raise RuntimeError("Orbbec color camera is not started")

        frames = self.pipeline.wait_for_frames(int(timeout_ms))
        if frames is None:
            return None
        color_frame = frames.get_color_frame()
        if color_frame is None:
            return None
        return self.decode_color_frame(color_frame)

    def stop(self) -> None:
        pipeline = self.pipeline
        self.pipeline = None
        self.config = None
        self.color_profile = None
        if pipeline is not None:
            pipeline.stop()

    @staticmethod
    def decode_color_frame(color_frame) -> Optional[ColorFrame]:
        width = int(color_frame.get_width())
        height = int(color_frame.get_height())
        frame_format = color_frame.get_format()
        data = np.frombuffer(color_frame.get_data(), dtype=np.uint8)

        if width <= 0 or height <= 0 or data.size == 0:
            return None

        try:
            if frame_format == OBFormat.RGB:
                rgb = data.reshape((height, width, 3))
                bgr = cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
                return ColorFrame(bgr=bgr, format_name="RGB")

            if frame_format == OBFormat.BGR:
                bgr = data.reshape((height, width, 3)).copy()
                return ColorFrame(bgr=bgr, format_name="BGR")

            if frame_format == OBFormat.MJPG:
                bgr = cv2.imdecode(data, cv2.IMREAD_COLOR)
                if bgr is None:
                    return None
                return ColorFrame(bgr=bgr, format_name="MJPG")

            if frame_format == OBFormat.YUYV:
                yuyv = data.reshape((height, width, 2))
                bgr = cv2.cvtColor(yuyv, cv2.COLOR_YUV2BGR_YUYV)
                return ColorFrame(bgr=bgr, format_name="YUYV")
        except (ValueError, cv2.error):
            return None

        return None
