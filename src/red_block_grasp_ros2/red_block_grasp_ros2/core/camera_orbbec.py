#!/usr/bin/env python3
import cv2
import numpy as np
from pyorbbecsdk import *


class OrbbecColorCamera:
    def __init__(self):
        self.pipeline = None
        self.config = None

    def start(self):
        self.pipeline = Pipeline()
        self.config = Config()
        color_profiles = self.pipeline.get_stream_profile_list(OBSensorType.COLOR_SENSOR)
        color_profile = color_profiles.get_default_video_stream_profile()
        self.config.enable_stream(color_profile)
        self.pipeline.start(self.config)
        print("Orbbec color camera started.")

    def read_bgr(self, timeout_ms=100):
        frames = self.pipeline.wait_for_frames(timeout_ms)
        if frames is None:
            return None
        color_frame = frames.get_color_frame()
        if color_frame is None:
            return None
        return self.frame_to_bgr_image(color_frame)

    def stop(self):
        if self.pipeline is not None:
            self.pipeline.stop()
            self.pipeline = None
            print("Orbbec color camera stopped.")

    @staticmethod
    def frame_to_bgr_image(color_frame):
        width = color_frame.get_width()
        height = color_frame.get_height()
        frame_format = color_frame.get_format()
        data = np.frombuffer(color_frame.get_data(), dtype=np.uint8)

        if frame_format == OBFormat.RGB:
            image = data.reshape((height, width, 3))
            return cv2.cvtColor(image, cv2.COLOR_RGB2BGR)

        if frame_format == OBFormat.BGR:
            return data.reshape((height, width, 3))

        if frame_format == OBFormat.MJPG:
            return cv2.imdecode(data, cv2.IMREAD_COLOR)

        if frame_format == OBFormat.YUYV:
            image = data.reshape((height, width, 2))
            return cv2.cvtColor(image, cv2.COLOR_YUV2BGR_YUYV)

        print("Unsupported color frame format:", frame_format)
        return None
