#!/usr/bin/env python3
# 中文说明：
# Orbbec RGBD 相机模块。
# 负责同时打开彩色图和深度图，并输出：
# 1. OpenCV BGR 图像
# 2. depth_mm 深度图，单位 mm
# 3. camera_matrix 相机内参矩阵

import cv2
import numpy as np
from pyorbbecsdk import *


class OrbbecRgbdCamera:
    def __init__(self):
        self.pipeline = None
        self.config = None
        self.camera_matrix = None

    def start(self):
        self.pipeline = Pipeline()
        self.config = Config()

        color_profiles = self.pipeline.get_stream_profile_list(OBSensorType.COLOR_SENSOR)
        depth_profiles = self.pipeline.get_stream_profile_list(OBSensorType.DEPTH_SENSOR)

        color_profile = color_profiles.get_default_video_stream_profile()
        depth_profile = depth_profiles.get_default_video_stream_profile()

        self.config.enable_stream(color_profile)
        self.config.enable_stream(depth_profile)

        try:
            self.config.set_align_mode(OBAlignMode.SW_MODE)
            print("Depth align mode: SW_MODE")
        except Exception as e:
            print("WARNING: cannot enable SW depth alignment:", e)

        self.pipeline.start(self.config)

        intrinsic = color_profile.as_video_stream_profile().get_intrinsic()
        self.camera_matrix = self.get_camera_matrix(intrinsic)

        print("Orbbec RGBD camera started.")

    def read(self, timeout_ms=1000):
        if self.pipeline is None:
            raise RuntimeError("Camera is not started.")

        frames = self.pipeline.wait_for_frames(timeout_ms)
        if frames is None:
            return None, None, None

        color_frame = frames.get_color_frame()
        depth_frame = frames.get_depth_frame()

        if color_frame is None or depth_frame is None:
            return None, None, None

        bgr = self.frame_to_bgr_image(color_frame)
        depth_mm = self.depth_frame_to_mm(depth_frame)

        if bgr is None or depth_mm is None:
            return None, None, None

        return bgr, depth_mm, self.camera_matrix

    def stop(self):
        if self.pipeline is not None:
            self.pipeline.stop()
            self.pipeline = None
            print("Orbbec RGBD camera stopped.")

    @staticmethod
    def get_camera_matrix(intrinsic):
        fx = float(getattr(intrinsic, "fx"))
        fy = float(getattr(intrinsic, "fy"))
        cx = float(getattr(intrinsic, "cx"))
        cy = float(getattr(intrinsic, "cy"))

        return np.array(
            [
                [fx, 0.0, cx],
                [0.0, fy, cy],
                [0.0, 0.0, 1.0],
            ],
            dtype=np.float64,
        )

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

    @staticmethod
    def depth_frame_to_mm(depth_frame):
        width = depth_frame.get_width()
        height = depth_frame.get_height()
        data = np.frombuffer(depth_frame.get_data(), dtype=np.uint16).reshape((height, width))

        scale = 1.0
        if hasattr(depth_frame, "get_depth_scale"):
            scale = float(depth_frame.get_depth_scale())
        elif hasattr(depth_frame, "get_value_scale"):
            scale = float(depth_frame.get_value_scale())

        return data.astype(np.float32) * scale
