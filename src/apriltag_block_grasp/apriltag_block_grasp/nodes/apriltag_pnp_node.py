#!/usr/bin/env python3
"""Stage-2A AprilTag PnP node without depth or robot-arm access."""

import json
import time
from typing import Optional

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from apriltag_block_grasp.core.apriltag_detector import (
    OpenCvAprilTag25h9Detector,
    draw_detections,
)
from apriltag_block_grasp.core.camera_calibration import read_orbbec_color_calibration
from apriltag_block_grasp.core.camera_color_orbbec import OrbbecColorCamera
from apriltag_block_grasp.core.pose_estimator import (
    AprilTagPoseEstimator,
    rotation_matrix_to_quaternion_xyzw,
)


class AprilTagPnpNode(Node):
    def __init__(self) -> None:
        super().__init__("apriltag_pnp_node")
        self.declare_parameter("allowed_ids", [0, 1])
        self.declare_parameter("tag_size_mm", 38.9)
        self.declare_parameter("timer_period_s", 0.10)
        self.declare_parameter("frame_timeout_ms", 300)
        self.declare_parameter("status_period_s", 2.0)
        self.declare_parameter("show_window", False)
        self.declare_parameter("axis_length_mm", 20.0)
        self.declare_parameter("distortion_mode", "rectified_zero_distortion")

        allowed_ids = [int(value) for value in self.get_parameter("allowed_ids").value]
        self.tag_size_mm = float(self.get_parameter("tag_size_mm").value)
        self.timer_period_s = max(0.01, float(self.get_parameter("timer_period_s").value))
        self.frame_timeout_ms = max(1, int(self.get_parameter("frame_timeout_ms").value))
        self.status_period_s = max(0.1, float(self.get_parameter("status_period_s").value))
        self.show_window = self.parse_bool(self.get_parameter("show_window").value)
        self.axis_length_mm = max(1.0, float(self.get_parameter("axis_length_mm").value))
        self.distortion_mode = str(self.get_parameter("distortion_mode").value)

        self.publisher = self.create_publisher(String, "/apriltag_grasp/pnp", 10)
        self.camera = OrbbecColorCamera()
        self.detector = OpenCvAprilTag25h9Detector(allowed_ids=allowed_ids)
        self.started_at = time.monotonic()
        self.last_status_time = 0.0
        self.frame_count = 0
        self.pnp_success_count = 0
        self.pnp_failure_count = 0
        self.empty_frame_count = 0
        self.window_name = "AprilTag stage 2A - PnP"

        self.get_logger().info(
            "Starting color-only AprilTag PnP. Depth, hand-eye and arm access are disabled."
        )
        try:
            self.camera.start()
            self.calibration = read_orbbec_color_calibration(self.camera)
            self.pose_estimator = AprilTagPoseEstimator(
                self.tag_size_mm,
                self.calibration,
                distortion_mode=self.distortion_mode,
            )
        except Exception:
            self.camera.stop()
            raise
        self.get_logger().info(
            f"Calibration source={self.calibration.source} "
            f"resolution={self.calibration.width}x{self.calibration.height} "
            f"tag_size_mm={self.tag_size_mm} "
            f"distortion_mode={self.pose_estimator.distortion_mode}"
        )
        self.timer = self.create_timer(self.timer_period_s, self.on_timer)

    @staticmethod
    def parse_bool(value) -> bool:
        if isinstance(value, bool):
            return value
        return str(value).strip().lower() in ("1", "true", "yes", "on")

    def on_timer(self) -> None:
        try:
            frame = self.camera.read(timeout_ms=self.frame_timeout_ms)
        except Exception as exc:
            self.pnp_failure_count += 1
            self.publish([], False, f"camera_read_failed: {type(exc).__name__}: {exc}", None)
            return
        if frame is None:
            self.empty_frame_count += 1
            self.publish([], False, "empty_or_unsupported_frame", None)
            return

        image = frame.bgr
        height, width = image.shape[:2]
        self.frame_count += 1
        if not self.calibration.matches_frame(width, height):
            self.publish([], False, "calibration_resolution_mismatch", frame)
            return

        batch = self.detector.detect(image)
        results = []
        poses_by_id = {}
        for detection in batch.detections:
            try:
                pose = self.pose_estimator.estimate(detection)
                quaternion = rotation_matrix_to_quaternion_xyzw(pose.rotation_matrix)
                results.append(
                    {
                        "tag_id": detection.tag_id,
                        "valid": True,
                        "reason": "ok",
                        "method": pose.method,
                        "center": {"u": detection.center[0], "v": detection.center[1]},
                        "camera_tag_mm": {
                            "x": float(pose.translation_mm[0]),
                            "y": float(pose.translation_mm[1]),
                            "z": float(pose.translation_mm[2]),
                        },
                        "rvec": [float(value) for value in pose.rvec],
                        "rotation_matrix": pose.rotation_matrix.tolist(),
                        "quaternion_xyzw": [float(value) for value in quaternion],
                        "reprojection_error_px": pose.reprojection_error_px,
                        "area_px2": detection.area_px2,
                    }
                )
                poses_by_id[detection.tag_id] = pose
                self.pnp_success_count += 1
            except Exception as exc:
                results.append(
                    {
                        "tag_id": detection.tag_id,
                        "valid": False,
                        "reason": f"pnp_failed: {type(exc).__name__}: {exc}",
                        "center": {"u": detection.center[0], "v": detection.center[1]},
                        "area_px2": detection.area_px2,
                    }
                )
                self.pnp_failure_count += 1

        valid_result_count = sum(1 for item in results if item.get("valid"))
        if not results:
            frame_valid = True
            reason = "no_allowed_tag"
        elif valid_result_count == 0:
            frame_valid = False
            reason = "all_pnp_failed"
        elif valid_result_count < len(results):
            frame_valid = True
            reason = "partial_pnp_success"
        else:
            frame_valid = True
            reason = "ok"
        self.publish(results, frame_valid, reason, frame)

        if self.show_window:
            display = draw_detections(image, list(batch.detections))
            for detection in batch.detections:
                pose = poses_by_id.get(detection.tag_id)
                if pose is None:
                    continue
                cv2.drawFrameAxes(
                    display,
                    self.calibration.camera_matrix,
                    self.pose_estimator.distortion_coefficients,
                    pose.rvec.reshape(3, 1),
                    pose.translation_mm.reshape(3, 1),
                    self.axis_length_mm,
                    2,
                )
                center = tuple(np.rint(detection.center).astype(np.int32))
                cv2.putText(
                    display,
                    f"z={pose.translation_mm[2]:.1f}mm err={pose.reprojection_error_px:.2f}px",
                    (center[0] + 8, center[1] + 24),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.52,
                    (0, 255, 255),
                    2,
                    cv2.LINE_AA,
                )
            cv2.imshow(self.window_name, display)
            key = cv2.waitKey(1) & 0xFF
            if key in (ord("q"), 27):
                rclpy.shutdown()

    def average_fps(self) -> float:
        return self.frame_count / max(time.monotonic() - self.started_at, 1e-6)

    def publish(self, detections, valid: bool, reason: str, frame) -> None:
        payload = {
            "stamp": time.time(),
            "frame_id": "camera_color_opencv",
            "valid": bool(valid),
            "reason": reason,
            "family": "tag25h9",
            "tag_size_mm": self.tag_size_mm,
            "tag_coordinate_convention": {
                "handedness": "right",
                "x": "printed_left_to_right",
                "y": "printed_top_to_bottom",
                "z": "printed_front_to_back_away_from_observer",
            },
            "calibration_source": self.calibration.source,
            "pnp_distortion_mode": self.pose_estimator.distortion_mode,
            "pnp_distortion_coefficients": (
                self.pose_estimator.distortion_coefficients.reshape(-1).tolist()
            ),
            "calibration_resolution": {
                "width": self.calibration.width,
                "height": self.calibration.height,
            },
            "count": len(detections),
            "detections": detections,
            "average_fps": self.average_fps(),
            "frame_count": self.frame_count,
            "pnp_success_count": self.pnp_success_count,
            "pnp_failure_count": self.pnp_failure_count,
            "empty_frame_count": self.empty_frame_count,
            "depth_enabled": False,
            "handeye_enabled": False,
            "arm_connected": False,
            "motion_commands_enabled": False,
        }
        if frame is not None:
            payload["width"] = int(frame.bgr.shape[1])
            payload["height"] = int(frame.bgr.shape[0])
            payload["format"] = frame.format_name
        message = String()
        message.data = json.dumps(payload, ensure_ascii=False)
        self.publisher.publish(message)

        now = time.monotonic()
        if now - self.last_status_time >= self.status_period_s:
            self.last_status_time = now
            valid_items = [item for item in detections if item.get("valid")]
            summary = [
                (item["tag_id"], round(item["camera_tag_mm"]["z"], 1), round(item["reprojection_error_px"], 2))
                for item in valid_items
            ]
            self.get_logger().info(
                f"PnP valid={valid} reason={reason} tags(id,z_mm,err_px)={summary} "
                f"fps={self.average_fps():.1f} failures={self.pnp_failure_count}"
            )

    def destroy_node(self) -> None:
        try:
            self.camera.stop()
            print("Orbbec color stream stopped.", flush=True)
        except Exception as exc:
            print(f"Failed to stop camera cleanly: {exc}", flush=True)
        try:
            cv2.destroyAllWindows()
        except Exception:
            pass
        super().destroy_node()


def main(args=None) -> None:
    rclpy.init(args=args)
    node: Optional[AprilTagPnpNode] = None
    try:
        node = AprilTagPnpNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            try:
                node.destroy_node()
            except Exception:
                pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
