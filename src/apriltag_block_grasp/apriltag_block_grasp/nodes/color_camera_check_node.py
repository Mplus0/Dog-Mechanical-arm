#!/usr/bin/env python3
"""Stage-1A Orbbec color-stream check with no robot-arm access."""

import json
import time
from typing import Optional

import cv2
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from apriltag_block_grasp.core.camera_color_orbbec import OrbbecColorCamera


class ColorCameraCheckNode(Node):
    """Open only the color stream and report decoded-frame health."""

    def __init__(self) -> None:
        super().__init__("apriltag_color_camera_check_node")

        self.declare_parameter("timer_period_s", 0.10)
        self.declare_parameter("frame_timeout_ms", 300)
        self.declare_parameter("status_period_s", 2.0)
        self.declare_parameter("show_window", False)

        self.timer_period_s = max(
            0.01, float(self.get_parameter("timer_period_s").value)
        )
        self.frame_timeout_ms = max(
            1, int(self.get_parameter("frame_timeout_ms").value)
        )
        self.status_period_s = max(
            0.1, float(self.get_parameter("status_period_s").value)
        )
        self.show_window = self.parse_bool(self.get_parameter("show_window").value)

        self.publisher = self.create_publisher(
            String, "/apriltag_grasp/camera_status", 10
        )
        self.camera = OrbbecColorCamera()
        self.started_at = time.monotonic()
        self.last_status_time = 0.0
        self.frame_count = 0
        self.empty_frame_count = 0
        self.decode_failure_count = 0
        self.latest_width: Optional[int] = None
        self.latest_height: Optional[int] = None
        self.latest_format: Optional[str] = None
        self.latest_error: Optional[str] = None
        self.window_name = "AprilTag stage 1A - Orbbec color check"

        self.get_logger().info(
            "Opening Orbbec color stream only. No arm connection or motion command is used."
        )
        try:
            self.camera.start()
        except Exception as exc:
            self.latest_error = f"camera_start_failed: {type(exc).__name__}: {exc}"
            self.publish_status(valid=False, reason="camera_start_failed", force_log=True)
            raise

        self.get_logger().info("Orbbec color stream started.")
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
            self.latest_error = f"camera_read_failed: {type(exc).__name__}: {exc}"
            self.publish_status(valid=False, reason="camera_read_failed")
            return

        if frame is None:
            self.empty_frame_count += 1
            self.publish_status(valid=False, reason="empty_or_unsupported_frame")
            return

        image = frame.bgr
        if image.ndim != 3 or image.shape[2] != 3:
            self.decode_failure_count += 1
            self.latest_error = f"unexpected_bgr_shape: {tuple(image.shape)}"
            self.publish_status(valid=False, reason="unexpected_bgr_shape")
            return

        self.frame_count += 1
        self.latest_height, self.latest_width = image.shape[:2]
        self.latest_format = frame.format_name
        self.latest_error = None
        self.publish_status(valid=True, reason="ok")

        if self.show_window:
            display = image.copy()
            cv2.putText(
                display,
                f"{self.latest_width}x{self.latest_height} {self.latest_format} "
                f"fps={self.average_fps():.1f}",
                (20, 35),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.7,
                (0, 255, 0),
                2,
            )
            cv2.imshow(self.window_name, display)
            key = cv2.waitKey(1) & 0xFF
            if key in (ord("q"), 27):
                self.get_logger().info("Exit requested from the debug window.")
                rclpy.shutdown()

    def average_fps(self) -> float:
        elapsed = max(time.monotonic() - self.started_at, 1e-6)
        return self.frame_count / elapsed

    def publish_status(self, *, valid: bool, reason: str, force_log: bool = False) -> None:
        now = time.monotonic()
        if not force_log and now - self.last_status_time < self.status_period_s:
            return
        self.last_status_time = now

        payload = {
            "stamp": time.time(),
            "valid": bool(valid),
            "reason": reason,
            "camera_started": self.camera.started,
            "width": self.latest_width,
            "height": self.latest_height,
            "format": self.latest_format,
            "average_fps": self.average_fps(),
            "frame_count": self.frame_count,
            "empty_frame_count": self.empty_frame_count,
            "decode_failure_count": self.decode_failure_count,
            "show_window": self.show_window,
            "arm_connected": False,
            "motion_commands_enabled": False,
            "error": self.latest_error,
        }
        message = String()
        message.data = json.dumps(payload, ensure_ascii=False)
        self.publisher.publish(message)

        summary = (
            f"camera valid={valid} reason={reason} "
            f"size={self.latest_width}x{self.latest_height} "
            f"format={self.latest_format} fps={self.average_fps():.1f} "
            f"empty={self.empty_frame_count} decode_fail={self.decode_failure_count}"
        )
        if valid:
            self.get_logger().info(summary)
        else:
            self.get_logger().warn(summary)

    def destroy_node(self) -> None:
        try:
            self.camera.stop()
            # SIGINT may already have invalidated the ROS context, so cleanup
            # messages must not use rosout here.
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
    node: Optional[ColorCameraCheckNode] = None
    try:
        node = ColorCameraCheckNode()
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
