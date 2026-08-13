#!/usr/bin/env python3
"""Stage-1B tag25h9 2D detection node with no depth or arm access."""

import json
import os
import time
from pathlib import Path
from typing import List, Optional

import cv2
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from apriltag_block_grasp.core.apriltag_detector import (
    AprilTagDetection2D,
    OpenCvAprilTag25h9Detector,
    draw_detections,
)
from apriltag_block_grasp.core.camera_color_orbbec import OrbbecColorCamera


class AprilTagDetection2DNode(Node):
    """Publish ID, center and corners for allowed tag25h9 markers."""

    def __init__(self) -> None:
        super().__init__("apriltag_detection_2d_node")

        self.declare_parameter("allowed_ids", [0, 1])
        self.declare_parameter("timer_period_s", 0.10)
        self.declare_parameter("frame_timeout_ms", 300)
        self.declare_parameter("status_period_s", 2.0)
        self.declare_parameter("show_window", False)
        self.declare_parameter("save_images", False)
        self.declare_parameter("save_interval_s", 2.0)
        self.declare_parameter(
            "save_dir",
            os.path.expanduser("~/.ros/apriltag_block_grasp/detection_2d"),
        )

        allowed_ids = [int(value) for value in self.get_parameter("allowed_ids").value]
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
        self.save_images = self.parse_bool(self.get_parameter("save_images").value)
        self.save_interval_s = max(
            0.1, float(self.get_parameter("save_interval_s").value)
        )
        self.save_dir = Path(
            os.path.expanduser(str(self.get_parameter("save_dir").value))
        )

        self.publisher = self.create_publisher(
            String, "/apriltag_grasp/detections_2d", 10
        )
        self.camera = OrbbecColorCamera()
        self.detector = OpenCvAprilTag25h9Detector(allowed_ids=allowed_ids)
        self.started_at = time.monotonic()
        self.last_status_time = 0.0
        self.last_save_time = 0.0
        self.frame_count = 0
        self.detection_frame_count = 0
        self.empty_frame_count = 0
        self.detect_error_count = 0
        self.save_count = 0
        self.window_name = "AprilTag stage 1B - tag25h9 2D detection"

        if self.save_images:
            self.save_dir.mkdir(parents=True, exist_ok=True)

        self.get_logger().info(
            "Opening Orbbec color stream for 2D tag25h9 detection only. "
            "Depth, PnP and arm access are disabled."
        )
        self.get_logger().info(f"Allowed IDs: {allowed_ids}")
        self.camera.start()
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
            self.detect_error_count += 1
            self.publish_result(
                valid=False,
                reason=f"camera_read_failed: {type(exc).__name__}: {exc}",
                width=None,
                height=None,
                frame_format=None,
                detections=[],
                ignored_ids=[],
                rejected_candidate_count=0,
            )
            return

        if frame is None:
            self.empty_frame_count += 1
            self.publish_result(
                valid=False,
                reason="empty_or_unsupported_frame",
                width=None,
                height=None,
                frame_format=None,
                detections=[],
                ignored_ids=[],
                rejected_candidate_count=0,
            )
            return

        image = frame.bgr
        height, width = image.shape[:2]
        self.frame_count += 1
        try:
            batch = self.detector.detect(image)
        except Exception as exc:
            self.detect_error_count += 1
            self.publish_result(
                valid=False,
                reason=f"detection_failed: {type(exc).__name__}: {exc}",
                width=width,
                height=height,
                frame_format=frame.format_name,
                detections=[],
                ignored_ids=[],
                rejected_candidate_count=0,
            )
            return

        detections = list(batch.detections)
        if detections:
            self.detection_frame_count += 1
        self.publish_result(
            valid=True,
            reason="ok" if detections else "no_allowed_tag",
            width=width,
            height=height,
            frame_format=frame.format_name,
            detections=detections,
            ignored_ids=list(batch.ignored_ids),
            rejected_candidate_count=batch.rejected_candidate_count,
        )

        needs_display = self.show_window or self.save_images
        if needs_display:
            display = draw_detections(image, detections)
            cv2.putText(
                display,
                f"tag25h9 count={len(detections)} fps={self.average_fps():.1f}",
                (20, 32),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.70,
                (0, 255, 255),
                2,
                cv2.LINE_AA,
            )
            self.maybe_save(display, detections)
            if self.show_window:
                cv2.imshow(self.window_name, display)
                key = cv2.waitKey(1) & 0xFF
                if key in (ord("q"), 27):
                    self.get_logger().info("Exit requested from the debug window.")
                    rclpy.shutdown()

    def average_fps(self) -> float:
        elapsed = max(time.monotonic() - self.started_at, 1e-6)
        return self.frame_count / elapsed

    @staticmethod
    def detection_to_dict(detection: AprilTagDetection2D) -> dict:
        return {
            "tag_id": detection.tag_id,
            "center": {"u": detection.center[0], "v": detection.center[1]},
            "corners": [
                {"index": index, "u": point[0], "v": point[1]}
                for index, point in enumerate(detection.corners)
            ],
            "corner_order": "opencv_detector_order_0_1_2_3",
            "area_px2": detection.area_px2,
            "perimeter_px": detection.perimeter_px,
        }

    def publish_result(
        self,
        *,
        valid: bool,
        reason: str,
        width: Optional[int],
        height: Optional[int],
        frame_format: Optional[str],
        detections: List[AprilTagDetection2D],
        ignored_ids: List[int],
        rejected_candidate_count: int,
    ) -> None:
        payload = {
            "stamp": time.time(),
            "frame_id": "camera_color",
            "valid": bool(valid),
            "reason": reason,
            "family": "tag25h9",
            "allowed_ids": list(self.detector.allowed_ids),
            "width": width,
            "height": height,
            "format": frame_format,
            "count": len(detections),
            "detections": [self.detection_to_dict(item) for item in detections],
            "ignored_ids": ignored_ids,
            "rejected_candidate_count": int(rejected_candidate_count),
            "average_fps": self.average_fps(),
            "frame_count": self.frame_count,
            "detection_frame_count": self.detection_frame_count,
            "empty_frame_count": self.empty_frame_count,
            "detect_error_count": self.detect_error_count,
            "saved_image_count": self.save_count,
            "depth_enabled": False,
            "pnp_enabled": False,
            "arm_connected": False,
            "motion_commands_enabled": False,
        }
        message = String()
        message.data = json.dumps(payload, ensure_ascii=False)
        self.publisher.publish(message)

        now = time.monotonic()
        if now - self.last_status_time >= self.status_period_s:
            self.last_status_time = now
            ids = [item.tag_id for item in detections]
            summary = (
                f"2D detection valid={valid} reason={reason} ids={ids} "
                f"size={width}x{height} format={frame_format} "
                f"fps={self.average_fps():.1f} empty={self.empty_frame_count} "
                f"errors={self.detect_error_count}"
            )
            if valid:
                self.get_logger().info(summary)
            else:
                self.get_logger().warn(summary)

    def maybe_save(
        self,
        display,
        detections: List[AprilTagDetection2D],
    ) -> None:
        if not self.save_images:
            return
        now = time.monotonic()
        if now - self.last_save_time < self.save_interval_s:
            return
        self.last_save_time = now
        stamp_ms = int(time.time() * 1000)
        ids = "none" if not detections else "-".join(str(item.tag_id) for item in detections)
        path = self.save_dir / f"frame_{stamp_ms}_ids_{ids}.jpg"
        if cv2.imwrite(str(path), display):
            self.save_count += 1
        else:
            self.get_logger().warn(f"Failed to save image: {path}")

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
    node: Optional[AprilTagDetection2DNode] = None
    try:
        node = AprilTagDetection2DNode()
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
