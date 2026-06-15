#!/usr/bin/env python3
import json
import time

import cv2
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from red_block_grasp_ros2.core.camera_orbbec import OrbbecColorCamera
from red_block_grasp_ros2.core.yolo_detector import YoloRedBlockDetector


class YoloCameraNode(Node):
    def __init__(self):
        super().__init__("yolo_camera_node")

        self.declare_parameter(
            "model_path",
            "/home/sunrise/dog/ros2_red_block_ws/src/red_block_grasp_ros2/models/red_block_yolo11n.pt"
        )
        self.declare_parameter("conf_thres", 0.35)
        self.declare_parameter("max_targets", 4)
        self.declare_parameter("show_window", True)
        self.declare_parameter("timer_period", 0.2)

        self.model_path = self.get_parameter("model_path").value
        self.conf_thres = float(self.get_parameter("conf_thres").value)
        self.max_targets = int(self.get_parameter("max_targets").value)
        self.show_window = bool(self.get_parameter("show_window").value)
        self.timer_period = float(self.get_parameter("timer_period").value)

        self.publisher = self.create_publisher(String, "/red_block/detections", 10)

        self.camera = OrbbecColorCamera()
        self.detector = YoloRedBlockDetector(
            model_path=self.model_path,
            conf_thres=self.conf_thres,
            max_targets=self.max_targets,
        )

        self.window_name = "ROS2 YOLO Red Block"
        self.last_time = time.time()
        self.fps = 0.0

        self.get_logger().info("Starting Orbbec camera...")
        self.camera.start()

        self.get_logger().info("Loading YOLO model...")
        self.detector.load()

        if self.show_window:
            cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
            cv2.resizeWindow(self.window_name, 1100, 700)

        self.timer = self.create_timer(self.timer_period, self.on_timer)
        self.get_logger().info("YOLO camera node started.")

    def on_timer(self):
        bgr = self.camera.read_bgr(timeout_ms=100)
        if bgr is None:
            return

        now = time.time()
        dt = now - self.last_time
        self.last_time = now
        if dt > 1e-6:
            self.fps = 0.9 * self.fps + 0.1 * (1.0 / dt)

        detections = self.detector.detect(bgr)

        msg_dict = {
            "stamp": now,
            "frame_id": "camera_color",
            "count": len(detections),
            "detections": [det.to_dict(index=i) for i, det in enumerate(detections)]
        }

        msg = String()
        msg.data = json.dumps(msg_dict, ensure_ascii=False)
        self.publisher.publish(msg)

        if self.show_window:
            display = self.draw_detections(bgr, detections)
            cv2.imshow(self.window_name, display)
            key = cv2.waitKey(1) & 0xFF
            if key in (ord("q"), 27):
                self.get_logger().info("Quit key received.")
                rclpy.shutdown()

    def draw_detections(self, image, detections):
        display = image.copy()

        cv2.putText(
            display,
            f"ROS2 YOLO red block | FPS={self.fps:.1f}",
            (20, 35),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.8,
            (0, 255, 255),
            2,
        )

        if not detections:
            cv2.putText(display, "NO RED BLOCK", (20, 75), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
            return display

        for idx, det in enumerate(detections):
            color = (0, 255, 255) if idx == 0 else (0, 0, 255)
            thickness = 3 if idx == 0 else 2

            cv2.rectangle(display, (det.x1, det.y1), (det.x2, det.y2), color, thickness)
            cx, cy = det.center
            cv2.circle(display, (cx, cy), 5, color, -1)

            label = f"red#{idx} conf={det.conf:.2f} center=({cx},{cy})"
            cv2.putText(display, label, (det.x1, max(25, det.y1 - 8)), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)

        return display

    def destroy_node(self):
        try:
            self.camera.stop()
        except Exception:
            pass
        try:
            cv2.destroyAllWindows()
        except Exception:
            pass
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = YoloCameraNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
