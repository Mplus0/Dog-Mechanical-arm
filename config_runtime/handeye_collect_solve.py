import os
import sys
import json
import time
import math
from pathlib import Path

import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from roarm_msgs.srv import GetPoseCmd

# 允许直接从源码导入 OrbbecRgbdCamera
WS = Path("/home/sunrise/dog/ros2_red_block_ws")
SRC_PKG = WS / "src/red_block_grasp_ros2"
if str(SRC_PKG) not in sys.path:
    sys.path.insert(0, str(SRC_PKG))

from red_block_grasp_ros2.core.camera_rgbd_orbbec import OrbbecRgbdCamera


# 你的棋盘格参数：9x6 内角点，小格 2.3cm
PATTERN_SIZE = (9, 6)
SQUARE_SIZE_M = 0.023

OUT_DIR = WS / "config_runtime/handeye_calib" / f"session_{time.strftime('%Y%m%d_%H%M%S')}"
OUT_DIR.mkdir(parents=True, exist_ok=True)


def rpy_to_R(roll, pitch, yaw):
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)

    Rx = np.array([[1, 0, 0],
                   [0, cr, -sr],
                   [0, sr, cr]], dtype=np.float64)
    Ry = np.array([[cp, 0, sp],
                   [0, 1, 0],
                   [-sp, 0, cp]], dtype=np.float64)
    Rz = np.array([[cy, -sy, 0],
                   [sy, cy, 0],
                   [0, 0, 1]], dtype=np.float64)
    return Rz @ Ry @ Rx


def T_from_R_t(R, t):
    T = np.eye(4, dtype=np.float64)
    T[:3, :3] = R
    T[:3, 3] = np.asarray(t, dtype=np.float64).reshape(3)
    return T


class PoseClient(Node):
    def __init__(self):
        super().__init__("handeye_pose_client")
        self.cli = self.create_client(GetPoseCmd, "/get_pose_cmd")
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info("等待 /get_pose_cmd ...")

    def get_pose(self):
        req = GetPoseCmd.Request()
        future = self.cli.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        if future.result() is None:
            raise RuntimeError("读取 /get_pose_cmd 失败")
        res = future.result()
        return {
            "x": float(res.x),
            "y": float(res.y),
            "z": float(res.z),
            "roll": float(res.roll),
            "pitch": float(res.pitch),
            "yaw": float(res.yaw),
        }


def make_object_points():
    objp = np.zeros((PATTERN_SIZE[0] * PATTERN_SIZE[1], 3), np.float32)
    grid = np.mgrid[0:PATTERN_SIZE[0], 0:PATTERN_SIZE[1]].T.reshape(-1, 2)
    objp[:, :2] = grid * SQUARE_SIZE_M
    return objp


def detect_chessboard(bgr):
    gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)

    ok, corners = cv2.findChessboardCorners(
        gray,
        PATTERN_SIZE,
        cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE
    )

    if ok:
        criteria = (
            cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER,
            30,
            0.001
        )
        corners = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)

    return ok, corners


def solve_one_sample(bgr, camera_matrix):
    ok, corners = detect_chessboard(bgr)
    if not ok:
        return None

    objp = make_object_points()
    dist = np.zeros((5, 1), dtype=np.float64)

    ok_pnp, rvec, tvec = cv2.solvePnP(
        objp,
        corners,
        camera_matrix.astype(np.float64),
        dist,
        flags=cv2.SOLVEPNP_ITERATIVE
    )
    if not ok_pnp:
        return None

    R_board_to_cam, _ = cv2.Rodrigues(rvec)

    return {
        "corners": corners,
        "rvec": rvec.reshape(3).tolist(),
        "tvec": tvec.reshape(3).tolist(),
        "R_target2cam": R_board_to_cam.tolist(),
        "t_target2cam": tvec.reshape(3).tolist(),
    }


def load_samples(session_dir):
    samples = []
    for p in sorted(session_dir.glob("sample_*.json")):
        with open(p, "r", encoding="utf-8") as f:
            samples.append(json.load(f))
    return samples


def solve_handeye(session_dir):
    samples = load_samples(session_dir)
    if len(samples) < 8:
        print(f"样本太少：{len(samples)}，建议至少 15 组，最低 8 组。")
        return

    R_gripper2base = []
    t_gripper2base = []
    R_target2cam = []
    t_target2cam = []

    for s in samples:
        pose = s["base_T_gripper_pose"]
        R_bg = rpy_to_R(pose["roll"], pose["pitch"], pose["yaw"])
        t_bg = np.array([pose["x"], pose["y"], pose["z"]], dtype=np.float64)

        R_tc = np.array(s["R_target2cam"], dtype=np.float64)
        t_tc = np.array(s["t_target2cam"], dtype=np.float64).reshape(3)

        R_gripper2base.append(R_bg)
        t_gripper2base.append(t_bg)
        R_target2cam.append(R_tc)
        t_target2cam.append(t_tc)

    methods = {
        "TSAI": cv2.CALIB_HAND_EYE_TSAI,
        "PARK": cv2.CALIB_HAND_EYE_PARK,
        "HORAUD": cv2.CALIB_HAND_EYE_HORAUD,
    }

    results = {}

    for name, method in methods.items():
        R_cam2gripper, t_cam2gripper = cv2.calibrateHandEye(
            R_gripper2base,
            t_gripper2base,
            R_target2cam,
            t_target2cam,
            method=method
        )

        T = T_from_R_t(R_cam2gripper, t_cam2gripper)
        results[name] = {
            "R_cam2gripper": R_cam2gripper.tolist(),
            "t_cam2gripper_m": t_cam2gripper.reshape(3).tolist(),
            "T_cam2gripper": T.tolist(),
            "note": "This is gripper_T_camera / camera-to-gripper transform."
        }

    out = {
        "pattern_size": PATTERN_SIZE,
        "square_size_m": SQUARE_SIZE_M,
        "sample_count": len(samples),
        "results": results,
    }

    out_path = session_dir / "handeye_result.json"
    with open(out_path, "w", encoding="utf-8") as f:
        json.dump(out, f, ensure_ascii=False, indent=2)

    print("\n========== 手眼标定结果 ==========")
    print("样本数:", len(samples))
    print("结果文件:", out_path)

    main = results["TSAI"]
    t = np.array(main["t_cam2gripper_m"]) * 1000.0
    print("\n推荐先看 TSAI：")
    print("t_cam2gripper / gripper_T_camera translation mm =", t)
    print("T_cam2gripper / gripper_T_camera =")
    print(np.array(main["T_cam2gripper"]))
    print("=================================\n")


def main():
    rclpy.init()
    pose_client = PoseClient()

    print("输出目录:", OUT_DIR)
    print("棋盘格: 9x6 内角点, square_size=0.023m")
    print("按键说明：")
    print("  c：采集当前帧")
    print("  s：求解手眼标定")
    print("  q：退出")
    print("注意：采集时棋盘格固定不动，只移动机械臂相机。")

    camera = OrbbecRgbdCamera()
    camera.start()
    time.sleep(1.0)

    sample_id = 0

    try:
        while True:
            bgr, depth_mm, camera_matrix = camera.read(timeout_ms=100)
            if bgr is None or camera_matrix is None:
                print("未读到图像或 camera_matrix")
                continue

            result = solve_one_sample(bgr, camera_matrix)
            vis = bgr.copy()

            if result is not None:
                corners = result["corners"]
                cv2.drawChessboardCorners(vis, PATTERN_SIZE, corners, True)
                status = "FOUND - press c to capture"
                color = (0, 255, 0)
            else:
                status = "NOT FOUND"
                color = (0, 0, 255)

            cv2.putText(vis, status, (20, 40), cv2.FONT_HERSHEY_SIMPLEX, 1.0, color, 2)
            cv2.putText(vis, f"samples: {sample_id}", (20, 80), cv2.FONT_HERSHEY_SIMPLEX, 1.0, color, 2)
            cv2.imshow("handeye chessboard capture", vis)

            key = cv2.waitKey(1) & 0xFF

            if key == ord("q"):
                break

            if key == ord("s"):
                solve_handeye(OUT_DIR)
                continue

            if key == ord("c"):
                if result is None:
                    print("当前没有识别到完整棋盘格，不能采集。")
                    continue

                pose = pose_client.get_pose()
                sample_id += 1

                img_path = OUT_DIR / f"sample_{sample_id:02d}.png"
                json_path = OUT_DIR / f"sample_{sample_id:02d}.json"

                cv2.imwrite(str(img_path), vis)

                sample = {
                    "id": sample_id,
                    "time": time.time(),
                    "image": str(img_path),
                    "camera_matrix": camera_matrix.tolist(),
                    "base_T_gripper_pose": pose,
                    "rvec_target2cam": result["rvec"],
                    "tvec_target2cam": result["tvec"],
                    "R_target2cam": result["R_target2cam"],
                    "t_target2cam": result["t_target2cam"],
                    "pattern_size": PATTERN_SIZE,
                    "square_size_m": SQUARE_SIZE_M,
                }

                with open(json_path, "w", encoding="utf-8") as f:
                    json.dump(sample, f, ensure_ascii=False, indent=2)

                print(f"已采集 sample {sample_id:02d}: {json_path}")
                print("arm pose:", pose)

    finally:
        try:
            camera.stop()
        except Exception:
            pass
        cv2.destroyAllWindows()
        pose_client.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
