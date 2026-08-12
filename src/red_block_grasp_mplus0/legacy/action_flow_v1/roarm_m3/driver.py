#!/usr/bin/env python3
# 中文说明：
# 这是 RoArm-M3 的串口底层驱动封装。
# 主要负责：
# 1. 打开 /dev/ttyUSB0 串口
# 2. 发送 JSON 指令到机械臂
# 3. 读取 T=1051 状态反馈
# 4. 把关节弧度转换成角度
# 调试时一般不需要改这个文件，优先改 scripts/ 里的上层逻辑。

import json
import math
import time
import serial


class RoArmM3:
    JOINT_ID = {
        "b": 1,
        "s": 2,
        "e": 3,
        "t": 4,
        "r": 5,
        "g": 6,
    }

    JOINT_NAMES = ["b", "s", "e", "t", "r", "g"]

    def __init__(self, port="/dev/ttyUSB0", baudrate=115200, timeout=0.2):
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.ser = None

    def connect(self):
        self.ser = serial.Serial(
            port=self.port,
            baudrate=self.baudrate,
            timeout=self.timeout,
            write_timeout=1.0,
            xonxoff=False,
            rtscts=False,
            dsrdtr=False,
        )
        self.ser.setRTS(False)
        self.ser.setDTR(False)
        time.sleep(1.5)
        self.ser.reset_input_buffer()
        self.ser.reset_output_buffer()

    def close(self):
        if self.ser is not None:
            self.ser.close()
            self.ser = None

    def send_cmd(self, cmd):
        if self.ser is None:
            raise RuntimeError("Serial port is not connected.")

        text = json.dumps(cmd, separators=(",", ":"))
        print("SEND:", text)
        self.ser.write((text + "\n").encode("utf-8"))
        self.ser.flush()

    def read_latest_state(self, duration_s=2.0):
        if self.ser is None:
            raise RuntimeError("Serial port is not connected.")

        end_time = time.time() + duration_s
        latest = None

        while time.time() < end_time:
            raw = self.ser.readline()
            if not raw:
                continue

            line = raw.decode("utf-8", errors="replace").strip()
            if not line:
                continue

            try:
                obj = json.loads(line)
            except json.JSONDecodeError:
                continue

            if not isinstance(obj, dict):
                continue

            if obj.get("T") == 1051:
                latest = obj

        return latest

    @staticmethod
    def rad_to_deg(value):
        return value * 180.0 / math.pi

    @classmethod
    def state_to_deg(cls, state):
        result = {}
        for name in cls.JOINT_NAMES:
            result[name] = cls.rad_to_deg(float(state.get(name, 0.0)))
        return result

    @staticmethod
    def voltage_from_state(state):
        return float(state.get("v", 0)) / 100.0

    def move_joint_deg(self, joint_name, angle_deg, speed_deg_s=10, acc=10):
        if joint_name not in self.JOINT_ID:
            raise ValueError(f"Unknown joint name: {joint_name}")

        cmd = {
            "T": 121,
            "joint": self.JOINT_ID[joint_name],
            "angle": float(angle_deg),
            "spd": float(speed_deg_s),
            "acc": float(acc),
        }
        self.send_cmd(cmd)

    @classmethod
    def print_state(cls, title, state):
        if state is None:
            print(title)
            print("  No state received.")
            return

        deg = cls.state_to_deg(state)
        voltage = cls.voltage_from_state(state)

        print(title)
        print(f"  xyz(mm): x={state.get('x', 0):.2f}, y={state.get('y', 0):.2f}, z={state.get('z', 0):.2f}")
        print(
            "  joint(deg): "
            f"b={deg['b']:.2f}, "
            f"s={deg['s']:.2f}, "
            f"e={deg['e']:.2f}, "
            f"t={deg['t']:.2f}, "
            f"r={deg['r']:.2f}, "
            f"g={deg['g']:.2f}"
        )
        print(f"  voltage: {voltage:.2f} V")
