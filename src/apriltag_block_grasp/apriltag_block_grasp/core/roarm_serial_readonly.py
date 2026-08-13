"""Read-only serial access for RoArm-M3 state frames."""

import json
import time
from typing import Any, Dict, Optional


class RoArmSerialStateReader:
    """Open one serial port and accept only unsolicited T=1051 state frames.

    This class intentionally has no write or command method and transmits no
    bytes. On RoArm-M3 hardware, opening its ESP32 USB serial device may still
    pulse DTR/RTS in the USB/TTY stack and reset the controller. Callers must
    therefore treat connect() as hardware-affecting even though it is read-only.
    """

    def __init__(
        self,
        port: str = "/dev/ttyUSB0",
        baudrate: int = 115200,
        timeout_s: float = 0.2,
    ) -> None:
        self.port = str(port)
        self.baudrate = int(baudrate)
        self.timeout_s = float(timeout_s)
        self.serial_port = None

    @property
    def connected(self) -> bool:
        return self.serial_port is not None

    def connect(self, settle_time_s: float = 1.5) -> None:
        if self.connected:
            raise RuntimeError("RoArm serial reader is already connected")
        try:
            import serial
        except ImportError as exc:
            raise RuntimeError(
                "pyserial is unavailable; install the ROS dependency python3-serial"
            ) from exc
        # Configure the requested steady line state before opening. Some USB
        # serial drivers can still produce a short DTR/RTS transition in open(),
        # so this cannot guarantee that an ESP32 auto-reset circuit will not fire.
        serial_port = serial.Serial(
            port=None,
            baudrate=self.baudrate,
            timeout=self.timeout_s,
            write_timeout=1.0,
            xonxoff=False,
            rtscts=False,
            dsrdtr=False,
        )
        serial_port.port = self.port
        serial_port.rts = False
        serial_port.dtr = False
        serial_port.open()
        if settle_time_s > 0.0:
            time.sleep(float(settle_time_s))
        serial_port.reset_input_buffer()
        self.serial_port = serial_port

    def close(self) -> None:
        serial_port = self.serial_port
        self.serial_port = None
        if serial_port is not None:
            serial_port.close()

    def reset_input_buffer(self) -> None:
        """Discard queued state frames so the next read reflects current hardware."""

        if self.serial_port is None:
            raise RuntimeError("RoArm serial reader is not connected")
        self.serial_port.reset_input_buffer()

    def read_state(self, timeout_s: float = 1.0) -> Optional[Dict[str, Any]]:
        if self.serial_port is None:
            raise RuntimeError("RoArm serial reader is not connected")
        deadline = time.monotonic() + max(0.0, float(timeout_s))
        while time.monotonic() < deadline:
            raw = self.serial_port.readline()
            if not raw:
                continue
            line = raw.decode("utf-8", errors="replace").strip()
            if not line:
                continue
            try:
                message = json.loads(line)
            except json.JSONDecodeError:
                continue
            if isinstance(message, dict) and message.get("T") == 1051:
                return message
        return None
