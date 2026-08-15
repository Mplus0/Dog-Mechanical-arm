#!/usr/bin/env python3
"""Authenticated full-duplex TCP transport for the ROS2 arm protocol."""

from collections import OrderedDict
import hashlib
import hmac
import json
import os
import socket
import threading
import time
import uuid

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String


PROTOCOL_VERSION = 1


class DogArmTcpServerNode(Node):
    def __init__(self):
        super().__init__("dog_arm_tcp_server")

        self.declare_parameter("bind_host", "192.168.31.56")
        self.declare_parameter("allowed_client_ip", "192.168.31.192")
        self.declare_parameter("server_port", 47001)
        self.declare_parameter("shared_secret_file", "~/.ros/dog_arm_shared_secret")
        self.declare_parameter("heartbeat_interval", 1.0)
        self.declare_parameter("heartbeat_timeout", 5.0)
        self.declare_parameter("handshake_timeout", 3.0)
        self.declare_parameter("resend_interval", 0.8)
        self.declare_parameter("outbound_ttl", 300.0)
        self.declare_parameter("max_frame_bytes", 65536)
        self.declare_parameter("dedupe_cache_size", 512)
        self.declare_parameter("task_cmd_topic", "/dog_arm/task_cmd")
        self.declare_parameter("task_result_topic", "/dog_arm/task_result")
        self.declare_parameter("base_adjust_req_topic", "/dog_arm/base_adjust_req")
        self.declare_parameter("connected_topic", "/dog_arm/transport_connected")
        self.declare_parameter("status_topic", "/dog_arm/transport_status")

        self.bind_host = str(self.get_parameter("bind_host").value).strip()
        self.allowed_client_ip = str(self.get_parameter("allowed_client_ip").value).strip()
        self.server_port = int(self.get_parameter("server_port").value)
        self.shared_secret_file = os.path.expanduser(str(self.get_parameter("shared_secret_file").value))
        self._shared_secret = self._load_shared_secret(self.shared_secret_file)
        self.heartbeat_interval = max(0.2, float(self.get_parameter("heartbeat_interval").value))
        self.heartbeat_timeout = max(
            self.heartbeat_interval * 2.0,
            float(self.get_parameter("heartbeat_timeout").value),
        )
        self.handshake_timeout = max(0.5, float(self.get_parameter("handshake_timeout").value))
        self.resend_interval = max(0.1, float(self.get_parameter("resend_interval").value))
        self.outbound_ttl = max(1.0, float(self.get_parameter("outbound_ttl").value))
        self.max_frame_bytes = max(1024, int(self.get_parameter("max_frame_bytes").value))
        self.dedupe_cache_size = max(16, int(self.get_parameter("dedupe_cache_size").value))

        self.task_cmd_topic = str(self.get_parameter("task_cmd_topic").value)
        self.task_result_topic = str(self.get_parameter("task_result_topic").value)
        self.base_adjust_req_topic = str(self.get_parameter("base_adjust_req_topic").value)
        self.connected_topic = str(self.get_parameter("connected_topic").value)
        self.status_topic = str(self.get_parameter("status_topic").value)

        self.session_id = "arm-%s" % uuid.uuid4().hex[:12]
        self._stop = threading.Event()
        self._lock = threading.RLock()
        self._pending = OrderedDict()
        self._seen_tasks = OrderedDict()
        self._connected = False
        self._listen_socket = None
        self._client_socket = None
        self._last_no_task_subscriber_log = 0.0

        self.task_pub = self.create_publisher(String, self.task_cmd_topic, 10)
        self.connected_pub = self.create_publisher(Bool, self.connected_topic, 1)
        self.status_pub = self.create_publisher(String, self.status_topic, 10)
        self.result_sub = self.create_subscription(String, self.task_result_topic, self._on_task_result, 10)
        self.adjust_sub = self.create_subscription(String, self.base_adjust_req_topic, self._on_base_adjust_req, 10)

        self._publish_connected(False)
        self._publish_status("listening", "starting")
        self._worker = threading.Thread(target=self._run, name="dog_arm_tcp_server")
        self._worker.daemon = True
        self._worker.start()
        self.get_logger().info(
            "dog-arm TCP server: bind=%s:%d allowed_client=%s"
            % (self.bind_host, self.server_port, self.allowed_client_ip)
        )

    def _load_shared_secret(self, path):
        try:
            with open(path, "rb") as handle:
                secret = handle.read().strip()
        except OSError as exc:
            raise RuntimeError("cannot read dog-arm shared secret %s: %s" % (path, exc))
        if len(secret) < 16:
            raise RuntimeError("dog-arm shared secret must contain at least 16 bytes: %s" % path)
        return secret

    def _proof(self, role, client_nonce, server_nonce):
        text = "%s|%s|%s|%d" % (role, client_nonce, server_nonce, PROTOCOL_VERSION)
        return hmac.new(self._shared_secret, text.encode("utf-8"), hashlib.sha256).hexdigest()

    def _sign_envelope(self, envelope):
        signed = dict(envelope)
        signed.pop("signature", None)
        canonical = json.dumps(
            signed,
            ensure_ascii=False,
            sort_keys=True,
            separators=(",", ":"),
        ).encode("utf-8")
        signed["signature"] = hmac.new(self._shared_secret, canonical, hashlib.sha256).hexdigest()
        return signed

    def _verify_envelope(self, envelope):
        signature = str(envelope.get("signature", ""))
        if not signature:
            return False
        unsigned = dict(envelope)
        unsigned.pop("signature", None)
        canonical = json.dumps(
            unsigned,
            ensure_ascii=False,
            sort_keys=True,
            separators=(",", ":"),
        ).encode("utf-8")
        expected = hmac.new(self._shared_secret, canonical, hashlib.sha256).hexdigest()
        return hmac.compare_digest(signature, expected)

    def _encode(self, value):
        text = json.dumps(value, ensure_ascii=False, sort_keys=True, separators=(",", ":"))
        return (text + "\n").encode("utf-8")

    def _decode_payload(self, text, label):
        try:
            value = json.loads(text)
        except ValueError as exc:
            self.get_logger().warning("invalid %s JSON: %s; raw=%s" % (label, exc, text))
            return None
        if not isinstance(value, dict):
            self.get_logger().warning("invalid %s: expected JSON object" % label)
            return None
        return value

    def _envelope(self, message_type, payload=None, message_id=None):
        value = {
            "version": PROTOCOL_VERSION,
            "type": message_type,
            "session_id": self.session_id,
            "timestamp": time.time(),
        }
        if message_id:
            value["message_id"] = message_id
        if payload is not None:
            value["payload"] = payload
        return value

    def _on_task_result(self, msg):
        payload = self._decode_payload(msg.data, "task result")
        if payload is None:
            return
        task_id = str(payload.get("task_id", "")).strip()
        result = str(payload.get("result", "")).strip()
        if not task_id or not result:
            self.get_logger().warning("task result missing task_id/result: %s" % msg.data)
            return
        error = str(payload.get("error", ""))
        message_id = "task_result:%s:%s:%s" % (task_id, result, error)
        self._queue_outbound("task_result", payload, message_id)

    def _on_base_adjust_req(self, msg):
        payload = self._decode_payload(msg.data, "base adjust request")
        if payload is None:
            return
        task_id = str(payload.get("task_id", "")).strip()
        direction = str(payload.get("direction", "")).strip().lower()
        if not task_id or direction not in ("left", "right"):
            self.get_logger().warning("invalid base adjust request: %s" % msg.data)
            return
        self._queue_outbound("base_adjust_req", payload, "base_adjust_req:%s" % task_id)

    def _queue_outbound(self, message_type, payload, message_id):
        now = time.monotonic()
        with self._lock:
            if message_id in self._pending:
                return
            self._pending[message_id] = {
                "envelope": self._envelope(message_type, payload, message_id),
                "created": now,
                "last_send": 0.0,
            }
        self.get_logger().info("queued arm -> dog %s: %s" % (message_type, message_id))

    def _publish_connected(self, connected):
        msg = Bool()
        msg.data = bool(connected)
        self.connected_pub.publish(msg)

    def _publish_status(self, state, detail="", peer=""):
        msg = String()
        msg.data = json.dumps(
            {
                "state": state,
                "detail": str(detail),
                "peer": str(peer),
                "session_id": self.session_id,
                "timestamp": time.time(),
            },
            ensure_ascii=False,
            sort_keys=True,
        )
        self.status_pub.publish(msg)

    def _set_connected(self, connected, detail="", peer=""):
        changed = False
        with self._lock:
            connected = bool(connected)
            if self._connected != connected:
                self._connected = connected
                changed = True
        if not changed:
            return
        self._publish_connected(connected)
        self._publish_status("connected" if connected else "disconnected", detail, peer)
        if connected:
            self.get_logger().info("authenticated dog TCP client connected: %s" % peer)
        else:
            self.get_logger().warning("dog-arm TCP disconnected: %s" % detail)

    def _remember_task(self, task_id):
        with self._lock:
            duplicate = task_id in self._seen_tasks
            self._seen_tasks[task_id] = time.monotonic()
            self._seen_tasks.move_to_end(task_id)
            while len(self._seen_tasks) > self.dedupe_cache_size:
                self._seen_tasks.popitem(last=False)
        return duplicate

    def _send(self, sock, envelope):
        sock.sendall(self._encode(self._sign_envelope(envelope)))

    def _ack(self, sock, message_id):
        self._send(sock, self._envelope("ack", {"ack_id": message_id}))

    def _handle_inbound(self, sock, envelope, handshake):
        if not self._verify_envelope(envelope):
            raise RuntimeError("dog TCP frame signature verification failed")
        if int(envelope.get("version", -1)) != PROTOCOL_VERSION:
            raise RuntimeError("unsupported protocol version: %s" % envelope.get("version"))
        message_type = str(envelope.get("type", ""))

        if message_type == "hello":
            if str(envelope.get("role", "")) != "dog":
                raise RuntimeError("unexpected TCP peer role")
            client_nonce = str(envelope.get("client_nonce", ""))
            if not client_nonce:
                raise RuntimeError("missing dog authentication nonce")
            handshake["client_nonce"] = client_nonce
            handshake["server_nonce"] = uuid.uuid4().hex
            challenge = self._envelope("challenge")
            challenge["client_nonce"] = client_nonce
            challenge["server_nonce"] = handshake["server_nonce"]
            self._send(sock, challenge)
            return
        if message_type == "auth":
            client_nonce = str(envelope.get("client_nonce", ""))
            server_nonce = str(envelope.get("server_nonce", ""))
            if client_nonce != handshake["client_nonce"] or server_nonce != handshake["server_nonce"]:
                raise RuntimeError("authentication nonce mismatch")
            expected = self._proof("dog", client_nonce, server_nonce)
            if not hmac.compare_digest(str(envelope.get("proof", "")), expected):
                raise RuntimeError("dog authentication failed")
            response = self._envelope("hello_ack")
            response["role"] = "arm"
            response["proof"] = self._proof("arm", client_nonce, server_nonce)
            self._send(sock, response)
            handshake["complete"] = True
            return
        if not handshake["complete"]:
            raise RuntimeError("message received before authentication")
        if message_type == "heartbeat":
            self._send(sock, self._envelope("heartbeat_ack"))
            return
        if message_type == "heartbeat_ack":
            return
        if message_type == "ack":
            payload = envelope.get("payload", {})
            ack_id = str(payload.get("ack_id", "")) if isinstance(payload, dict) else ""
            if ack_id:
                with self._lock:
                    self._pending.pop(ack_id, None)
            return
        if message_type != "task_cmd":
            self.get_logger().warning("ignored unknown dog TCP message type: %s" % message_type)
            return

        message_id = str(envelope.get("message_id", "")).strip()
        payload = envelope.get("payload")
        if not message_id or not isinstance(payload, dict):
            self.get_logger().warning("ignored invalid task_cmd envelope")
            return
        task_id = str(payload.get("task_id", "")).strip()
        cmd = str(payload.get("cmd", "")).strip()
        if not task_id or cmd not in ("pick", "place_to_zone"):
            self.get_logger().warning("ignored invalid task command payload: %s" % payload)
            return
        if self.task_pub.get_subscription_count() == 0:
            now = time.monotonic()
            if now - self._last_no_task_subscriber_log > 5.0:
                self.get_logger().warning(
                    "task not acknowledged yet: no ROS2 subscriber on %s" % self.task_cmd_topic
                )
                self._last_no_task_subscriber_log = now
            return
        self._ack(sock, message_id)
        if self._remember_task(task_id):
            self.get_logger().info("duplicate task acknowledged without republish: %s" % task_id)
            return
        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=False, sort_keys=True, separators=(",", ":"))
        self.task_pub.publish(msg)
        self.get_logger().info("dog -> arm task_cmd: %s" % msg.data)

    def _send_pending(self, sock, now):
        expired = []
        due = []
        with self._lock:
            for message_id, item in list(self._pending.items()):
                if now - item["created"] > self.outbound_ttl:
                    expired.append(message_id)
                elif now - item["last_send"] >= self.resend_interval:
                    due.append((message_id, item["envelope"]))
            for message_id in expired:
                self._pending.pop(message_id, None)
        for message_id in expired:
            self.get_logger().error("dropping stale outbound message: %s" % message_id)
        for message_id, envelope in due:
            self._send(sock, envelope)
            with self._lock:
                if message_id in self._pending:
                    self._pending[message_id]["last_send"] = now

    def _serve_client(self, sock, peer):
        sock.settimeout(0.15)
        buffer = b""
        handshake = {"complete": False, "client_nonce": "", "server_nonce": ""}
        connected_at = time.monotonic()
        last_rx = connected_at
        last_heartbeat = 0.0
        peer_text = "%s:%s" % peer[:2]
        while not self._stop.is_set() and rclpy.ok():
            now = time.monotonic()
            try:
                chunk = sock.recv(4096)
                if not chunk:
                    raise ConnectionError("peer closed connection")
                buffer += chunk
                last_rx = now
                if len(buffer) > self.max_frame_bytes and b"\n" not in buffer:
                    raise RuntimeError("incoming frame exceeds max_frame_bytes")
                while b"\n" in buffer:
                    raw, buffer = buffer.split(b"\n", 1)
                    if not raw:
                        continue
                    if len(raw) > self.max_frame_bytes:
                        raise RuntimeError("incoming frame exceeds max_frame_bytes")
                    envelope = json.loads(raw.decode("utf-8"))
                    if not isinstance(envelope, dict):
                        raise RuntimeError("incoming frame is not a JSON object")
                    self._handle_inbound(sock, envelope, handshake)
            except socket.timeout:
                pass
            now = time.monotonic()
            if not handshake["complete"]:
                if now - connected_at > self.handshake_timeout:
                    raise RuntimeError("dog authentication timeout")
                continue
            self._set_connected(True, "authentication complete", peer_text)
            self._send_pending(sock, now)
            if now - last_heartbeat >= self.heartbeat_interval:
                self._send(sock, self._envelope("heartbeat"))
                last_heartbeat = now
            if now - last_rx > self.heartbeat_timeout:
                raise RuntimeError("dog heartbeat timeout")

    def _run(self):
        listen_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        listen_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        listen_socket.settimeout(0.5)
        try:
            listen_socket.bind((self.bind_host, self.server_port))
            listen_socket.listen(1)
            with self._lock:
                self._listen_socket = listen_socket
            self._publish_status("listening", "%s:%d" % (self.bind_host, self.server_port))
            while not self._stop.is_set() and rclpy.ok():
                try:
                    client_socket, peer = listen_socket.accept()
                except socket.timeout:
                    continue
                if peer[0] != self.allowed_client_ip:
                    self.get_logger().warning("rejected TCP client from unauthorized IP: %s" % (peer[0],))
                    client_socket.close()
                    continue
                with self._lock:
                    self._client_socket = client_socket
                try:
                    self._serve_client(client_socket, peer)
                except Exception as exc:
                    if not self._stop.is_set():
                        self.get_logger().warning("dog-arm TCP client error: %s" % exc)
                finally:
                    self._set_connected(False, "connection closed")
                    with self._lock:
                        self._client_socket = None
                    try:
                        client_socket.shutdown(socket.SHUT_RDWR)
                    except Exception:
                        pass
                    client_socket.close()
        except Exception as exc:
            self.get_logger().error("dog-arm TCP server stopped: %s" % exc)
            self._publish_status("error", str(exc))
        finally:
            with self._lock:
                self._listen_socket = None
            listen_socket.close()

    def stop_server(self):
        self._stop.set()
        with self._lock:
            sockets = (self._client_socket, self._listen_socket)
        for sock in sockets:
            if sock is None:
                continue
            try:
                sock.shutdown(socket.SHUT_RDWR)
            except Exception:
                pass
            try:
                sock.close()
            except Exception:
                pass
        if self._worker.is_alive():
            self._worker.join(timeout=2.0)
        self._set_connected(False, "shutdown")


def main(args=None):
    rclpy.init(args=args)
    node = DogArmTcpServerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.stop_server()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
