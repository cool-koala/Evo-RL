from __future__ import annotations

import glob
import os
import time
from dataclasses import dataclass

import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image


@dataclass
class CameraSpec:
    name: str
    topic: str
    device: str
    serial: str
    publisher: object | None = None
    capture: cv2.VideoCapture | None = None
    failed_reads: int = 0
    last_reopen_s: float = 0.0


def _env(name: str, default: str = "") -> str:
    return os.environ.get(name, default)


def _resolve_device(device: str, serial: str) -> str | int:
    if device:
        return int(device) if device.isdigit() else device
    if serial:
        for base in ("/dev/v4l/by-id", "/dev/v4l/by-path"):
            matches = sorted(glob.glob(f"{base}/*{serial}*"))
            if matches:
                return matches[0]
    return int(device) if device.isdigit() else device


class CobotMagicCameraNode(Node):
    def __init__(self) -> None:
        super().__init__("cobot_magic_rgb_cameras")
        self.declare_parameter("width", int(_env("CAMERA_WIDTH", "640")))
        self.declare_parameter("height", int(_env("CAMERA_HEIGHT", "480")))
        self.declare_parameter("fps", int(_env("CAMERA_FPS", "30")))
        self.declare_parameter("fourcc", _env("CAMERA_FOURCC", "MJPG"))
        self.declare_parameter("camera_f_device", _env("CAMERA_F_DEVICE"))
        self.declare_parameter("camera_l_device", _env("CAMERA_L_DEVICE"))
        self.declare_parameter("camera_r_device", _env("CAMERA_R_DEVICE"))
        self.declare_parameter("camera_f_serial", _env("CAMERA_F_SERIAL", "AU1SB3300XB"))
        self.declare_parameter("camera_l_serial", _env("CAMERA_L_SERIAL", "AU1SB3300YB"))
        self.declare_parameter("camera_r_serial", _env("CAMERA_R_SERIAL", "AU1SB33005A"))

        self.width = int(self.get_parameter("width").value)
        self.height = int(self.get_parameter("height").value)
        self.fps = int(self.get_parameter("fps").value)
        self.fourcc = str(self.get_parameter("fourcc").value).strip().upper()
        period = 1.0 / max(self.fps, 1)

        self.cameras = [
            CameraSpec(
                "camera_f",
                "/camera_f/color/image_raw",
                str(self.get_parameter("camera_f_device").value),
                str(self.get_parameter("camera_f_serial").value),
            ),
            CameraSpec(
                "camera_l",
                "/camera_l/color/image_raw",
                str(self.get_parameter("camera_l_device").value),
                str(self.get_parameter("camera_l_serial").value),
            ),
            CameraSpec(
                "camera_r",
                "/camera_r/color/image_raw",
                str(self.get_parameter("camera_r_device").value),
                str(self.get_parameter("camera_r_serial").value),
            ),
        ]

        for spec in self.cameras:
            resolved = _resolve_device(spec.device, spec.serial)
            if resolved == "":
                raise RuntimeError(
                    f"No device found for {spec.name}; set {spec.name.upper()}_DEVICE or {spec.name.upper()}_SERIAL."
                )
            spec.publisher = self.create_publisher(Image, spec.topic, qos_profile_sensor_data)
            spec.device = str(resolved)
            spec.capture = self._open_capture(spec)
            if spec.capture is not None:
                self._log_capture_opened(spec)
            else:
                self.get_logger().warning(
                    f"{spec.name}: {spec.device!r} -> {spec.topic} is not ready; retrying in background."
                )

        self.timer = self.create_timer(period, self.publish_frames)

    def _log_capture_opened(self, spec: CameraSpec) -> None:
        assert spec.capture is not None
        self.get_logger().info(
            f"{spec.name}: {spec.device!r} -> {spec.topic} "
            f"({int(spec.capture.get(cv2.CAP_PROP_FRAME_WIDTH))}x"
            f"{int(spec.capture.get(cv2.CAP_PROP_FRAME_HEIGHT))}, "
            f"{spec.capture.get(cv2.CAP_PROP_FPS):.1f} fps, fourcc={self.fourcc or 'default'})"
        )

    def _open_capture(self, spec: CameraSpec) -> cv2.VideoCapture | None:
        device = int(spec.device.removeprefix("/dev/video")) if spec.device.startswith("/dev/video") else spec.device
        capture = cv2.VideoCapture(device, cv2.CAP_V4L2)
        if self.fourcc:
            capture.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*self.fourcc[:4]))
        capture.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
        capture.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
        capture.set(cv2.CAP_PROP_FPS, self.fps)
        if not capture.isOpened():
            capture.release()
            spec.failed_reads += 1
            spec.last_reopen_s = time.monotonic()
            return None
        for _ in range(5):
            capture.grab()
        spec.failed_reads = 0
        spec.last_reopen_s = time.monotonic()
        return capture

    def _reopen_capture(self, spec: CameraSpec) -> None:
        now_s = time.monotonic()
        if now_s - spec.last_reopen_s < 2.0:
            return
        self.get_logger().warning(f"Reopening {spec.name} after repeated read failures ({spec.device}).")
        if spec.capture is not None:
            spec.capture.release()
        time.sleep(0.2)
        spec.capture = self._open_capture(spec)
        if spec.capture is not None:
            self._log_capture_opened(spec)

    def publish_frames(self) -> None:
        stamp = self.get_clock().now().to_msg()
        for spec in self.cameras:
            if spec.capture is None:
                self._reopen_capture(spec)
                continue
            ok, frame = spec.capture.read()
            if not ok or frame is None:
                spec.failed_reads += 1
                if spec.failed_reads == 1 or spec.failed_reads % 30 == 0:
                    self.get_logger().warning(
                        f"Failed to read frame from {spec.name} ({spec.device}); "
                        f"consecutive failures={spec.failed_reads}."
                    )
                if spec.failed_reads >= 30:
                    self._reopen_capture(spec)
                time.sleep(0.01)
                continue
            spec.failed_reads = 0

            if frame.shape[1] != self.width or frame.shape[0] != self.height:
                frame = cv2.resize(frame, (self.width, self.height), interpolation=cv2.INTER_AREA)
            rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

            msg = Image()
            msg.header.stamp = stamp
            msg.header.frame_id = spec.name
            msg.height = self.height
            msg.width = self.width
            msg.encoding = "rgb8"
            msg.is_bigendian = False
            msg.step = self.width * 3
            msg.data = rgb.tobytes()
            spec.publisher.publish(msg)

    def destroy_node(self) -> bool:
        for spec in self.cameras:
            if spec.capture is not None:
                spec.capture.release()
        return super().destroy_node()


def main() -> None:
    rclpy.init()
    node = CobotMagicCameraNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
