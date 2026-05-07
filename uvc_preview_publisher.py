#!/usr/bin/env python3
from __future__ import annotations

import argparse
import logging
import os
import socket
import signal
import sys
import time
from pathlib import Path

import cv2


def ensure_ros_python_available() -> None:
    for path in (
        Path("/opt/ros/noetic/lib/python3/dist-packages"),
        Path("/usr/lib/python3/dist-packages"),
    ):
        text = str(path)
        if path.exists() and text not in sys.path:
            sys.path.append(text)


def normalize_device_path(source: str) -> str:
    text = str(source).strip()
    if text.isdigit():
        return f"/dev/video{text}"
    return text


def wait_for_ros_publisher(topic: str):
    ensure_ros_python_available()
    import rosgraph.roslogging  # type: ignore
    import rospy  # type: ignore
    from sensor_msgs.msg import CompressedImage  # type: ignore

    for name, level in (
        ("CRITICAL", logging.CRITICAL),
        ("ERROR", logging.ERROR),
        ("WARNING", logging.WARNING),
        ("WARN", logging.WARNING),
        ("INFO", logging.INFO),
        ("DEBUG", logging.DEBUG),
        ("NOTSET", logging.NOTSET),
    ):
        logging._nameToLevel.setdefault(name, level)
        logging._levelToName.setdefault(level, name)

    original_configure_logging = rosgraph.roslogging.configure_logging

    def safe_configure_logging(*args, **kwargs):
        try:
            return original_configure_logging(*args, **kwargs)
        except Exception:
            ros_home = Path(os.environ.get("ROS_HOME", "/tmp/autorun_final_ros_home"))
            log_dir = ros_home / "log"
            log_dir.mkdir(parents=True, exist_ok=True)
            logging.basicConfig(level=logging.INFO)
            return str(log_dir / "rospy.log")

    rosgraph.roslogging.configure_logging = safe_configure_logging
    if not rospy.core.is_initialized():
        rospy.init_node("autorun_final_uvc_preview", anonymous=True, disable_signals=True)
    return rospy, rospy.Publisher(topic, CompressedImage, queue_size=1), CompressedImage


def ros_master_is_ready(timeout_sec: float = 0.2) -> bool:
    uri = os.environ.get("ROS_MASTER_URI", "http://localhost:11311")
    try:
        host_port = uri.split("://", 1)[1]
        host, port_text = host_port.rsplit(":", 1)
        port = int(port_text)
        with socket.create_connection((host, port), timeout=timeout_sec):
            return True
    except Exception:
        return False


def open_capture(source: str, width: int, height: int, fps: int, fourcc: str) -> cv2.VideoCapture:
    source = normalize_device_path(source)
    if source.isdigit():
        capture = cv2.VideoCapture(int(source))
    elif source.startswith("/dev/video"):
        capture = cv2.VideoCapture(source, cv2.CAP_V4L2)
    else:
        capture = cv2.VideoCapture(source)
    if not capture.isOpened():
        raise RuntimeError(f"failed to open UVC source: {source}")
    capture.set(cv2.CAP_PROP_FRAME_WIDTH, max(1, int(width)))
    capture.set(cv2.CAP_PROP_FRAME_HEIGHT, max(1, int(height)))
    capture.set(cv2.CAP_PROP_FPS, max(1, int(fps)))
    if fourcc:
        capture.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*fourcc[:4]))
    return capture


def prepare_preview(frame, width: int, height: int):
    target_width = max(1, int(width))
    target_height = max(1, int(height))
    h, w = frame.shape[:2]
    scale = min(target_width / float(w), target_height / float(h))
    scale = max(scale, 1e-6)
    resized_w = max(1, int(w * scale))
    resized_h = max(1, int(h * scale))
    interpolation = cv2.INTER_AREA if scale < 1.0 else cv2.INTER_LINEAR
    return cv2.resize(frame, (resized_w, resized_h), interpolation=interpolation)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="Publish lightweight UVC preview to a ROS compressed topic")
    parser.add_argument("--source", default="/dev/video0")
    parser.add_argument("--camera-width", type=int, default=1024)
    parser.add_argument("--camera-height", type=int, default=768)
    parser.add_argument("--camera-fps", type=int, default=10)
    parser.add_argument("--camera-fourcc", default="MJPG")
    parser.add_argument("--image-topic", default="/autorun_final/uvc_preview/compressed")
    parser.add_argument("--preview-width", type=int, default=640)
    parser.add_argument("--preview-height", type=int, default=360)
    parser.add_argument("--jpeg-quality", type=int, default=35)
    parser.add_argument("--publish-fps", type=float, default=6.0)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    stop_requested = False

    def _handle_stop(signum, frame):
        del signum, frame
        nonlocal stop_requested
        stop_requested = True

    signal.signal(signal.SIGINT, _handle_stop)
    signal.signal(signal.SIGTERM, _handle_stop)

    capture = open_capture(
        args.source,
        args.camera_width,
        args.camera_height,
        args.camera_fps,
        args.camera_fourcc,
    )
    try:
        while not stop_requested and not ros_master_is_ready():
            time.sleep(0.3)
        if stop_requested:
            return 0
        rospy, publisher, compressed_type = wait_for_ros_publisher(args.image_topic)
        min_interval = 0.0 if args.publish_fps <= 0 else (1.0 / float(args.publish_fps))
        last_publish_at = 0.0
        jpeg_quality = max(10, min(80, int(args.jpeg_quality)))
        while not stop_requested and not rospy.is_shutdown():
            ok, frame = capture.read()
            if not ok or frame is None:
                time.sleep(0.05)
                continue
            now = time.monotonic()
            if min_interval > 0.0 and (now - last_publish_at) < min_interval:
                continue
            preview = prepare_preview(frame, args.preview_width, args.preview_height)
            ok_encoded, encoded = cv2.imencode(".jpg", preview, [cv2.IMWRITE_JPEG_QUALITY, jpeg_quality])
            if not ok_encoded:
                time.sleep(0.02)
                continue
            msg = compressed_type()
            msg.header.stamp = rospy.Time.now()
            msg.format = "jpeg"
            msg.data = encoded.tobytes()
            publisher.publish(msg)
            last_publish_at = now
    finally:
        capture.release()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
