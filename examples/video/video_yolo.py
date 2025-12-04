"""Play an MP4 file, publish each frame as sensor_msgs/Image, and run an Ultralytics YOLO detector."""

import argparse

import cv2
import lwrclpy
import numpy as np
import torch
from lwrclpy.executors import SingleThreadedExecutor
from lwrclpy.node import Node
from sensor_msgs.msg import Image
from ultralytics import YOLO
from lwrclpy.qos import QoSProfile


def default_device() -> str:
    if torch.backends.mps.is_available():
        return "mps"
    if torch.cuda.is_available():
        return "cuda"
    return "cpu"


def _set_header(msg: Image, stamp, frame_id: str) -> None:
    hdr = msg.header()
    st = hdr.stamp()
    st.sec(getattr(stamp, "sec", 0) if callable(getattr(stamp, "sec", None)) else getattr(stamp, "sec", 0))
    st.nanosec(
        getattr(stamp, "nanosec", 0) if callable(getattr(stamp, "nanosec", None)) else getattr(stamp, "nanosec", 0)
    )
    hdr.frame_id(frame_id)


def frame_to_image_msg(frame: np.ndarray, stamp, frame_id: str = "camera_frame"):
    msg = Image()
    msg.header = msg.header() if callable(getattr(msg, "header", None)) else msg.header
    msg.header.stamp = stamp
    msg.header.frame_id = frame_id
    h, w = frame.shape[:2]
    msg.height = h
    msg.width = w
    msg.encoding = "bgr8"
    msg.is_bigendian = 0
    msg.step = w * 3
    msg.data = frame.tobytes()
    return msg


class VideoYoloNode(Node):
    def __init__(self, video_path: str, model_path: str, device: str, publish_rate: float):
        super().__init__("mac_video_yolo_node")
        self.video_path = video_path
        self.cap = cv2.VideoCapture(self.video_path)
        if not self.cap.isOpened():
            raise RuntimeError(f"Unable to open video file: {self.video_path}")
        self.model = YOLO(model_path)
        self.device = device
        if self.device:
            self.model.to(self.device)
        sensor_qos = QoSProfile.sensor_data()
        self.image_pub = self.create_publisher(Image, "video/image_raw", sensor_qos)
        self.annotated_pub = self.create_publisher(Image, "video/image_annotated", sensor_qos)
        self.timer = self.create_timer(1.0 / publish_rate, self.publish_frame)

    def publish_frame(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().info("Video reached end, rewinding.")
            self.cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
            return

        stamp = self.get_clock().now().to_msg()
        raw_msg = frame_to_image_msg(frame, stamp)
        self.image_pub.publish(raw_msg)

        results = self.model(frame, verbose=False, device=self.device)[0]
        annotated = results.plot()
        annotated_msg = frame_to_image_msg(annotated, stamp)
        self.annotated_pub.publish(annotated_msg)


def main():
    parser = argparse.ArgumentParser(description="Publish video frames and YOLO detections over lwrclpy")
    parser.add_argument(
        "--video",
        "-v",
        required=True,
        help="Path to an MP4/MOV file that cv2 can read.",
    )
    parser.add_argument(
        "--model",
        "-m",
        default="yolov8n.pt",
        help="Path or Ultralytics model spec (default: yolov8n.pt).",
    )
    parser.add_argument(
        "--device",
        "-d",
        default=default_device(),
        help="Inference device (e.g. 'cpu', 'mps', 'cuda').",
    )
    parser.add_argument(
        "--rate",
        "-r",
        type=float,
        default=24.0,
        help="Target publish rate in Hz (default: 24).",
    )
    args = parser.parse_args()

    lwrclpy.init()
    try:
        node = VideoYoloNode(video_path=args.video, model_path=args.model, device=args.device, publish_rate=args.rate)
        executor = SingleThreadedExecutor()
        executor.add_node(node)
        executor.spin()
    finally:
        lwrclpy.shutdown()


if __name__ == "__main__":
    main()
