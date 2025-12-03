"""Subscribe to sensor_msgs/Image, run YOLO, and publish annotated frames."""

import argparse

import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from ultralytics import YOLO
from lwrclpy.qos import QoSProfile


def default_device() -> str:
    try:
        import torch

        if torch.backends.mps.is_available():
            return "mps"
        if torch.cuda.is_available():
            return "cuda"
    except ImportError:  # pragma: no cover
        pass
    return "cpu"


def _get_field(obj, name):
    attr = getattr(obj, name, None)
    if callable(attr):
        return attr()
    return attr


def image_msg_to_numpy(msg: Image) -> np.ndarray:
    h = _get_field(msg, "height")
    w = _get_field(msg, "width")
    data = _get_field(msg, "data")
    return np.frombuffer(data, np.uint8).reshape((h, w, 3))


def numpy_to_image_msg(frame: np.ndarray, stamp, frame_id="video_frame") -> Image:
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


class VideoYoloDetector(Node):
    def __init__(self, in_topic: str, out_topic: str, model_path: str, device: str):
        super().__init__("mac_video_yolo_detector")
        self.model = YOLO(model_path)
        self.device = device
        if self.device != "cpu":
            self.model.to(self.device)
        sensor_qos = QoSProfile.sensor_data()
        self.subscription = self.create_subscription(Image, in_topic, self.handle_frame, sensor_qos)
        self.publisher = self.create_publisher(Image, out_topic, sensor_qos)

    def handle_frame(self, msg: Image):
        print("Received frame")
        frame = image_msg_to_numpy(msg)
        results = self.model(frame, verbose=False, device=self.device)[0]
        annotated = results.plot()
        annotated_msg = numpy_to_image_msg(annotated, msg.header.stamp)
        self.publisher.publish(annotated_msg)


def main():
    parser = argparse.ArgumentParser(description="Run YOLO on sensor_msgs/Image over lwrclpy")
    parser.add_argument(
        "--input-topic",
        "-i",
        default="video/image_raw",
        help="Image topic to subscribe to",
    )
    parser.add_argument(
        "--output-topic",
        "-o",
        default="video/image_annotated",
        help="Annotated image topic",
    )
    parser.add_argument(
        "--model",
        "-m",
        default="yolov8n.pt",
        help="Ultralytics model path/spec (default: yolov8n.pt)",
    )
    parser.add_argument(
        "--device",
        "-d",
        default=default_device(),
        help="Inference device (cpu/mps/cuda)",
    )
    args = parser.parse_args()

    rclpy.init()
    try:
        node = VideoYoloDetector(args.input_topic, args.output_topic, args.model, args.device)
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
