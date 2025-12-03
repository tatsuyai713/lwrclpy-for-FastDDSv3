"""Read an MP4/MOV loop and publish each frame as sensor_msgs/Image."""

import argparse

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import Image


def _set_header(msg: Image, stamp, frame_id: str) -> None:
    hdr = msg.header()
    st = hdr.stamp()
    st.sec(getattr(stamp, "sec", 0) if callable(getattr(stamp, "sec", None)) else getattr(stamp, "sec", 0))
    st.nanosec(
        getattr(stamp, "nanosec", 0) if callable(getattr(stamp, "nanosec", None)) else getattr(stamp, "nanosec", 0)
    )
    hdr.frame_id(frame_id)


def frame_to_image_msg(frame: np.ndarray, stamp, frame_id="video_frame") -> Image:
    msg = Image()
    # attribute-style assignment (rclpy compatible); compat layer maps to SWIG setters
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


class VideoPublisher(Node):
    def __init__(self, video_path: str, topic: str, rate: float):
        super().__init__("mac_video_publisher")
        self.cap = cv2.VideoCapture(video_path)
        if not self.cap.isOpened():
            raise RuntimeError(f"Unable to open video file: {video_path}")
        qos = QoSProfile(
            depth=5,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
        )
        self.publisher = self.create_publisher(Image, topic, qos)
        self.get_logger().info(f"Publishing {video_path} to '{topic}' at {rate} Hz")
        self.timer = self.create_timer(1.0 / rate, self.publish_frame)

    def publish_frame(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().info("Video reached end. Rewinding.")
            self.cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
            return
        stamp = self.get_clock().now().to_msg()
        msg = frame_to_image_msg(frame, stamp)
        self.publisher.publish(msg)


def main():
    parser = argparse.ArgumentParser(description="Publish MP4 frames as sensor_msgs/Image")
    parser.add_argument("--video", "-v", required=True, help="Path to the video file (MP4/MOV)")
    parser.add_argument("--topic", "-t", default="video/image_raw", help="Topic name to publish frames")
    parser.add_argument("--rate", "-r", type=float, default=24.0, help="Publish rate in Hz (default: 24)")
    args = parser.parse_args()

    rclpy.init()
    try:
        node = VideoPublisher(args.video, args.topic, args.rate)
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
