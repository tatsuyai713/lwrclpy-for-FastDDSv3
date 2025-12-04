"""Subscribe to sensor_msgs/Image and republish to another topic (relay)."""

import argparse

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from lwrclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy


class VideoRelay(Node):
    def __init__(self, in_topic: str, out_topic: str):
        super().__init__("mac_video_relay")
        sub_qos = QoSProfile.sensor_data()  # best-effort, depth=5 (ROS2 SensorData equivalent)
        # Publish reliable so viewers expecting RELIABLE still receive
        pub_qos = QoSProfile(
            depth=sub_qos.depth,
            history=sub_qos.history,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self._sub = self.create_subscription(Image, in_topic, self._handle_frame, sub_qos)
        self._pub = self.create_publisher(Image, out_topic, pub_qos)
        self._count = 0
        self.get_logger().info(f"Relaying '{in_topic}' -> '{out_topic}' (sub: sensor_data, pub: reliable)")

    def _handle_frame(self, msg: Image):
        self._count += 1
        if self._count <= 5 or self._count % 30 == 0:
            self.get_logger().info(f"Relay frame {self._count}")
        # Forward as-is; Publisher will clone if必要
        self._pub.publish(msg)


def main():
    parser = argparse.ArgumentParser(description="Relay sensor_msgs/Image from one topic to another")
    parser.add_argument("--input-topic", "-i", default="video/image_raw", help="Input image topic")
    parser.add_argument("--output-topic", "-o", default="video/image_relay", help="Output image topic")
    args = parser.parse_args()

    rclpy.init()
    try:
        node = VideoRelay(args.input_topic, args.output_topic)
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
