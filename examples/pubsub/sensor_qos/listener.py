#!/usr/bin/env python3
"""Listener using sensor_data QoS profile (best-effort)."""
import rclpy
from rclpy.qos import QoSProfile
from std_msgs.msg import String


def main():
    rclpy.init()
    node = rclpy.Node("sensor_qos_listener")
    qos = QoSProfile.sensor_data()

    def on_msg(msg: String):
        print(f"[recv] {msg.data}")

    sub = node.create_subscription(String, "sensor/chatter", on_msg, qos)
    try:
        rclpy.spin(node)
    finally:
        sub.destroy()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
