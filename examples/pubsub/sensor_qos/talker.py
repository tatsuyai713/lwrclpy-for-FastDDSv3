#!/usr/bin/env python3
"""Talker using sensor_data QoS profile (best-effort)."""
import time
import rclpy
from rclpy.qos import QoSProfile
from std_msgs.msg import String


def main():
    rclpy.init()
    node = rclpy.Node("sensor_qos_talker")
    qos = QoSProfile.sensor_data()
    pub = node.create_publisher(String, "sensor/chatter", qos)
    msg = String()
    i = 0
    try:
        while rclpy.ok() and i < 20:
            msg.data = f"sensor tick {i}"
            pub.publish(msg)
            print(f"[send] {msg.data}")
            i += 1
            time.sleep(0.05)
    finally:
        pub.destroy()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
