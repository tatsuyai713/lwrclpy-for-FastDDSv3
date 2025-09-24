#!/usr/bin/env python3
import lwrclpy as rclpy
from lwrclpy import QoSProfile
from std_msgs.msg import String


def main():
    rclpy.init()
    node = rclpy.Node("talker")
    pub = node.create_publisher(String, "chatter", QoSProfile(depth=10))

    msg = String()
    i = 0
    rate = node.create_rate(10.0)  # 10 Hz 周期で送信

    try:
        while rclpy.ok():
            msg.data(f"hello {i}")
            pub.publish(msg)
            print(f"[send] {msg.data()}")
            i += 1
            rate.sleep()  # 周期同期
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()