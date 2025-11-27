#!/usr/bin/env python3
"""Listener for the wall_timer_talker publishing on timer/chatter."""
import rclpy
from std_msgs.msg import String


def main():
    rclpy.init()
    node = rclpy.Node("wall_timer_listener")

    def on_msg(msg: String):
        print(f"[timer recv] {msg.data}")

    node.create_subscription(String, "timer/chatter", on_msg, 10)
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
