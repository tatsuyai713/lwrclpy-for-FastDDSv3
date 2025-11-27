#!/usr/bin/env python3
"""Listener for the oneshot_and_periodic timer talker on timer/combo."""
import rclpy
from std_msgs.msg import String


def main():
    rclpy.init()
    node = rclpy.Node("timer_combo_listener")

    def on_msg(msg: String):
        print(f"[combo recv] {msg.data}")

    node.create_subscription(String, "timer/combo", on_msg, 10)
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
