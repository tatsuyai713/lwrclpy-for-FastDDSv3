#!/usr/bin/env python3
import lwrclpy as rclpy
from std_msgs.msg import String

def on_msg(msg: String):
    print("[recv]", msg.data())

def main():
    rclpy.init()
    node = rclpy.Node("ml_listener")
    _ = node.create_subscription(String, "ml/topic", on_msg, 10)
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()