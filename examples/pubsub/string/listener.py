#!/usr/bin/env python3
import lwrclpy as rclpy
from std_msgs.msg import String

def callback(msg: String):
    print(f"[recv] {msg.data()}")

def main():
    rclpy.init()
    node = rclpy.Node("listener")
    _ = node.create_subscription(String, "chatter", callback, 10)
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()