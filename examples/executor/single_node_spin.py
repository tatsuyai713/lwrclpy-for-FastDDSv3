#!/usr/bin/env python3
"""Single node spin example using rclpy.spin(node)."""
import time
import lwrclpy as rclpy
from std_msgs.msg import String


def main():
    rclpy.init()
    node = rclpy.Node("single_spin_loopback")
    pub = node.create_publisher(String, "chatter", 10)
    count = {"n": 0}

    def on_msg(msg: String):
        print(f"[single] recv: {msg.data}")

    node.create_subscription(String, "chatter", on_msg, 10)
    # Give DDS discovery a brief moment before starting to publish.
    time.sleep(0.5)

    def on_timer():
        msg = String()
        msg.data = f"single message {count['n']}"
        pub.publish(msg)
        print(f"[single] send: {msg.data}")
        count["n"] += 1
        if count["n"] >= 30:
            print("[single] finished sending 30 messages; shutting down.")
            timer.cancel()
            rclpy.shutdown()

    timer = node.create_wall_timer(0.2, on_timer)
    try:
        rclpy.spin(node)
    finally:
        timer.cancel()
        node.destroy_node()
        rclpy.shutdown()
        print("[single] spin stopped and node destroyed cleanly.")


if __name__ == "__main__":
    main()
