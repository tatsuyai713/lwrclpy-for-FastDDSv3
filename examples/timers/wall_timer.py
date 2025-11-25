#!/usr/bin/env python3
"""Wall timer demo: publish String at a fixed rate using create_wall_timer."""
import rclpy
from std_msgs.msg import String


def main():
    rclpy.init()
    node = rclpy.Node("wall_timer_talker")
    pub = node.create_publisher(String, "timer/chatter", 10)
    count = {"n": 0}

    def on_timer():
        msg = String()
        msg.data = f"wall timer tick {count['n']}"
        pub.publish(msg)
        print(f"[timer send] {msg.data}")
        count["n"] += 1
        if count["n"] >= 30:
            print(f"[wall_timer_talker] finished sending {count['n']} messages")
            timer.cancel()
            rclpy.shutdown()

    timer = node.create_wall_timer(0.5, on_timer)
    try:
        rclpy.spin(node)
    finally:
        timer.cancel()
        node.destroy_node()
        rclpy.shutdown()
        print("Completed 30 wall timer messages; shutting down cleanly.")


if __name__ == "__main__":
    main()
