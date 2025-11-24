#!/usr/bin/env python3
"""MultiThreadedExecutor demo: two nodes each with its own topic in parallel."""
import time
import lwrclpy as rclpy
from lwrclpy import MultiThreadedExecutor
from std_msgs.msg import String


def make_talker(name, topic):
    node = rclpy.Node(name)
    pub = node.create_publisher(String, topic, 10)
    msg = String()
    count = {"n": 0}

    def on_timer():
        msg.data = f"{name} msg {count['n']}"
        pub.publish(msg)
        print(f"[{name}] send: {msg.data}")
        count["n"] += 1
        if count["n"] >= 30:
            print(f"[{name}] finished sending {count['n']} messages")
            timer.cancel()
    timer = node.create_wall_timer(0.2, on_timer)
    return node


def make_listener(name, topic):
    node = rclpy.Node(name)

    def on_msg(msg: String):
        print(f"[{name}] recv: {msg.data}")

    node.create_subscription(String, topic, on_msg, 10)
    return node


def main():
    rclpy.init()
    talker_a = make_talker("talker_a", "mt/chatter_a")
    talker_b = make_talker("talker_b", "mt/chatter_b")
    listener_a = make_listener("listener_a", "mt/chatter_a")
    listener_b = make_listener("listener_b", "mt/chatter_b")

    executor = MultiThreadedExecutor()
    for n in (talker_a, talker_b, listener_a, listener_b):
        executor.add_node(n)

    try:
        executor.spin()
    finally:
        executor.shutdown()
        print("Completed 30 messages per talker; shutting down cleanly.")
        for n in (talker_a, talker_b, listener_a, listener_b):
            n.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
