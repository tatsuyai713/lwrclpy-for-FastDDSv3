#!/usr/bin/env python3
"""Guard condition demo showing the new Node.create_guard_condition API."""

import rclpy
from rclpy.executors import ExternalShutdownException


def main(args=None):
    try:
        with rclpy.init(args=args):
            node = rclpy.create_node("guard_condition_demo")
            executor = rclpy.executors.SingleThreadedExecutor()
            executor.add_node(node)

            def guard_condition_callback():
                node.get_logger().info("Guard callback triggered – shutting down.")
                rclpy.shutdown()

            def timer_callback():
                node.get_logger().info("Timer fired – triggering guard condition.")
                guard_condition.trigger()

            guard_condition = node.create_guard_condition(guard_condition_callback)
            node.create_timer(2.0, timer_callback)

            while rclpy.ok():
                node.get_logger().info("Waiting for spin_once to finish…")
                executor.spin_once()
    except (KeyboardInterrupt, ExternalShutdownException):
        pass


if __name__ == "__main__":
    main()
