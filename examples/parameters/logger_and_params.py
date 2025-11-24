#!/usr/bin/env python3
"""Parameter declare/get/set demo with logging."""
import time
import lwrclpy as rclpy


def main():
    rclpy.init()
    node = rclpy.Node(
        "param_logger_demo",
        allow_undeclared_parameters=False,
        parameters=[("greeting", "hello"), ("rate_hz", 2)],
    )
    log = node.get_logger()

    log.info(f"Declared greeting={node.get_parameter('greeting').value}")
    log.info(f"Declared rate_hz={node.get_parameter('rate_hz').value}")

    rate = node.create_rate(node.get_parameter("rate_hz").value)
    count = 0
    try:
        while rclpy.ok() and count < 30:
            log.info(f"{node.get_parameter('greeting').value} #{count}")
            if count == 2:
                node.set_parameters([("greeting", "hi again")])
            count += 1
            rate.sleep()
    finally:
        node.destroy_node()
        rclpy.shutdown()
        log.info("Completed 30 greeting logs; shutting down cleanly.")


if __name__ == "__main__":
    main()
