#!/usr/bin/env python3
"""Minimal service client using std_srvs/srv/SetBool."""
import time

import rclpy
from std_srvs.srv import SetBool


def main():
    rclpy.init()
    node = rclpy.Node("set_bool_client")
    client = node.create_client(SetBool, "set_flag")
    client.wait_for_service()
    req = SetBool.Request()

    for i in range(5):
        req.data = (i % 2) == 0
        node.get_logger().info(f"request {i}: data={req.data}")
        resp = client.call(req, timeout=5.0)
        if resp is None:
            node.get_logger().warning("No response yet (service not available?). retrying...")
            time.sleep(0.5)
            continue
        node.get_logger().info(f"response {i}: success={resp.success} msg={resp.message}")
        time.sleep(0.5)

    client.destroy()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
