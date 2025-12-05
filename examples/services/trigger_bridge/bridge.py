#!/usr/bin/env python3
"""Service + client in one process to show round-trip using new API."""
import rclpy
from std_srvs.srv import Trigger


def main():
    rclpy.init()
    node = rclpy.Node("service_echo_bridge")

    def on_trigger(req: Trigger.Request, res: Trigger.Response):
        res.success = True
        res.message = "pong"
        return res

    srv = node.create_service(Trigger, "ping", on_trigger)
    client = node.create_client(Trigger, "ping")
    client.wait_for_service()

    req = Trigger.Request()
    for i in range(3):
        resp = client.call(req, timeout=1.0)
        node.get_logger().info(f"call {i}: success={resp.success} msg={resp.message}")

    client.destroy()
    srv.destroy()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
