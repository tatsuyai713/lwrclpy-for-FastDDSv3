#!/usr/bin/env python3
"""Minimal service server using std_msgs/srv/SetBool."""
import rclpy
from std_msgs.srv import SetBool


def handle_set_bool(request: SetBool.Request, response: SetBool.Response):
    response.success = True
    response.message = "got flag=%s" % request.data
    return response


def main():
    rclpy.init()
    node = rclpy.Node("set_bool_server")
    node.get_logger().info("Starting SetBool server on /set_flag")
    srv = node.create_service(SetBool, "set_flag", handle_set_bool)
    try:
        rclpy.spin(node)
    finally:
        srv.destroy()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
