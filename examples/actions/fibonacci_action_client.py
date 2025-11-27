#!/usr/bin/env python3
"""Minimal Fibonacci action client using the lwrclpy ActionClient."""

import rclpy
from rclpy.action import ActionClient
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from action_msgs.msg import GoalStatus
from example_interfaces.action import Fibonacci


class FibonacciActionClient(Node):
    def __init__(self):
        super().__init__("fibonacci_action_client")
        self._action_client = ActionClient(self, Fibonacci, "fibonacci")

    def send_goal(self, order: int):
        self.get_logger().info("Waiting for action server...")
        self._action_client.wait_for_server()

        goal_msg = Fibonacci.Goal()
        goal_msg.order = order
        self.get_logger().info(f"Sending goal request order={order}")

        send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback,
        )
        send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle or not goal_handle.accepted:
            self.get_logger().error("Goal rejected :(")
            return
        self.get_logger().info("Goal accepted :)")
        get_result_future = goal_handle.get_result_async()
        get_result_future.add_done_callback(self.get_result_callback)

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info(f"Feedback: {feedback.sequence}")

    def get_result_callback(self, future):
        result_msg = future.result()
        result = result_msg.result
        status = result_msg.status
        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info(f"Result: {result.sequence}")
        else:
            self.get_logger().warn(f"Goal finished with status {status}")
        rclpy.shutdown()


def main(args=None):
    try:
        with rclpy.init(args=args):
            action_client = FibonacciActionClient()
            action_client.send_goal(order=10)
            rclpy.spin(action_client)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass


if __name__ == "__main__":
    main()
