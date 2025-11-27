#!/usr/bin/env python3
"""Minimal Fibonacci action server built on lwrclpy's ActionServer."""

import time

import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node

from example_interfaces.action import Fibonacci


class FibonacciActionServer(Node):
    def __init__(self):
        super().__init__("fibonacci_action_server")
        self._action_server = ActionServer(
            self,
            Fibonacci,
            "fibonacci",
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
        )

    def goal_callback(self, goal_request):
        self.get_logger().info(f"Received goal request: order={goal_request.order}")
        if goal_request.order <= 0:
            self.get_logger().warn("Rejecting goal with non-positive order.")
            return GoalResponse.REJECT
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        self.get_logger().info("Received cancel request.")
        return CancelResponse.ACCEPT

    def execute_callback(self, goal_handle):
        self.get_logger().info(f"Executing goal order={goal_handle.request.order}")
        feedback_msg = Fibonacci.Feedback()
        feedback_msg.sequence = [0, 1]

        for i in range(1, goal_handle.request.order):
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                result = Fibonacci.Result()
                result.sequence = feedback_msg.sequence
                self.get_logger().info("Goal canceled.")
                return result

            feedback_msg.sequence.append(feedback_msg.sequence[i] + feedback_msg.sequence[i - 1])
            goal_handle.publish_feedback(feedback_msg)
            time.sleep(0.5)

        result = Fibonacci.Result()
        result.sequence = feedback_msg.sequence
        goal_handle.succeed()
        self.get_logger().info(f"Goal succeeded with sequence: {result.sequence}")
        return result


def main(args=None):
    try:
        with rclpy.init(args=args):
            action_server = FibonacciActionServer()
            rclpy.spin(action_server)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass


if __name__ == "__main__":
    main()
