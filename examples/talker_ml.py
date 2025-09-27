#!/usr/bin/env python3
import time
import torch

import lwrclpy as rclpy
from lwrclpy import QoSProfile
from std_msgs.msg import String

def main():
    rclpy.init()
    node = rclpy.Node("ml_talker")
    pub = node.create_publisher(String, "ml/topic", QoSProfile(depth=10))

    # Dummy PyTorch "inference" — pick any model/device/mode without ROS 2 constraints
    x = torch.randn(4, 4)
    w = torch.randn(4, 4)
    for i in range(10):
        y = (x @ w).relu().mean().item()
        msg = String()
        msg.data(f"[step {i}] score={y:.4f}")
        pub.publish(msg)
        time.sleep(0.5)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()