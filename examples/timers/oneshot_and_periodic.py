#!/usr/bin/env python3
"""One-shot + periodic timers: publish via periodic, adjust with oneshot."""
import rclpy
from std_msgs.msg import String


def main():
    rclpy.init()
    node = rclpy.Node("timer_combo")
    log = node.get_logger()
    count = {"n": 0}
    pub = node.create_publisher(String, "timer/combo", 10)

    def periodic():
        count["n"] += 1
        msg = String()
        msg.data = f"periodic tick {count['n']}"
        pub.publish(msg)
        log.info(msg.data)
        if count["n"] >= 30:
            log.info("stopping periodic timer after 30 messages and shutting down")
            periodic_timer.cancel()
            rclpy.shutdown()

    def oneshot():
        log.info("oneshot fired; resetting periodic schedule")
        periodic_timer.reset()

    periodic_timer = node.create_wall_timer(1.0, periodic)
    oneshot_timer = node.create_timer(0.5, oneshot, oneshot=True)  # queue through executor

    try:
        rclpy.spin(node)
    finally:
        periodic_timer.cancel()
        oneshot_timer.cancel()
        node.destroy_node()
        rclpy.shutdown()
        log.info("Completed 30 periodic timer messages; shutting down cleanly.")


if __name__ == "__main__":
    main()
