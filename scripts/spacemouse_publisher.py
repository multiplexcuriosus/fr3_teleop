#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import pyspacemouse


class SpaceMousePublisher(Node):
    def __init__(self):
        super().__init__("spacemouse_publisher")
        self.publisher_ = self.create_publisher(Twist, "spacemouse_cmd", 10)

        self.device = pyspacemouse.open()
        if not self.device:
            self.get_logger().error("Failed to open SpaceMouse device.")
            raise RuntimeError("Failed to open SpaceMouse device")

        self.timer = self.create_timer(0.05, self.timer_callback)

    def timer_callback(self):
        state = pyspacemouse.read()
        if state is None:
            return

        msg = Twist()
        msg.linear.x = float(state.x)
        msg.linear.y = float(state.y)
        msg.linear.z = float(state.z)
        msg.angular.x = float(state.roll)
        msg.angular.y = float(state.pitch)
        msg.angular.z = float(state.yaw)
        self.publisher_.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    spacemouse_publisher = None

    try:
        spacemouse_publisher = SpaceMousePublisher()
        rclpy.spin(spacemouse_publisher)
    except KeyboardInterrupt:
        pass
    except Exception as exc:
        if spacemouse_publisher is not None:
            spacemouse_publisher.get_logger().error(f"SpaceMouse node failed: {exc}")
        else:
            print(f"SpaceMouse node failed: {exc}")
    finally:
        if spacemouse_publisher is not None:
            spacemouse_publisher.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
