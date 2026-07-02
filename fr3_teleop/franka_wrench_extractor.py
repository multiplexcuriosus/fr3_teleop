#!/usr/bin/env python3

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile

from franka_msgs.msg import FrankaState
from std_msgs.msg import Float32MultiArray


class FrankaWrenchExtractor(Node):
    def __init__(self):
        super().__init__("franka_wrench_extractor")

        self.declare_parameter(
            "robot_state_topic",
            "/right_franka_robot_state_broadcaster/robot_state",
        )
        self.declare_parameter(
            "output_topic",
            "/right_franka/external_wrenches",
        )
        self.declare_parameter("publish_hz", 100.0)

        self.robot_state_topic = (
            self.get_parameter("robot_state_topic").get_parameter_value().string_value
        )
        self.output_topic = (
            self.get_parameter("output_topic").get_parameter_value().string_value
        )
        self.publish_hz = (
            self.get_parameter("publish_hz").get_parameter_value().double_value
        )

        if self.publish_hz <= 0.0 or not math.isfinite(self.publish_hz):
            raise ValueError("publish_hz must be a finite value > 0")

        self.publish_period_sec = 1.0 / self.publish_hz
        self.last_publish_time_sec = None

        qos = QoSProfile(depth=10)
        self.pub = self.create_publisher(Float32MultiArray, self.output_topic, qos)
        self.sub = self.create_subscription(
            FrankaState,
            self.robot_state_topic,
            self._robot_state_cb,
            qos,
        )

        self.get_logger().info(
            "Franka wrench extractor running: "
            f"{self.robot_state_topic} -> {self.output_topic} "
            f"at max {self.publish_hz:.1f} Hz"
        )
        self.get_logger().info(
            "Output layout: "
            "[o_Fx, o_Fy, o_Fz, o_Tx, o_Ty, o_Tz, "
            "k_Fx, k_Fy, k_Fz, k_Tx, k_Ty, k_Tz]"
        )

    def _robot_state_cb(self, msg: FrankaState):
        now_sec = self.get_clock().now().nanoseconds * 1e-9

        if self.last_publish_time_sec is not None:
            if now_sec - self.last_publish_time_sec < self.publish_period_sec:
                return

        o_wrench = list(msg.o_f_ext_hat_k)
        k_wrench = list(msg.k_f_ext_hat_k)

        if len(o_wrench) != 6 or len(k_wrench) != 6:
            self.get_logger().warn(
                f"Unexpected wrench lengths: o={len(o_wrench)}, k={len(k_wrench)}"
            )
            return

        out = Float32MultiArray()
        out.data = [float(v) for v in (o_wrench + k_wrench)]

        self.pub.publish(out)
        self.last_publish_time_sec = now_sec


def main(args=None):
    rclpy.init(args=args)
    node = FrankaWrenchExtractor()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
