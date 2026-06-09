#!/usr/bin/env python3

import argparse
import csv
import subprocess
from pathlib import Path

import rclpy
from rclpy.node import Node
from rosidl_runtime_py.utilities import get_message


def get_topic_type(topic: str) -> str:
    """Ask ROS2 for the topic type."""
    result = subprocess.run(
        ["ros2", "topic", "type", topic],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
        check=False,
    )

    topic_type = result.stdout.strip()

    if not topic_type:
        raise RuntimeError(
            f"Could not determine type for topic {topic}. "
            f"Is the topic currently being published?"
        )

    # If multiple types are printed, take first non-empty line.
    return topic_type.splitlines()[0].strip()


def get_nested_attr(obj, path: str):
    """Resolve dotted field path, e.g. twist.linear.x."""
    current = obj
    for part in path.split("."):
        current = getattr(current, part)
    return current


def get_msg_stamp_ns(msg, fallback_ns: int) -> int:
    """Use msg.header.stamp if available, otherwise fallback to node time."""
    try:
        stamp = msg.header.stamp
        return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)
    except AttributeError:
        return fallback_ns


class FieldLogger(Node):
    def __init__(self, topic_fields, out_path: Path):
        super().__init__("field_logger")

        self.out_path = out_path
        self.file = open(out_path, "w", newline="")
        self.writer = csv.writer(self.file)

        self.writer.writerow([
            "ros_time_ns",
            "ros_time_s",
            "topic",
            "field",
            "value",
        ])

        self.subscriptions_list = []

        for topic, fields in topic_fields.items():
            topic_type = get_topic_type(topic)
            msg_cls = get_message(topic_type)

            self.get_logger().info(
                f"Subscribing to {topic} [{topic_type}] fields={fields}"
            )

            sub = self.create_subscription(
                msg_cls,
                topic,
                self.make_callback(topic, fields),
                100,
            )
            self.subscriptions_list.append(sub)

    def make_callback(self, topic, fields):
        def callback(msg):
            fallback_ns = self.get_clock().now().nanoseconds
            stamp_ns = get_msg_stamp_ns(msg, fallback_ns)
            stamp_s = stamp_ns * 1e-9

            for field in fields:
                try:
                    value = get_nested_attr(msg, field)
                except AttributeError:
                    self.get_logger().warn(
                        f"Field '{field}' does not exist on topic '{topic}'"
                    )
                    continue

                # Only scalar fields are supported.
                try:
                    value = float(value)
                except Exception:
                    self.get_logger().warn(
                        f"Field '{field}' on topic '{topic}' is not scalar: {value}"
                    )
                    continue

                self.writer.writerow([
                    stamp_ns,
                    f"{stamp_s:.9f}",
                    topic,
                    field,
                    value,
                ])

            self.file.flush()

        return callback

    def destroy_node(self):
        self.file.close()
        super().destroy_node()


def parse_topic_fields(entries):
    """
    Parse:
      /topic:field1,field2,field3
    """
    topic_fields = {}

    for entry in entries:
        if ":" not in entry:
            raise ValueError(
                f"Invalid --topic-field entry: {entry}. "
                f"Expected /topic:field1,field2"
            )

        topic, fields_str = entry.split(":", 1)
        fields = [f.strip() for f in fields_str.split(",") if f.strip()]

        if not topic.startswith("/"):
            raise ValueError(f"Topic must start with '/': {topic}")

        if not fields:
            raise ValueError(f"No fields specified for topic {topic}")

        topic_fields[topic] = fields

    return topic_fields


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--out",
        required=True,
        help="Output CSV path.",
    )
    parser.add_argument(
        "--topic-field",
        action="append",
        required=True,
        help=(
            "Topic and fields to log. "
            "Example: /cartesian_cmd/twist:twist.linear.x,twist.linear.y,twist.linear.z "
            "Can be passed multiple times."
        ),
    )

    args = parser.parse_args()

    topic_fields = parse_topic_fields(args.topic_field)
    out_path = Path(args.out)

    rclpy.init()
    node = FieldLogger(topic_fields, out_path)

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.get_logger().info(f"Saved CSV to {out_path}")
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()