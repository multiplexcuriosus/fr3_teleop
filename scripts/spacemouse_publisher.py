#!/usr/bin/env python3
"""
Configurable ROS2 SpaceMouse publisher.

Publishes geometry_msgs/Twist on a configurable topic.

This is the robot-facing version of the SpaceMouse trainer customizability:
  - deadzone
  - translation / rotation multipliers
  - axis inversion
  - translation-axis blocking
  - rotation-axis blocking
  - translation_only / yaw_only convenience modes
  - raw or trainer-style rotation mapping

Important:
  This node publishes instantaneous command values only. It does not integrate
  pose/orientation. Local-vs-world frame behavior belongs in the downstream
  teleop controller, not in this publisher.
"""

import math
import os
import glob
import select
import threading
from typing import Dict, Any

import pyspacemouse

import rclpy
from rclpy.node import Node
from rcl_interfaces.msg import SetParametersResult
from geometry_msgs.msg import Twist
from std_msgs.msg import UInt8


def deadzone(value: float, dz: float) -> float:
    return 0.0 if abs(value) < dz else value


class HidrawButtonReader:
    """
    Low-level reader for SpaceMouse Compact button reports.

    From observed reports:
      03 00 00 -> no button
      03 01 00 -> button 0 pressed
      03 02 00 -> button 1 pressed
      03 03 00 -> both buttons pressed

    Therefore:
      report[0] == 0x03
      report[1] is the button bitmask
        bit 0 = button 0
        bit 1 = button 1
    """

    def __init__(self, vendor_id="256f", product_id="c635"):
        self.vendor_id = vendor_id.lower()
        self.product_id = product_id.lower()
        self.path = self._find_hidraw()
        self.fd = None
        self.mask = 0
        self.lock = threading.Lock()
        self.running = False
        self.thread = None

    def _find_hidraw(self):
        for path in sorted(glob.glob("/dev/hidraw*")):
            name = os.path.basename(path)
            uevent_path = f"/sys/class/hidraw/{name}/device/uevent"

            try:
                text = open(uevent_path, "r").read().lower()
            except OSError:
                continue

            if self.vendor_id in text and self.product_id in text:
                return path

        return None

    def start(self):
        if self.path is None:
            return False

        self.fd = os.open(self.path, os.O_RDONLY | os.O_NONBLOCK)
        self.running = True
        self.thread = threading.Thread(target=self._loop, daemon=True)
        self.thread.start()
        return True

    def stop(self):
        self.running = False

        if self.thread is not None:
            self.thread.join(timeout=0.5)

        if self.fd is not None:
            os.close(self.fd)
            self.fd = None

    def _loop(self):
        while self.running:
            try:
                readable, _, _ = select.select([self.fd], [], [], 0.05)
            except (OSError, ValueError):
                break

            if not readable:
                continue

            try:
                data = os.read(self.fd, 64)
            except BlockingIOError:
                continue
            except OSError:
                break

            if len(data) >= 2 and data[0] == 0x03:
                with self.lock:
                    self.mask = data[1]

    def get_mask(self):
        with self.lock:
            return self.mask


class SpaceMousePublisher(Node):
    def __init__(self):
        super().__init__("spacemouse_publisher")

        # ROS parameters. These can be set from YAML/launch/CLI.
        self.declare_parameter("topic_name", "spacemouse_cmd")
        self.declare_parameter("publish_hz", 200.0)

        self.declare_parameter("deadzone", 0.04)
        self.declare_parameter("translation_sensitivity", 1.0)
        self.declare_parameter("rotation_sensitivity", 1.0)

        # Rotation mapping:
        #   "raw"     : raw roll -> angular.x, raw pitch -> angular.y
        #   "trainer" : raw pitch -> angular.x, raw roll -> angular.y
        #               matching the trainer observation.
        self.declare_parameter("rotation_mapping", "trainer")

        # Optional sign flips.
        self.declare_parameter("invert_x", False)
        self.declare_parameter("invert_y", False)
        self.declare_parameter("invert_z", False)
        self.declare_parameter("invert_roll", False)
        self.declare_parameter("invert_pitch", False)
        self.declare_parameter("invert_yaw", False)

        # Optional axis blocking.
        self.declare_parameter("block_x", False)
        self.declare_parameter("block_y", False)
        self.declare_parameter("block_z", False)
        self.declare_parameter("block_roll", False)
        self.declare_parameter("block_pitch", False)
        self.declare_parameter("block_yaw", False)

        # Convenience modes.
        self.declare_parameter("translation_only", False)
        self.declare_parameter("yaw_only", False)

        # Debug / behavior.
        self.declare_parameter("publish_zero_on_no_state", False)
        self.declare_parameter("print_debug", False)
        self.declare_parameter("debug_log_period_sec", 0.5)

        # Button reading.
        self.declare_parameter("enable_buttons", True)
        self.declare_parameter("button_topic_name", "/spacemouse_buttons")
        self.declare_parameter("hidraw_vendor_id", "256f")
        self.declare_parameter("hidraw_product_id", "c635")

        self._load_params_from_ros()

        self.publisher_ = self.create_publisher(Twist, self.topic_name, 10)
        
        # Button publisher (conditionally created)
        self.button_publisher_ = None
        self.button_reader_ = None
        if self.enable_buttons:
            self.button_publisher_ = self.create_publisher(UInt8, self.button_topic_name, 10)
            self.button_reader_ = HidrawButtonReader(
                vendor_id=self.hidraw_vendor_id,
                product_id=self.hidraw_product_id
            )
            if self.button_reader_.start():
                self.get_logger().info(
                    f"Button reader started: {self.button_reader_.path}"
                )
            else:
                self.get_logger().warning(
                    "SpaceMouse button device not found; button publishing disabled"
                )
                self.button_reader_ = None
                self.button_publisher_ = None

        self.device = pyspacemouse.open()
        if not self.device:
            self.get_logger().error("Failed to open SpaceMouse device.")
            raise RuntimeError("Failed to open SpaceMouse device")

        self.last_debug_log_time = self.get_clock().now()

        self.param_callback_handle = self.add_on_set_parameters_callback(
            self._on_set_parameters
        )

        self.timer = self.create_timer(1.0 / self.publish_hz, self.timer_callback)

        self.get_logger().info(
            "SpaceMouse publisher started: "
            f"topic={self.topic_name!r}, hz={self.publish_hz:.1f}, "
            f"deadzone={self.deadzone:.3f}, "
            f"trans_sens={self.translation_sensitivity:.3f}, "
            f"rot_sens={self.rotation_sensitivity:.3f}, "
            f"rotation_mapping={self.rotation_mapping!r}, "
            f"translation_only={self.translation_only}, yaw_only={self.yaw_only}, "
            f"buttons_enabled={self.enable_buttons}, button_topic={self.button_topic_name!r}"
        )

    def _load_params_from_ros(self) -> None:
        self.topic_name = self.get_parameter("topic_name").value
        self.publish_hz = float(self.get_parameter("publish_hz").value)

        self.deadzone = float(self.get_parameter("deadzone").value)
        self.translation_sensitivity = float(
            self.get_parameter("translation_sensitivity").value
        )
        self.rotation_sensitivity = float(
            self.get_parameter("rotation_sensitivity").value
        )
        self.rotation_mapping = str(self.get_parameter("rotation_mapping").value)

        self.invert_x = bool(self.get_parameter("invert_x").value)
        self.invert_y = bool(self.get_parameter("invert_y").value)
        self.invert_z = bool(self.get_parameter("invert_z").value)
        self.invert_roll = bool(self.get_parameter("invert_roll").value)
        self.invert_pitch = bool(self.get_parameter("invert_pitch").value)
        self.invert_yaw = bool(self.get_parameter("invert_yaw").value)

        self.block_x = bool(self.get_parameter("block_x").value)
        self.block_y = bool(self.get_parameter("block_y").value)
        self.block_z = bool(self.get_parameter("block_z").value)
        self.block_roll = bool(self.get_parameter("block_roll").value)
        self.block_pitch = bool(self.get_parameter("block_pitch").value)
        self.block_yaw = bool(self.get_parameter("block_yaw").value)

        self.translation_only = bool(self.get_parameter("translation_only").value)
        self.yaw_only = bool(self.get_parameter("yaw_only").value)

        self.publish_zero_on_no_state = bool(
            self.get_parameter("publish_zero_on_no_state").value
        )
        self.print_debug = bool(self.get_parameter("print_debug").value)
        self.debug_log_period_sec = float(
            self.get_parameter("debug_log_period_sec").value
        )

        self.enable_buttons = bool(self.get_parameter("enable_buttons").value)
        self.button_topic_name = str(self.get_parameter("button_topic_name").value)
        self.hidraw_vendor_id = str(self.get_parameter("hidraw_vendor_id").value)
        self.hidraw_product_id = str(self.get_parameter("hidraw_product_id").value)

        self._validate_current_params()

    def _validate_current_params(self) -> None:
        if not isinstance(self.topic_name, str) or not self.topic_name:
            raise ValueError("topic_name must be a non-empty string")

        if not math.isfinite(self.publish_hz) or self.publish_hz <= 0.0:
            raise ValueError("publish_hz must be positive and finite")

        if not math.isfinite(self.deadzone) or self.deadzone < 0.0:
            raise ValueError("deadzone must be finite and non-negative")

        if not math.isfinite(self.translation_sensitivity):
            raise ValueError("translation_sensitivity must be finite")

        if not math.isfinite(self.rotation_sensitivity):
            raise ValueError("rotation_sensitivity must be finite")

        if self.rotation_mapping not in ("raw", "trainer"):
            raise ValueError("rotation_mapping must be 'raw' or 'trainer'")

        if not math.isfinite(self.debug_log_period_sec) or self.debug_log_period_sec < 0.0:
            raise ValueError("debug_log_period_sec must be finite and non-negative")

        if not isinstance(self.button_topic_name, str) or not self.button_topic_name:
            raise ValueError("button_topic_name must be a non-empty string")

    def _on_set_parameters(self, params):
        """
        Live-update tuning params.

        Note:
          topic_name and publish_hz are intentionally not live-updated because
          changing them correctly would require recreating the publisher/timer.
          Set those from YAML/CLI before startup.
        """
        old: Dict[str, Any] = {
            "deadzone": self.deadzone,
            "translation_sensitivity": self.translation_sensitivity,
            "rotation_sensitivity": self.rotation_sensitivity,
            "rotation_mapping": self.rotation_mapping,
            "invert_x": self.invert_x,
            "invert_y": self.invert_y,
            "invert_z": self.invert_z,
            "invert_roll": self.invert_roll,
            "invert_pitch": self.invert_pitch,
            "invert_yaw": self.invert_yaw,
            "block_x": self.block_x,
            "block_y": self.block_y,
            "block_z": self.block_z,
            "block_roll": self.block_roll,
            "block_pitch": self.block_pitch,
            "block_yaw": self.block_yaw,
            "translation_only": self.translation_only,
            "yaw_only": self.yaw_only,
            "publish_zero_on_no_state": self.publish_zero_on_no_state,
            "print_debug": self.print_debug,
            "debug_log_period_sec": self.debug_log_period_sec,
        }

        try:
            for param in params:
                name = param.name

                if name == "topic_name":
                    raise ValueError("topic_name is not live-updatable; restart node")
                if name == "publish_hz":
                    raise ValueError("publish_hz is not live-updatable; restart node")

                if name in old:
                    setattr(self, name, param.value)

            # Normalize types after rclpy parameter assignment.
            self.deadzone = float(self.deadzone)
            self.translation_sensitivity = float(self.translation_sensitivity)
            self.rotation_sensitivity = float(self.rotation_sensitivity)
            self.rotation_mapping = str(self.rotation_mapping)

            self.invert_x = bool(self.invert_x)
            self.invert_y = bool(self.invert_y)
            self.invert_z = bool(self.invert_z)
            self.invert_roll = bool(self.invert_roll)
            self.invert_pitch = bool(self.invert_pitch)
            self.invert_yaw = bool(self.invert_yaw)

            self.block_x = bool(self.block_x)
            self.block_y = bool(self.block_y)
            self.block_z = bool(self.block_z)
            self.block_roll = bool(self.block_roll)
            self.block_pitch = bool(self.block_pitch)
            self.block_yaw = bool(self.block_yaw)

            self.translation_only = bool(self.translation_only)
            self.yaw_only = bool(self.yaw_only)

            self.publish_zero_on_no_state = bool(self.publish_zero_on_no_state)
            self.print_debug = bool(self.print_debug)
            self.debug_log_period_sec = float(self.debug_log_period_sec)

            self._validate_current_params()

        except Exception as exc:
            # Roll back to previous good values.
            for name, value in old.items():
                setattr(self, name, value)

            return SetParametersResult(successful=False, reason=str(exc))

        self.get_logger().info(
            "Updated SpaceMouse params: "
            f"deadzone={self.deadzone:.3f}, "
            f"trans_sens={self.translation_sensitivity:.3f}, "
            f"rot_sens={self.rotation_sensitivity:.3f}, "
            f"rotation_mapping={self.rotation_mapping!r}, "
            f"blocks xyz=({self.block_x},{self.block_y},{self.block_z}), "
            f"blocks rpy=({self.block_roll},{self.block_pitch},{self.block_yaw}), "
            f"translation_only={self.translation_only}, yaw_only={self.yaw_only}"
        )
        return SetParametersResult(successful=True)

    def _map_translation(self, state):#incl. remapping to fr3 tcp
        x = deadzone(float(state.y), self.deadzone)
        y = deadzone(float(state.x), self.deadzone)
        z = deadzone(float(state.z), self.deadzone)

        if self.invert_x:
            x *= -1.0
        if self.invert_y:
            y *= -1.0
        if self.invert_z:
            z *= -1.0

        if self.block_x:
            x = 0.0
        if self.block_y:
            y = 0.0
        if self.block_z:
            z = 0.0

        return (
            x * self.translation_sensitivity,
            y * self.translation_sensitivity,
            z * self.translation_sensitivity,
        )

    def _map_rotation(self, state):
        roll_raw = deadzone(float(state.roll), self.deadzone)
        pitch_raw = deadzone(float(state.pitch), self.deadzone)
        yaw_raw = deadzone(float(state.yaw), self.deadzone)

        if self.rotation_mapping == "trainer":
            # Trainer observation:
            #   raw pitch -> semantic roll
            #   raw roll  -> semantic pitch
            #   raw yaw   -> semantic yaw
            roll = pitch_raw
            pitch = roll_raw
            yaw = yaw_raw
        else:
            # Backward-compatible raw mapping.
            roll = roll_raw
            pitch = pitch_raw
            yaw = yaw_raw

        if self.invert_roll:
            roll *= -1.0
        if self.invert_pitch:
            pitch *= -1.0
        if self.invert_yaw:
            yaw *= -1.0

        if self.translation_only:
            roll = 0.0
            pitch = 0.0
            yaw = 0.0

        if self.yaw_only:
            roll = 0.0
            pitch = 0.0

        if self.block_roll:
            roll = 0.0
        if self.block_pitch:
            pitch = 0.0
        if self.block_yaw:
            yaw = 0.0

        return (
            roll * self.rotation_sensitivity,
            pitch * self.rotation_sensitivity,
            yaw * self.rotation_sensitivity,
        )

    def _make_zero_twist(self):
        return Twist()

    def timer_callback(self):
        state = pyspacemouse.read()
        if state is None:
            if self.publish_zero_on_no_state:
                self.publisher_.publish(self._make_zero_twist())
            # Still publish current button state even if no motion
            if self.button_publisher_ is not None and self.button_reader_ is not None:
                button_msg = UInt8()
                button_msg.data = self.button_reader_.get_mask()
                self.button_publisher_.publish(button_msg)
            return

        x, y, z = self._map_translation(state)
        roll, pitch, yaw = self._map_rotation(state)

        msg = Twist()
        msg.linear.x = float(x)
        msg.linear.y = float(y)
        msg.linear.z = float(z)
        msg.angular.x = float(roll)
        msg.angular.y = float(pitch)
        msg.angular.z = float(yaw)

        self.publisher_.publish(msg)

        # Publish button state
        if self.button_publisher_ is not None and self.button_reader_ is not None:
            button_msg = UInt8()
            button_msg.data = self.button_reader_.get_mask()
            self.button_publisher_.publish(button_msg)

        if self.print_debug:
            now = self.get_clock().now()
            elapsed = (now - self.last_debug_log_time).nanoseconds * 1e-9
            if elapsed >= self.debug_log_period_sec:
                self.last_debug_log_time = now
                self.get_logger().info(
                    "cmd "
                    f"x={msg.linear.x:+.3f} "
                    f"y={msg.linear.y:+.3f} "
                    f"z={msg.linear.z:+.3f} "
                    f"roll={msg.angular.x:+.3f} "
                    f"pitch={msg.angular.y:+.3f} "
                    f"yaw={msg.angular.z:+.3f}"
                )


def main(args=None):
    rclpy.init(args=args)
    node = None

    try:
        node = SpaceMousePublisher()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as exc:
        if node is not None:
            node.get_logger().error(f"SpaceMouse node failed: {exc}")
        else:
            print(f"SpaceMouse node failed: {exc}")
    finally:
        if node is not None:
            # Clean shutdown of button reader
            if node.button_reader_ is not None:
                node.button_reader_.stop()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
