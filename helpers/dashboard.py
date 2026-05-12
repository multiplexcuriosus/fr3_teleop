#!/usr/bin/env python3

import sys
import time
import threading
from dataclasses import dataclass
from typing import Optional

import cv2
import numpy as np

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from std_msgs.msg import UInt8, UInt32
from std_srvs.srv import Trigger

from cv_bridge import CvBridge
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from PySide6.QtCore import Qt, QTimer, Slot
from PySide6.QtGui import QImage, QPixmap
from PySide6.QtWidgets import (
    QApplication,
    QFrame,
    QHBoxLayout,
    QLabel,
    QMainWindow,
    QPushButton,
    QSizePolicy,
    QVBoxLayout,
    QWidget,
)

rgb_qos = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    history=HistoryPolicy.KEEP_LAST,
    depth=5,
    durability=DurabilityPolicy.VOLATILE,
)

event_qos = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=5,
    durability=DurabilityPolicy.VOLATILE,
)


# ----------------------------
# Shared latest-frame buffer
# ----------------------------

class LatestFrameBuffer:
    def __init__(self):
        self._lock = threading.Lock()
        self._frame: Optional[np.ndarray] = None
        self._is_rgb: bool = False
        self._stamp_sec: float = 0.0
        self._seq: int = 0

    def set(self, frame: np.ndarray, is_rgb: bool):
        now = time.monotonic()
        with self._lock:
            self._frame = frame
            self._is_rgb = is_rgb
            self._stamp_sec = now
            self._seq += 1

    def get(self):
        with self._lock:
            if self._frame is None:
                return None, False, 0.0, 0
            return self._frame.copy(), self._is_rgb, self._stamp_sec, self._seq


@dataclass
class TeleopState:
    teleop_enabled: bool = False
    recording_active: bool = False
    session_recording_active: bool = False
    event_frame_publishing_active: bool = False
    last_service_status: str = ""
    successful_episodes: int = 0
    episode_active: bool = False
    episode_start_monotonic: Optional[float] = None
    episode_elapsed_frozen: float = 0.0


# ----------------------------
# Image helpers
# ----------------------------

def pad_to_square_black(img: np.ndarray) -> np.ndarray:
    h, w = img.shape[:2]
    if h == w:
        return img

    size = max(h, w)
    if img.ndim == 2:
        out = np.zeros((size, size), dtype=img.dtype)
    else:
        out = np.zeros((size, size, img.shape[2]), dtype=img.dtype)

    y0 = (size - h) // 2
    x0 = (size - w) // 2
    out[y0:y0 + h, x0:x0 + w] = img
    return out


def np_to_qimage(img: np.ndarray, already_rgb: bool) -> QImage:
    if img.ndim == 2:
        h, w = img.shape
        bytes_per_line = w
        return QImage(
            img.data,
            w,
            h,
            bytes_per_line,
            QImage.Format_Grayscale8,
        ).copy()

    if img.ndim == 3 and img.shape[2] == 3:
        rgb = img if already_rgb else cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        h, w, ch = rgb.shape
        bytes_per_line = ch * w
        return QImage(
            rgb.data,
            w,
            h,
            bytes_per_line,
            QImage.Format_RGB888,
        ).copy()

    raise ValueError(f"Unsupported image shape: {img.shape}")


# ----------------------------
# GUI widgets
# ----------------------------

class ImageTile(QFrame):
    def __init__(self, title: str):
        super().__init__()
        self.setFrameShape(QFrame.StyledPanel)
        self.setStyleSheet("""
            QFrame {
                background-color: #000000;
                border: 1px solid #444444;
                border-radius: 8px;
            }
            QLabel {
                color: white;
                background-color: transparent;
            }
        """)

        self.title_label = QLabel(title)
        self.title_label.setAlignment(Qt.AlignCenter)
        self.title_label.setFixedHeight(28)
        self.title_label.setStyleSheet("font-size: 16px; font-weight: 600;")

        self.image_label = QLabel("No image")
        self.image_label.setAlignment(Qt.AlignCenter)
        self.image_label.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.image_label.setMinimumSize(200, 200)
        self.image_label.setStyleSheet("font-size: 18px;")

        layout = QVBoxLayout()
        layout.setContentsMargins(8, 8, 8, 8)
        layout.setSpacing(8)
        layout.addWidget(self.title_label)
        layout.addWidget(self.image_label, 1)
        self.setLayout(layout)

        self._last_qimage: Optional[QImage] = None

    def set_qimage(self, qimg: QImage):
        self._last_qimage = qimg
        self._refresh_pixmap()

    def resizeEvent(self, event):
        super().resizeEvent(event)
        self._refresh_pixmap()

    def _refresh_pixmap(self):
        if self._last_qimage is None:
            return

        pix = QPixmap.fromImage(self._last_qimage)
        scaled = pix.scaled(
            self.image_label.size(),
            Qt.KeepAspectRatio,
            Qt.SmoothTransformation,
        )
        self.image_label.setPixmap(scaled)


class MetricCard(QFrame):
    def __init__(self, title: str, initial_value: str):
        super().__init__()
        self.setFrameShape(QFrame.StyledPanel)
        self.setStyleSheet("""
            QFrame {
                background-color: #111111;
                border: 1px solid #444444;
                border-radius: 10px;
            }
            QLabel {
                color: white;
                background-color: transparent;
            }
        """)

        self.title_label = QLabel(title)
        self.title_label.setAlignment(Qt.AlignCenter)
        self.title_label.setStyleSheet("font-size: 20px; font-weight: 600;")

        self.value_label = QLabel(initial_value)
        self.value_label.setAlignment(Qt.AlignCenter)
        self.value_label.setStyleSheet("font-size: 44px; font-weight: 700;")

        layout = QVBoxLayout()
        layout.setContentsMargins(16, 16, 16, 16)
        layout.setSpacing(10)
        layout.addWidget(self.title_label)
        layout.addWidget(self.value_label, 1)
        self.setLayout(layout)


class EpisodeCard(QFrame):
    def __init__(self):
        super().__init__()
        self.setFrameShape(QFrame.StyledPanel)
        self.setStyleSheet("""
            QFrame {
                background-color: #111111;
                border: 1px solid #444444;
                border-radius: 10px;
            }
            QLabel {
                color: white;
                background-color: transparent;
            }
        """)

        self.title_label = QLabel("Episode")
        self.title_label.setAlignment(Qt.AlignCenter)
        self.title_label.setStyleSheet("font-size: 20px; font-weight: 600;")

        self.indicator = QLabel("IDLE")
        self.indicator.setAlignment(Qt.AlignCenter)
        self.indicator.setMinimumHeight(110)
        self.indicator.setStyleSheet("""
            QLabel {
                background-color: #7a0000;
                color: white;
                font-size: 42px;
                font-weight: 800;
                border-radius: 12px;
                padding: 12px;
            }
        """)

        layout = QVBoxLayout()
        layout.setContentsMargins(16, 16, 16, 16)
        layout.setSpacing(10)
        layout.addWidget(self.title_label)
        layout.addWidget(self.indicator, 1)
        self.setLayout(layout)

    def set_episode_active(self, active: bool):
        if active:
            self.indicator.setText("ACTIVE")
            self.indicator.setStyleSheet("""
                QLabel {
                    background-color: #008c3a;
                    color: white;
                    font-size: 42px;
                    font-weight: 800;
                    border-radius: 12px;
                    padding: 12px;
                }
            """)
        else:
            self.indicator.setText("IDLE")
            self.indicator.setStyleSheet("""
                QLabel {
                    background-color: #7a0000;
                    color: white;
                    font-size: 42px;
                    font-weight: 800;
                    border-radius: 12px;
                    padding: 12px;
                }
            """)

    def set_recording(self, active: bool):
        self.set_episode_active(active)


class SessionRecordingControlCard(QFrame):
    def __init__(self, on_start, on_stop):
        super().__init__()
        self._on_start = on_start
        self._on_stop = on_stop

        self.setFrameShape(QFrame.StyledPanel)
        self.setStyleSheet("""
            QFrame {
                background-color: #111111;
                border: 1px solid #444444;
                border-radius: 10px;
            }
            QLabel {
                color: white;
                background-color: transparent;
            }
            QPushButton {
                color: white;
                border: 1px solid #555555;
                border-radius: 8px;
                padding: 10px 14px;
                font-size: 16px;
                font-weight: 700;
                min-height: 22px;
            }
            QPushButton:disabled {
                background-color: #2d2d2d;
                color: #888888;
                border-color: #3a3a3a;
            }
        """)

        self.title_label = QLabel("Bag + Raw Events")
        self.title_label.setAlignment(Qt.AlignCenter)
        self.title_label.setStyleSheet("font-size: 18px; font-weight: 600;")

        self.indicator = QLabel("OFF")
        self.indicator.setAlignment(Qt.AlignCenter)
        self.indicator.setMinimumHeight(86)
        self.indicator.setStyleSheet("""
            QLabel {
                background-color: #7a0000;
                color: white;
                font-size: 30px;
                font-weight: 800;
                border-radius: 12px;
                padding: 10px;
            }
        """)

        self.start_button = QPushButton("Start Bag/H5")
        self.stop_button = QPushButton("Stop Bag/H5")
        self.start_button.setStyleSheet("background-color: #0051a8;")
        self.stop_button.setStyleSheet("background-color: #8c1f1f;")
        self.start_button.clicked.connect(self._on_start)
        self.stop_button.clicked.connect(self._on_stop)

        button_row = QHBoxLayout()
        button_row.setSpacing(8)
        button_row.addWidget(self.start_button)
        button_row.addWidget(self.stop_button)

        layout = QVBoxLayout()
        layout.setContentsMargins(16, 16, 16, 16)
        layout.setSpacing(10)
        layout.addWidget(self.title_label)
        layout.addWidget(self.indicator, 1)
        layout.addLayout(button_row)
        self.setLayout(layout)

        self.set_active(False)

    def set_active(self, active: bool):
        if active:
            self.indicator.setText("REC")
            self.indicator.setStyleSheet("""
                QLabel {
                    background-color: #008c3a;
                    color: white;
                    font-size: 30px;
                    font-weight: 800;
                    border-radius: 12px;
                    padding: 10px;
                }
            """)
        else:
            self.indicator.setText("OFF")
            self.indicator.setStyleSheet("""
                QLabel {
                    background-color: #7a0000;
                    color: white;
                    font-size: 30px;
                    font-weight: 800;
                    border-radius: 12px;
                    padding: 10px;
                }
            """)

        self.start_button.setEnabled(not active)
        self.stop_button.setEnabled(active)


class EventFramePublishingControlCard(QFrame):
    def __init__(self, on_start, on_stop):
        super().__init__()
        self._on_start = on_start
        self._on_stop = on_stop

        self.setFrameShape(QFrame.StyledPanel)
        self.setStyleSheet("""
            QFrame {
                background-color: #111111;
                border: 1px solid #444444;
                border-radius: 10px;
            }
            QLabel {
                color: white;
                background-color: transparent;
            }
            QPushButton {
                color: white;
                border: 1px solid #555555;
                border-radius: 8px;
                padding: 10px 14px;
                font-size: 16px;
                font-weight: 700;
                min-height: 22px;
            }
            QPushButton:disabled {
                background-color: #2d2d2d;
                color: #888888;
                border-color: #3a3a3a;
            }
        """)

        self.title_label = QLabel("Event Frames")
        self.title_label.setAlignment(Qt.AlignCenter)
        self.title_label.setStyleSheet("font-size: 18px; font-weight: 600;")

        self.indicator = QLabel("OFF")
        self.indicator.setAlignment(Qt.AlignCenter)
        self.indicator.setMinimumHeight(86)
        self.indicator.setStyleSheet("""
            QLabel {
                background-color: #7a0000;
                color: white;
                font-size: 30px;
                font-weight: 800;
                border-radius: 12px;
                padding: 10px;
            }
        """)

        self.start_button = QPushButton("Start Frames")
        self.stop_button = QPushButton("Stop Frames")
        self.start_button.setStyleSheet("background-color: #0b6b6b;")
        self.stop_button.setStyleSheet("background-color: #8c1f1f;")
        self.start_button.clicked.connect(self._on_start)
        self.stop_button.clicked.connect(self._on_stop)

        button_row = QHBoxLayout()
        button_row.setSpacing(8)
        button_row.addWidget(self.start_button)
        button_row.addWidget(self.stop_button)

        layout = QVBoxLayout()
        layout.setContentsMargins(16, 16, 16, 16)
        layout.setSpacing(10)
        layout.addWidget(self.title_label)
        layout.addWidget(self.indicator, 1)
        layout.addLayout(button_row)
        self.setLayout(layout)

        self.set_active(False)

    def set_active(self, active: bool):
        if active:
            self.indicator.setText("ON")
            self.indicator.setStyleSheet("""
                QLabel {
                    background-color: #0f7f8c;
                    color: white;
                    font-size: 30px;
                    font-weight: 800;
                    border-radius: 12px;
                    padding: 10px;
                }
            """)
        else:
            self.indicator.setText("OFF")
            self.indicator.setStyleSheet("""
                QLabel {
                    background-color: #7a0000;
                    color: white;
                    font-size: 30px;
                    font-weight: 800;
                    border-radius: 12px;
                    padding: 10px;
                }
            """)

        self.start_button.setEnabled(not active)
        self.stop_button.setEnabled(active)


class TeleopCard(QFrame):
    def __init__(self):
        super().__init__()
        self.setFrameShape(QFrame.StyledPanel)
        self.setStyleSheet("""
            QFrame {
                background-color: #111111;
                border: 1px solid #444444;
                border-radius: 10px;
            }
            QLabel {
                color: white;
                background-color: transparent;
            }
        """)

        self.title_label = QLabel("Teleop")
        self.title_label.setAlignment(Qt.AlignCenter)
        self.title_label.setStyleSheet("font-size: 20px; font-weight: 600;")

        self.indicator = QLabel("DISABLED")
        self.indicator.setAlignment(Qt.AlignCenter)
        self.indicator.setMinimumHeight(110)
        self.indicator.setStyleSheet("""
            QLabel {
                background-color: #7a0000;
                color: white;
                font-size: 34px;
                font-weight: 800;
                border-radius: 12px;
                padding: 12px;
            }
        """)

        layout = QVBoxLayout()
        layout.setContentsMargins(16, 16, 16, 16)
        layout.setSpacing(10)
        layout.addWidget(self.title_label)
        layout.addWidget(self.indicator, 1)
        self.setLayout(layout)

    def set_teleop(self, enabled: bool):
        if enabled:
            self.indicator.setText("ENABLED")
            self.indicator.setStyleSheet("""
                QLabel {
                    background-color: #0051a8;
                    color: white;
                    font-size: 34px;
                    font-weight: 800;
                    border-radius: 12px;
                    padding: 12px;
                }
            """)
        else:
            self.indicator.setText("DISABLED")
            self.indicator.setStyleSheet("""
                QLabel {
                    background-color: #7a0000;
                    color: white;
                    font-size: 34px;
                    font-weight: 800;
                    border-radius: 12px;
                    padding: 12px;
                }
            """)


# ----------------------------
# ROS node
# ----------------------------

class TeleopDashboardNode(Node):
    def __init__(self):
        super().__init__("teleop_dashboard")
        self.bridge = CvBridge()

        self.episode_start_cmd = 1
        self.episode_stop_cmd = 2
        self.episode_cancel_current_cmd = 3
        self.episode_cancel_last_cmd = 4
        self.teleop_start_cmd = 1
        self.teleop_stop_cmd = 2

        self.declare_parameter("rgb_topic", "/camera/camera/color/image_raw")
        self.declare_parameter("event_topic", "/openmv_cam/image")
        self.declare_parameter("episode_control_topic", "/episode/control")
        self.declare_parameter("teleop_control_topic", "/teleop/control")
        self.declare_parameter("num_valid_episodes_topic", "/data_collection/num_valid_episodes")
        self.declare_parameter("start_recording_service", "/record_manager/start_recording")
        self.declare_parameter("stop_recording_service", "/record_manager/stop_recording")
        self.declare_parameter("start_event_frames_service", "/openmv_cam/start_event_frame_publishing")
        self.declare_parameter("stop_event_frames_service", "/openmv_cam/stop_event_frame_publishing")
        self.declare_parameter("service_timeout_sec", 3.0)

        self.rgb_topic = self.get_parameter("rgb_topic").value
        self.event_topic = self.get_parameter("event_topic").value
        self.episode_control_topic = self.get_parameter("episode_control_topic").value
        self.teleop_control_topic = self.get_parameter("teleop_control_topic").value
        self.num_valid_episodes_topic = self.get_parameter("num_valid_episodes_topic").value
        self.start_recording_service = self.get_parameter("start_recording_service").value
        self.stop_recording_service = self.get_parameter("stop_recording_service").value
        self.start_event_frames_service = self.get_parameter("start_event_frames_service").value
        self.stop_event_frames_service = self.get_parameter("stop_event_frames_service").value

        self.rgb_buffer = LatestFrameBuffer()
        self.event_buffer = LatestFrameBuffer()

        self.state_lock = threading.Lock()
        self.state = TeleopState()

        self.rgb_sub = self.create_subscription(
            Image,
            self.rgb_topic,
            self.rgb_cb,
            rgb_qos,
        )
        self.event_sub = self.create_subscription(
            Image,
            self.event_topic,
            self.event_cb,
            event_qos,
        )
        self.episode_control_sub = self.create_subscription(
            UInt8,
            self.episode_control_topic,
            self.episode_control_cb,
            10,
        )
        self.teleop_control_sub = self.create_subscription(
            UInt8,
            self.teleop_control_topic,
            self.teleop_control_cb,
            10,
        )
        self.num_valid_episodes_sub = self.create_subscription(
            UInt32,
            self.num_valid_episodes_topic,
            self.num_valid_episodes_cb,
            10,
        )

        self.start_recording_client = self.create_client(Trigger, self.start_recording_service)
        self.stop_recording_client = self.create_client(Trigger, self.stop_recording_service)
        self.start_event_frames_client = self.create_client(Trigger, self.start_event_frames_service)
        self.stop_event_frames_client = self.create_client(Trigger, self.stop_event_frames_service)

        self.get_logger().info(f"RGB topic: {self.rgb_topic}")
        self.get_logger().info(f"Event topic: {self.event_topic}")
        self.get_logger().info(f"Episode control topic: {self.episode_control_topic}")
        self.get_logger().info(f"Teleop control topic: {self.teleop_control_topic}")
        self.get_logger().info(f"Num valid episodes topic: {self.num_valid_episodes_topic}")
        self.get_logger().info(f"Start recording service: {self.start_recording_service}")
        self.get_logger().info(f"Stop recording service: {self.stop_recording_service}")
        self.get_logger().info(f"Start event frames service: {self.start_event_frames_service}")
        self.get_logger().info(f"Stop event frames service: {self.stop_event_frames_service}")

    def _set_last_service_status(self, status: str):
        with self.state_lock:
            self.state.last_service_status = status

    def _get_double_param(self, name):
        return self.get_parameter(name).get_parameter_value().double_value

    def _wait_for_service(self, client, timeout_sec):
        start = time.monotonic()
        while rclpy.ok() and not client.wait_for_service(timeout_sec=0.2):
            if time.monotonic() - start > timeout_sec:
                return False
        return True

    def call_trigger_service_async(self, client, service_name, on_success, on_failure):
        timeout_sec = self._get_double_param("service_timeout_sec")

        def worker():
            self.get_logger().info(f"Requesting service call: {service_name}")
            self._set_last_service_status(f"Requesting {service_name}")

            try:
                if not self._wait_for_service(client, timeout_sec):
                    reason = f"Service not available: {service_name}"
                    self.get_logger().error(reason)
                    status = on_failure(reason)
                    if status:
                        self._set_last_service_status(status)
                    return

                future = client.call_async(Trigger.Request())
                start = time.monotonic()

                while rclpy.ok() and not future.done():
                    if time.monotonic() - start > timeout_sec:
                        reason = f"Service call timed out: {service_name}"
                        self.get_logger().error(reason)
                        status = on_failure(reason)
                        if status:
                            self._set_last_service_status(status)
                        return
                    time.sleep(0.02)

                result = future.result()
                if result is None:
                    reason = f"Service call failed: {service_name}"
                    self.get_logger().error(reason)
                    status = on_failure(reason)
                    if status:
                        self._set_last_service_status(status)
                    return

                if not result.success:
                    reason = f"Service returned failure: {service_name}"
                    self.get_logger().error(reason)
                    status = on_failure(reason)
                    if status:
                        self._set_last_service_status(status)
                    return

                status = on_success()
                self.get_logger().info(f"Service call succeeded: {service_name}")
                if status:
                    self._set_last_service_status(status)

            except Exception as e:
                reason = f"Service call error for {service_name}: {e}"
                self.get_logger().error(reason)
                status = on_failure(reason)
                if status:
                    self._set_last_service_status(status)

        threading.Thread(target=worker, daemon=True).start()

    def rgb_cb(self, msg: Image):
        try:
            img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
            img = cv2.rotate(img, cv2.ROTATE_180)
            self.rgb_buffer.set(img, is_rgb=False)
        except Exception as e:
            self.get_logger().error(f"RGB callback failed: {e}")

    def event_cb(self, msg: Image):
        try:
            img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")

            if img.ndim == 2:
                disp = pad_to_square_black(img)
                is_rgb = False
            elif img.ndim == 3 and img.shape[2] == 3:
                disp = pad_to_square_black(img)
                is_rgb = False
            elif img.ndim == 3 and img.shape[2] == 4:
                disp = cv2.cvtColor(img, cv2.COLOR_BGRA2BGR)
                disp = pad_to_square_black(disp)
                is_rgb = False
            else:
                self.get_logger().warning(
                    f"Unsupported event image format: encoding={msg.encoding}, shape={img.shape}"
                )
                return

            self.event_buffer.set(disp, is_rgb=is_rgb)

        except Exception as e:
            self.get_logger().error(f"Event callback failed: {e}")

    def episode_control_cb(self, msg: UInt8):
        now = time.monotonic()
        with self.state_lock:
            cmd = int(msg.data)

            if cmd == self.episode_start_cmd:
                self.state.episode_active = True
                self.state.recording_active = True
                self.state.episode_start_monotonic = now
                self.state.episode_elapsed_frozen = 0.0

            elif cmd == self.episode_stop_cmd:
                if self.state.episode_start_monotonic is not None:
                    self.state.episode_elapsed_frozen = now - self.state.episode_start_monotonic
                self.state.episode_active = False
                self.state.recording_active = False
                self.state.episode_start_monotonic = None

            elif cmd == self.episode_cancel_current_cmd:
                if self.state.episode_start_monotonic is not None:
                    self.state.episode_elapsed_frozen = now - self.state.episode_start_monotonic
                self.state.episode_active = False
                self.state.recording_active = False
                self.state.episode_start_monotonic = None

            elif cmd == self.episode_cancel_last_cmd:
                # cancel_last does not affect live recording state or duration display.
                pass

    def num_valid_episodes_cb(self, msg: UInt32):
        with self.state_lock:
            self.state.successful_episodes = int(msg.data)

    def teleop_control_cb(self, msg: UInt8):
        with self.state_lock:
            cmd = int(msg.data)
            if cmd == self.teleop_start_cmd:
                self.state.teleop_enabled = True
            elif cmd == self.teleop_stop_cmd:
                self.state.teleop_enabled = False

    def get_state_snapshot(self) -> TeleopState:
        with self.state_lock:
            s = self.state
            return TeleopState(
                teleop_enabled=s.teleop_enabled,
                recording_active=s.recording_active,
                session_recording_active=s.session_recording_active,
                event_frame_publishing_active=s.event_frame_publishing_active,
                last_service_status=s.last_service_status,
                successful_episodes=s.successful_episodes,
                episode_active=s.episode_active,
                episode_start_monotonic=s.episode_start_monotonic,
                episode_elapsed_frozen=s.episode_elapsed_frozen,
            )

    def start_session_recording(self):
        self.get_logger().info("Requested start of bag + raw-event session recording")

        def on_success():
            with self.state_lock:
                self.state.session_recording_active = True
                self.state.last_service_status = "Session recording started successfully."
            return "Session recording started successfully."

        def on_failure(reason: str):
            with self.state_lock:
                self.state.last_service_status = f"Session recording start failed: {reason}"
            return self.state.last_service_status

        self.call_trigger_service_async(
            self.start_recording_client,
            self.start_recording_service,
            on_success,
            on_failure,
        )

    def stop_session_recording(self):
        self.get_logger().info("Requested stop of bag + raw-event session recording")

        def on_success():
            with self.state_lock:
                self.state.session_recording_active = False
                self.state.last_service_status = "Session recording stopped successfully."
            return "Session recording stopped successfully."

        def on_failure(reason: str):
            with self.state_lock:
                self.state.last_service_status = f"Session recording stop failed: {reason}"
            return self.state.last_service_status

        self.call_trigger_service_async(
            self.stop_recording_client,
            self.stop_recording_service,
            on_success,
            on_failure,
        )

    def start_event_frame_publishing(self):
        self.get_logger().info("Requested start of OpenMV event-frame publishing")

        def on_success():
            with self.state_lock:
                self.state.event_frame_publishing_active = True
                self.state.last_service_status = "Event-frame publishing started successfully."
            return "Event-frame publishing started successfully."

        def on_failure(reason: str):
            with self.state_lock:
                self.state.last_service_status = f"Event-frame publishing start failed: {reason}"
            return self.state.last_service_status

        self.call_trigger_service_async(
            self.start_event_frames_client,
            self.start_event_frames_service,
            on_success,
            on_failure,
        )

    def stop_event_frame_publishing(self):
        self.get_logger().info("Requested stop of OpenMV event-frame publishing")

        def on_success():
            with self.state_lock:
                self.state.event_frame_publishing_active = False
                self.state.last_service_status = "Event-frame publishing stopped successfully."
            return "Event-frame publishing stopped successfully."

        def on_failure(reason: str):
            with self.state_lock:
                self.state.last_service_status = f"Event-frame publishing stop failed: {reason}"
            return self.state.last_service_status

        self.call_trigger_service_async(
            self.stop_event_frames_client,
            self.stop_event_frames_service,
            on_success,
            on_failure,
        )


# ----------------------------
# Main window
# ----------------------------

class TeleopDashboardWindow(QMainWindow):
    def __init__(self, node: TeleopDashboardNode, start_fullscreen: bool = True):
        super().__init__()
        self.node = node
        self.setWindowTitle("Teleop Dashboard")

        central = QWidget()
        self.setCentralWidget(central)

        root = QVBoxLayout()
        root.setContentsMargins(10, 10, 10, 10)
        root.setSpacing(10)
        central.setLayout(root)

        # Top metrics row
        top_row = QHBoxLayout()
        top_row.setSpacing(10)

        self.teleop_card = TeleopCard()
        self.episode_card = EpisodeCard()
        self.success_card = MetricCard("Successful Episodes", "0")
        self.duration_card = MetricCard("Current Episode [s]", "0.0")

        top_row.addWidget(self.teleop_card, 1)
        top_row.addWidget(self.episode_card, 1)
        top_row.addWidget(self.success_card, 1)
        top_row.addWidget(self.duration_card, 1)

        # Middle image row
        image_row = QHBoxLayout()
        image_row.setSpacing(10)

        self.rgb_tile = ImageTile("RGB")
        self.event_tile = ImageTile("Event")

        image_row.addWidget(self.rgb_tile, 1)
        image_row.addWidget(self.event_tile, 1)

        # Bottom controls
        bottom_row = QHBoxLayout()
        bottom_row.setSpacing(10)

        self.session_recording_card = SessionRecordingControlCard(
            on_start=self.node.start_session_recording,
            on_stop=self.node.stop_session_recording,
        )
        self.event_frames_card = EventFramePublishingControlCard(
            on_start=self.node.start_event_frame_publishing,
            on_stop=self.node.stop_event_frame_publishing,
        )

        self.help_label = QLabel("Double-click: fullscreen toggle | Status: ")
        self.help_label.setAlignment(Qt.AlignCenter)
        self.help_label.setStyleSheet("color: white; font-size: 16px;")

        bottom_row.addWidget(self.session_recording_card, 1)
        bottom_row.addWidget(self.event_frames_card, 1)
        bottom_row.addWidget(self.help_label, 2)

        root.addLayout(top_row, 0)
        root.addLayout(image_row, 1)
        root.addLayout(bottom_row, 0)

        self.setStyleSheet("background-color: #1b1b1b;")
        self.resize(1600, 950)

        self._is_fullscreen = False

        if start_fullscreen:
            self.showFullScreen()
            self._is_fullscreen = True

        self.gui_timer = QTimer(self)
        self.gui_timer.timeout.connect(self.refresh_gui)
        self.gui_timer.start(50)  # 20 Hz GUI refresh

    def mouseDoubleClickEvent(self, event):
        if event.button() == Qt.LeftButton:
            self.toggle_fullscreen()
        super().mouseDoubleClickEvent(event)

    def toggle_fullscreen(self):
        if self._is_fullscreen:
            self.showNormal()
            self.resize(1600, 950)
            self._is_fullscreen = False
        else:
            self.showFullScreen()
            self._is_fullscreen = True

    @Slot()
    def refresh_gui(self):
        # Images
        rgb_frame, rgb_is_rgb, _, _ = self.node.rgb_buffer.get()
        if rgb_frame is not None:
            try:
                qimg = np_to_qimage(rgb_frame, already_rgb=rgb_is_rgb)
                self.rgb_tile.set_qimage(qimg)
            except Exception:
                pass

        event_frame, event_is_rgb, _, _ = self.node.event_buffer.get()
        if event_frame is not None:
            try:
                qimg = np_to_qimage(event_frame, already_rgb=event_is_rgb)
                self.event_tile.set_qimage(qimg)
            except Exception:
                pass

        # State
        s = self.node.get_state_snapshot()
        self.teleop_card.set_teleop(s.teleop_enabled)
        self.episode_card.set_episode_active(s.episode_active)
        self.session_recording_card.set_active(s.session_recording_active)
        self.event_frames_card.set_active(s.event_frame_publishing_active)
        self.success_card.value_label.setText(str(s.successful_episodes))
        self.help_label.setText(f"Double-click: fullscreen toggle | Status: {s.last_service_status}")

        if s.episode_active and s.episode_start_monotonic is not None:
            elapsed = time.monotonic() - s.episode_start_monotonic
        else:
            elapsed = s.episode_elapsed_frozen

        self.duration_card.value_label.setText(f"{elapsed:.1f}")


# ----------------------------
# main
# ----------------------------

def main():
    rclpy.init(args=sys.argv)

    node = TeleopDashboardNode()

    ros_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    ros_thread.start()

    app = QApplication(sys.argv)
    window = TeleopDashboardWindow(node=node, start_fullscreen=True)
    window.show()

    rc = app.exec()

    node.destroy_node()
    rclpy.shutdown()
    ros_thread.join(timeout=1.0)
    sys.exit(rc)


if __name__ == "__main__":
    main()