#!/usr/bin/env python3

import sys
import time
import threading
from pathlib import Path
from dataclasses import dataclass
from typing import Optional

import cv2
import h5py
import numpy as np

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from fr3_husky_msgs.action import LineTrajectory
from fr3_husky_msgs.srv import CaptureLineCenter
from fr3_husky_msgs.srv import SetLineParams
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, UInt8, UInt32, String
from std_srvs.srv import Trigger
from std_srvs.srv import SetBool

from cv_bridge import CvBridge
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy

from PySide6.QtCore import Qt, QTimer, Slot
from PySide6.QtGui import QImage, QPixmap
from PySide6.QtWidgets import (
    QApplication,
    QButtonGroup,
    QFrame,
    QHBoxLayout,
    QLabel,
    QMainWindow,
    QPushButton,
    QSizePolicy,
    QVBoxLayout,
    QWidget,
)

image_qos = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
    durability=DurabilityPolicy.VOLATILE,
)

event_qos = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=5,
    durability=DurabilityPolicy.VOLATILE,
)

# Defaults below are only used when the launch file does not inject the
# corresponding parameter. The canonical values live in
# fr3_teleop.config.interfaces and are injected as ROS parameters by
# teleop.launch.py (DASHBOARD_INTERFACE_PARAMS) so this module does not
# import the interface registry directly.
EVENT_FRAME_MONO_TOPIC = "/openmv_cam/image"
EVENT_FRAME_3CH_TOPIC = "/openmv_cam/event_frame_3ch"
DEFAULT_EVENT_FRAME_VISUALIZATION = "mono"
RGB_CAM_TOPIC = "/top_cam/camera/color/image_raw"

INTERCEPTION_ARM_MODES = {"scene", "rollout", "both"}
INTERCEPTION_EXECUTION_MODES = {"both_dry", "scene_wet", "rollout_wet"}
INTERCEPTION_ACTIVE_STATES = {
    "RESETTING",
    "ARMED_WAITING",
    "PROJECTING",
    "SENDING_ACTION",
    "EXECUTING",
}


def extract_controller_state(status: str) -> str:
    raw = str(status or "").strip()
    if not raw:
        return "UNKNOWN"
    for part in raw.split(";"):
        token = part.strip()
        if token.lower().startswith("state="):
            value = token.split("=", 1)[1].strip()
            return value.upper() if value else "UNKNOWN"
    return "UNKNOWN"


def controller_state_is_active(status: str) -> bool:
    return extract_controller_state(status) in INTERCEPTION_ACTIVE_STATES


def extract_status_field(status: str, field_name: str) -> Optional[str]:
    raw = str(status or "").strip()
    if not raw:
        return None
    needle = str(field_name).strip().lower() + "="
    for part in raw.split(";"):
        token = part.strip()
        if token.lower().startswith(needle):
            return token.split("=", 1)[1].strip()
    return None


def extract_controller_dry_run(status: str) -> Optional[bool]:
    value = extract_status_field(status, "dry_run")
    if value is None:
        return None
    lowered = value.strip().lower()
    if lowered == "true":
        return True
    if lowered == "false":
        return False
    return None


def derive_execution_mode(scene_status: str, rollout_status: str) -> str:
    scene_dry = extract_controller_dry_run(scene_status)
    rollout_dry = extract_controller_dry_run(rollout_status)
    if scene_dry is None or rollout_dry is None:
        return "unknown"
    if scene_dry and rollout_dry:
        return "both_dry"
    if (not scene_dry) and rollout_dry:
        return "scene_wet"
    if scene_dry and (not rollout_dry):
        return "rollout_wet"
    return "both_wet"


def execution_mode_label(mode: str) -> str:
    normalized = str(mode).strip().lower()
    if normalized == "both_dry":
        return "BOTH DRY"
    if normalized == "scene_wet":
        return "SCENE WET"
    if normalized == "rollout_wet":
        return "ROLLOUT WET"
    if normalized == "both_wet":
        return "UNSAFE: BOTH WET"
    return "UNKNOWN"


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

    def clear(self):
        with self._lock:
            self._frame = None
            self._is_rgb = False
            self._stamp_sec = 0.0
            self._seq += 1


@dataclass
class TeleopState:
    teleop_enabled: bool = False
    recording_active: bool = False
    session_recording_active: bool = False
    event_frame_publishing_active: bool = False
    debug_bypass_topic_presence: bool = False
    last_service_status: str = ""
    successful_episodes: int = 0
    episode_active: bool = False
    episode_start_monotonic: Optional[float] = None
    episode_elapsed_frozen: float = 0.0
    reset_episode_counter_pending: bool = False

    interception_arm_mode: str = "scene"
    scene_interception_status: str = "state=UNKNOWN"
    rollout_interception_status: str = "state=UNKNOWN"
    scene_status_receive_monotonic: Optional[float] = None
    rollout_status_receive_monotonic: Optional[float] = None
    execution_mode_transition_pending: bool = False


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

    def clear(self):
        self._last_qimage = None
        self.image_label.clear()
        self.image_label.setText("No image")


class MetricCard(QFrame):
    def __init__(
        self,
        title: str,
        initial_value: str,
        button_text: Optional[str] = None,
        on_button_clicked=None,
    ):
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

        self.action_button: Optional[QPushButton] = None
        self.default_button_text = button_text if button_text is not None else ""
        if button_text is not None and on_button_clicked is not None:
            self.action_button = QPushButton(button_text)
            self.action_button.setStyleSheet("""
                QPushButton {
                    background-color: #6b5500;
                    color: white;
                    border: 1px solid #777777;
                    border-radius: 8px;
                    padding: 8px 12px;
                    font-size: 15px;
                    font-weight: 700;
                }
                QPushButton:disabled {
                    background-color: #2d2d2d;
                    color: #888888;
                }
            """)
            self.action_button.clicked.connect(on_button_clicked)

        layout = QVBoxLayout()
        layout.setContentsMargins(16, 16, 16, 16)
        layout.setSpacing(10)
        layout.addWidget(self.title_label)
        layout.addWidget(self.value_label, 1)
        if self.action_button is not None:
            layout.addWidget(self.action_button)
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
    def __init__(self, on_start, on_stop, on_toggle_debug_bypass):
        super().__init__()
        self._on_start = on_start
        self._on_stop = on_stop
        self._on_toggle_debug_bypass = on_toggle_debug_bypass
        self._debug_bypass_on = False

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
        self.debug_bypass_button = QPushButton("Debug Bypass: OFF")
        self.start_button.setStyleSheet("background-color: #0051a8;")
        self.stop_button.setStyleSheet("background-color: #8c1f1f;")
        self.debug_bypass_button.setStyleSheet("background-color: #4f3a00;")
        self.start_button.clicked.connect(self._on_start)
        self.stop_button.clicked.connect(self._on_stop)
        self.debug_bypass_button.clicked.connect(self._toggle_debug_bypass)

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
        layout.addWidget(self.debug_bypass_button)
        self.setLayout(layout)

        self.set_active(False)

    def _toggle_debug_bypass(self):
        self._on_toggle_debug_bypass(not self._debug_bypass_on)

    def set_debug_bypass(self, enabled: bool):
        self._debug_bypass_on = bool(enabled)
        if self._debug_bypass_on:
            self.debug_bypass_button.setText("Debug Bypass: ON")
            self.debug_bypass_button.setStyleSheet("background-color: #8c5f00;")
        else:
            self.debug_bypass_button.setText("Debug Bypass: OFF")
            self.debug_bypass_button.setStyleSheet("background-color: #4f3a00;")

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
    def __init__(self, on_start, on_stop, on_select_visualization, initial_mode: str = DEFAULT_EVENT_FRAME_VISUALIZATION):
        super().__init__()
        self._on_start = on_start
        self._on_stop = on_stop
        self._on_select_visualization = on_select_visualization
        self._selected_event_frame_visualization = initial_mode

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

        self.publishing_state_label = QLabel("Publishing: OFF")
        self.publishing_state_label.setAlignment(Qt.AlignCenter)
        self.publishing_state_label.setMinimumHeight(42)
        self.publishing_state_label.setStyleSheet("font-size: 20px; font-weight: 800;")

        self.single_channel_button = QPushButton("Single Channel")
        self.three_channel_button = QPushButton("3-Channel")
        self.single_channel_button.clicked.connect(lambda: self._on_select_visualization("mono"))
        self.three_channel_button.clicked.connect(lambda: self._on_select_visualization("3ch"))

        mode_row = QHBoxLayout()
        mode_row.setSpacing(8)
        mode_row.addWidget(self.single_channel_button)
        mode_row.addWidget(self.three_channel_button)

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
        layout.addWidget(self.publishing_state_label)
        layout.addLayout(mode_row)
        layout.addLayout(button_row)
        self.setLayout(layout)

        self.set_visualization_mode(initial_mode)
        self.set_active(False)

    def set_active(self, active: bool):
        if active:
            self.publishing_state_label.setText("Publishing: ON")
            self.publishing_state_label.setStyleSheet(
                "color: #0f7f8c; font-size: 20px; font-weight: 800;"
            )
        else:
            self.publishing_state_label.setText("Publishing: OFF")
            self.publishing_state_label.setStyleSheet(
                "color: #b34141; font-size: 20px; font-weight: 800;"
            )

        self.start_button.setEnabled(not active)
        self.stop_button.setEnabled(active)

    def set_visualization_mode(self, mode: str):
        self._selected_event_frame_visualization = mode

        active_style = "background-color: #0f7f8c; color: white;"
        inactive_style = "background-color: #2d2d2d; color: #dddddd;"

        if mode == "3ch":
            self.single_channel_button.setStyleSheet(inactive_style)
            self.three_channel_button.setStyleSheet(active_style)
        else:
            self.single_channel_button.setStyleSheet(active_style)
            self.three_channel_button.setStyleSheet(inactive_style)


class LineMotionControlCard(QFrame):
    def __init__(self, on_capture, on_to_start, on_to_end, on_to_center):
        super().__init__()
        self._on_capture = on_capture
        self._on_to_start = on_to_start
        self._on_to_end = on_to_end
        self._on_to_center = on_to_center

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

        self.title_label = QLabel("Line Motion")
        self.title_label.setAlignment(Qt.AlignCenter)
        self.title_label.setStyleSheet("font-size: 18px; font-weight: 600;")

        self.capture_button = QPushButton("capture")
        self.to_start_button = QPushButton("ToStart")
        self.to_end_button = QPushButton("ToEnd")
        self.to_center_button = QPushButton("ToCenter")

        self.capture_button.setStyleSheet("background-color: #6b5500;")
        self.to_start_button.setStyleSheet("background-color: #0051a8;")
        self.to_end_button.setStyleSheet("background-color: #0b6b6b;")
        self.to_center_button.setStyleSheet("background-color: #4b2f7d;")

        self.capture_button.clicked.connect(self._on_capture)
        self.to_start_button.clicked.connect(self._on_to_start)
        self.to_end_button.clicked.connect(self._on_to_end)
        self.to_center_button.clicked.connect(self._on_to_center)

        top_button_row = QHBoxLayout()
        top_button_row.setSpacing(8)
        top_button_row.addWidget(self.capture_button)
        top_button_row.addWidget(self.to_start_button)

        bottom_button_row = QHBoxLayout()
        bottom_button_row.setSpacing(8)
        bottom_button_row.addWidget(self.to_end_button)
        bottom_button_row.addWidget(self.to_center_button)

        layout = QVBoxLayout()
        layout.setContentsMargins(16, 16, 16, 16)
        layout.setSpacing(10)
        layout.addWidget(self.title_label)
        layout.addLayout(top_button_row)
        layout.addLayout(bottom_button_row)
        layout.addStretch(1)
        self.setLayout(layout)

class InterceptionControlCard(QFrame):
    def __init__(self, on_arm, on_mode_change, on_execution_mode_change, initial_mode: str):
        super().__init__()
        self._on_arm = on_arm
        self._on_mode_change = on_mode_change
        self._on_execution_mode_change = on_execution_mode_change
        self._mode = "scene"
        self._execution_mode = "unknown"
        self._transition_pending = False
        self._arm_allowed = False
        self._scene_wet_compatible = True
        self._rollout_wet_compatible = True

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

        self.title_label = QLabel("Ball Interception")
        self.title_label.setAlignment(Qt.AlignCenter)
        self.title_label.setStyleSheet("font-size: 18px; font-weight: 600;")

        self.indicator = QLabel("UNKNOWN")
        self.indicator.setAlignment(Qt.AlignCenter)
        self.indicator.setMinimumHeight(90)
        self.indicator.setStyleSheet("""
            QLabel {
                background-color: #333333;
                color: white;
                font-size: 30px;
                font-weight: 800;
                border-radius: 10px;
                padding: 10px;
            }
        """)

        self.scene_mode_button = QPushButton("Scene")
        self.rollout_mode_button = QPushButton("Rollout")
        self.both_mode_button = QPushButton("Both")

        self.mode_button_group = QButtonGroup(self)
        self.mode_button_group.setExclusive(True)
        for button in (self.scene_mode_button, self.rollout_mode_button, self.both_mode_button):
            button.setCheckable(True)
            self.mode_button_group.addButton(button)

        self.scene_mode_button.clicked.connect(lambda: self._handle_mode_button("scene"))
        self.rollout_mode_button.clicked.connect(lambda: self._handle_mode_button("rollout"))
        self.both_mode_button.clicked.connect(lambda: self._handle_mode_button("both"))

        mode_row = QHBoxLayout()
        mode_row.setSpacing(8)
        mode_row.addWidget(self.scene_mode_button)
        mode_row.addWidget(self.rollout_mode_button)
        mode_row.addWidget(self.both_mode_button)

        self.mode_title = QLabel("ARM ON NEXT TRIAL")
        self.mode_title.setAlignment(Qt.AlignCenter)
        self.mode_title.setStyleSheet("font-size: 12px; font-weight: 700; color: #bbbbbb;")

        self.exec_title = QLabel("ROBOT EXECUTION")
        self.exec_title.setAlignment(Qt.AlignCenter)
        self.exec_title.setStyleSheet("font-size: 12px; font-weight: 700; color: #bbbbbb;")

        self.exec_both_dry_button = QPushButton("Both dry")
        self.exec_scene_wet_button = QPushButton("Scene WET")
        self.exec_rollout_wet_button = QPushButton("Rollout WET")

        self.exec_button_group = QButtonGroup(self)
        self.exec_button_group.setExclusive(True)
        for button in (
            self.exec_both_dry_button,
            self.exec_scene_wet_button,
            self.exec_rollout_wet_button,
        ):
            button.setCheckable(True)
            self.exec_button_group.addButton(button)

        self.exec_both_dry_button.clicked.connect(
            lambda: self._handle_execution_mode_button("both_dry")
        )
        self.exec_scene_wet_button.clicked.connect(
            lambda: self._handle_execution_mode_button("scene_wet")
        )
        self.exec_rollout_wet_button.clicked.connect(
            lambda: self._handle_execution_mode_button("rollout_wet")
        )

        exec_row = QHBoxLayout()
        exec_row.setSpacing(8)
        exec_row.addWidget(self.exec_both_dry_button)
        exec_row.addWidget(self.exec_scene_wet_button)
        exec_row.addWidget(self.exec_rollout_wet_button)

        self.arm_button = QPushButton("ARM INTERCEPT")

        self.arm_button.setStyleSheet("background-color: #b36200;")

        self.arm_button.clicked.connect(self._on_arm)

        layout = QVBoxLayout()
        layout.setContentsMargins(16, 16, 16, 16)
        layout.setSpacing(10)
        layout.addWidget(self.title_label)
        layout.addWidget(self.indicator, 1)
        layout.addWidget(self.mode_title)
        layout.addLayout(mode_row)
        layout.addWidget(self.exec_title)
        layout.addLayout(exec_row)
        layout.addWidget(self.arm_button)
        self.setLayout(layout)

        self.set_mode(initial_mode)
        self.set_execution_mode("unknown")
        self.set_transition_pending(False)
        self._update_indicator("UNARMED", "#333333")

    def _button_for_mode(self, mode: str) -> Optional[QPushButton]:
        mapping = {
            "scene": self.scene_mode_button,
            "rollout": self.rollout_mode_button,
            "both": self.both_mode_button,
        }
        return mapping.get(mode)

    def _set_mode_styles(self) -> None:
        for mode in ("scene", "rollout", "both"):
            button = self._button_for_mode(mode)
            if button is None:
                continue
            bg = "#b36200" if mode == self._mode else "#2d2d2d"
            button.setStyleSheet(f"background-color: {bg};")

    def _button_for_execution_mode(self, mode: str) -> Optional[QPushButton]:
        mapping = {
            "both_dry": self.exec_both_dry_button,
            "scene_wet": self.exec_scene_wet_button,
            "rollout_wet": self.exec_rollout_wet_button,
        }
        return mapping.get(mode)

    def _set_execution_mode_styles(self) -> None:
        for mode in ("both_dry", "scene_wet", "rollout_wet"):
            button = self._button_for_execution_mode(mode)
            if button is None:
                continue
            selected = (mode == self._execution_mode)
            if selected and mode == "both_dry":
                bg = "#4f6070"
            elif selected and mode in {"scene_wet", "rollout_wet"}:
                bg = "#7a0000"
            else:
                bg = "#2d2d2d"
            button.setStyleSheet(f"background-color: {bg};")

    def set_mode(self, mode: str) -> None:
        normalized = str(mode).strip().lower()
        if normalized not in INTERCEPTION_ARM_MODES:
            normalized = "scene"
        self._mode = normalized
        target = self._button_for_mode(self._mode)
        if target is not None:
            for button in (self.scene_mode_button, self.rollout_mode_button, self.both_mode_button):
                button.blockSignals(True)
            target.setChecked(True)
            for button in (self.scene_mode_button, self.rollout_mode_button, self.both_mode_button):
                button.blockSignals(False)
        self._set_mode_styles()
        self._scene_wet_compatible = self._mode in {"scene", "both"}
        self._rollout_wet_compatible = self._mode in {"rollout", "both"}
        self._update_control_enablement()

    def _handle_mode_button(self, mode: str) -> None:
        requested = str(mode).strip().lower()
        if requested == self._mode:
            return
        accepted = bool(self._on_mode_change(requested))
        if accepted:
            self.set_mode(requested)
            return
        # Revert visual check-state if node rejects mode change.
        self.set_mode(self._mode)

    def set_execution_mode(self, mode: str) -> None:
        normalized = str(mode).strip().lower()
        if normalized not in INTERCEPTION_EXECUTION_MODES:
            normalized = "unknown"
        self._execution_mode = normalized

        for button in (
            self.exec_both_dry_button,
            self.exec_scene_wet_button,
            self.exec_rollout_wet_button,
        ):
            button.blockSignals(True)
            button.setChecked(False)
            button.blockSignals(False)

        target = self._button_for_execution_mode(self._execution_mode)
        if target is not None:
            target.blockSignals(True)
            target.setChecked(True)
            target.blockSignals(False)

        self._set_execution_mode_styles()

    def _handle_execution_mode_button(self, mode: str) -> None:
        if self._transition_pending:
            return
        accepted = bool(self._on_execution_mode_change(mode))
        if not accepted:
            self.set_execution_mode(self._execution_mode)

    def set_transition_pending(self, pending: bool) -> None:
        self._transition_pending = bool(pending)
        self._update_control_enablement()

    def _update_control_enablement(self) -> None:
        if self._transition_pending:
            self.exec_both_dry_button.setEnabled(False)
            self.exec_scene_wet_button.setEnabled(False)
            self.exec_rollout_wet_button.setEnabled(False)
        else:
            self.exec_both_dry_button.setEnabled(True)
            self.exec_scene_wet_button.setEnabled(self._scene_wet_compatible)
            self.exec_rollout_wet_button.setEnabled(self._rollout_wet_compatible)

        self.arm_button.setEnabled((not self._transition_pending) and self._arm_allowed)

    def set_arm_allowed(self, allowed: bool) -> None:
        self._arm_allowed = bool(allowed)
        self._update_control_enablement()

    def _update_indicator(self, text: str, color: str) -> None:
        self.indicator.setText(text)
        self.indicator.setStyleSheet(f"""
            QLabel {{
                background-color: {color};
                color: white;
                font-size: 30px;
                font-weight: 800;
                border-radius: 10px;
                padding: 10px;
            }}
        """)

    def set_controller_statuses(
        self,
        mode: str,
        scene_status: str,
        rollout_status: str,
        execution_mode: str,
        transition_pending: bool,
        arm_allowed: bool,
    ) -> None:
        self.set_mode(mode)
        self.set_execution_mode(execution_mode)
        self.set_transition_pending(transition_pending)
        self.set_arm_allowed(arm_allowed)
        scene_active = controller_state_is_active(scene_status)
        rollout_active = controller_state_is_active(rollout_status)
        live_label = execution_mode_label(execution_mode)

        if execution_mode == "both_wet":
            self._update_indicator("UNSAFE\nLIVE: BOTH", "#d40000")
            return
        if execution_mode == "unknown":
            armed_text = "ARMED" if (scene_active or rollout_active) else "UNARMED"
            self._update_indicator(f"{armed_text}\nLIVE: UNKNOWN", "#5f2f00")
            return

        if self._mode == "scene":
            if scene_active:
                self._update_indicator(f"ARMED: SCENE\nLIVE: {live_label}", "#b36200")
            else:
                self._update_indicator(f"UNARMED\nLIVE: {live_label}", "#333333")
            return

        if self._mode == "rollout":
            if rollout_active:
                self._update_indicator(f"ARMED: ROLLOUT\nLIVE: {live_label}", "#b36200")
            else:
                self._update_indicator(f"UNARMED\nLIVE: {live_label}", "#333333")
            return

        if scene_active and rollout_active:
            self._update_indicator(f"ARMED: BOTH\nLIVE: {live_label}", "#b36200")
        elif scene_active or rollout_active:
            armed_mode = "SCENE" if scene_active else "ROLLOUT"
            self._update_indicator(f"ARMED: {armed_mode}\nLIVE: {live_label}", "#6a1b1b")
        else:
            self._update_indicator(f"UNARMED\nLIVE: {live_label}", "#333333")


class TeleopEpisodeCard(QFrame):
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

        self.teleop_title = QLabel("Teleop")
        self.teleop_title.setAlignment(Qt.AlignCenter)
        self.teleop_title.setStyleSheet("font-size: 16px; font-weight: 600;")

        self.episode_title = QLabel("Episode")
        self.episode_title.setAlignment(Qt.AlignCenter)
        self.episode_title.setStyleSheet("font-size: 16px; font-weight: 600;")

        self.teleop_indicator = QLabel("DISABLED")
        self.teleop_indicator.setAlignment(Qt.AlignCenter)
        self.teleop_indicator.setMinimumHeight(74)

        self.episode_indicator = QLabel("IDLE")
        self.episode_indicator.setAlignment(Qt.AlignCenter)
        self.episode_indicator.setMinimumHeight(74)

        teleop_col = QVBoxLayout()
        teleop_col.setContentsMargins(0, 0, 0, 0)
        teleop_col.setSpacing(6)
        teleop_col.addWidget(self.teleop_title)
        teleop_col.addWidget(self.teleop_indicator, 1)

        episode_col = QVBoxLayout()
        episode_col.setContentsMargins(0, 0, 0, 0)
        episode_col.setSpacing(6)
        episode_col.addWidget(self.episode_title)
        episode_col.addWidget(self.episode_indicator, 1)

        row = QHBoxLayout()
        row.setSpacing(10)
        row.addLayout(teleop_col, 1)
        row.addLayout(episode_col, 1)

        layout = QVBoxLayout()
        layout.setContentsMargins(16, 12, 16, 12)
        layout.setSpacing(8)
        layout.addLayout(row)
        self.setLayout(layout)

        self.set_teleop(False)
        self.set_episode_active(False)

    def set_teleop(self, enabled: bool):
        if enabled:
            self.teleop_indicator.setText("ENABLED")
            bg = "#0051a8"
        else:
            self.teleop_indicator.setText("DISABLED")
            bg = "#7a0000"

        self.teleop_indicator.setStyleSheet(f"""
            QLabel {{
                background-color: {bg};
                color: white;
                font-size: 24px;
                font-weight: 800;
                border-radius: 10px;
                padding: 8px;
            }}
        """)

    def set_episode_active(self, active: bool):
        if active:
            self.episode_indicator.setText("ACTIVE")
            bg = "#008c3a"
        else:
            self.episode_indicator.setText("IDLE")
            bg = "#7a0000"

        self.episode_indicator.setStyleSheet(f"""
            QLabel {{
                background-color: {bg};
                color: white;
                font-size: 24px;
                font-weight: 800;
                border-radius: 10px;
                padding: 8px;
            }}
        """)


class RGBModeControlCard(QFrame):
    def __init__(
        self,
        on_toggle_overlay,
        on_select_source,
        overlay_available: bool,
        initial_overlay: bool = False,
        initial_source: str = "raw",
    ):
        super().__init__()
        self._on_toggle_overlay = on_toggle_overlay
        self._on_select_source = on_select_source
        self._overlay_available = bool(overlay_available)

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
                padding: 9px 12px;
                font-size: 15px;
                font-weight: 700;
                min-height: 22px;
            }
            QPushButton:disabled {
                background-color: #2d2d2d;
                color: #888888;
                border-color: #3a3a3a;
            }
        """)

        self.title_label = QLabel("RGB Mode")
        self.title_label.setAlignment(Qt.AlignCenter)
        self.title_label.setStyleSheet("font-size: 18px; font-weight: 600;")

        self.live_overlay_button = QPushButton()
        self.raw_debug_button = QPushButton()

        self.live_overlay_button.clicked.connect(self._on_toggle_overlay)
        self.raw_debug_button.clicked.connect(self._toggle_source)

        self.live_overlay_button.setEnabled(self._overlay_available)

        layout = QVBoxLayout()
        layout.setContentsMargins(16, 12, 16, 12)
        layout.setSpacing(10)
        layout.addWidget(self.title_label)
        layout.addWidget(self.live_overlay_button)
        layout.addWidget(self.raw_debug_button)
        self.setLayout(layout)

        self.set_overlay_mode(initial_overlay)
        self.set_source_mode(initial_source)

    def _toggle_source(self):
        current = str(getattr(self, "_source_mode", "raw")).strip().lower()
        next_mode = "debug" if current == "raw" else "raw"
        self._on_select_source(next_mode)

    def set_overlay_mode(self, overlay: bool):
        if not self._overlay_available:
            self.live_overlay_button.setText("RGB: Live (Overlay N/A)")
            self.live_overlay_button.setStyleSheet("background-color: #2d2d2d; color: #888888;")
            return

        if overlay:
            self.live_overlay_button.setText("RGB: Overlay")
            self.live_overlay_button.setStyleSheet("background-color: #0f7f8c; color: white;")
        else:
            self.live_overlay_button.setText("RGB: Live")
            self.live_overlay_button.setStyleSheet("background-color: #2d2d2d; color: #dddddd;")

    def set_source_mode(self, source: str):
        self._source_mode = str(source).strip().lower()
        if self._source_mode == "debug":
            self.raw_debug_button.setText("RGB: Debug")
            self.raw_debug_button.setStyleSheet("background-color: #4b2f7d; color: white;")
        else:
            self.raw_debug_button.setText("RGB: Raw")
            self.raw_debug_button.setStyleSheet("background-color: #0051a8; color: white;")

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

        self.declare_parameter("rgb_topic", RGB_CAM_TOPIC)
        self.declare_parameter("rgb_debug_topic", "/scene_localizer_debug/top_cam/debug_image")
        self.declare_parameter("selected_rgb_source", "raw")  # raw | debug
        self.declare_parameter("event_frame_mono_topic", EVENT_FRAME_MONO_TOPIC)
        self.declare_parameter("event_frame_3ch_topic", EVENT_FRAME_3CH_TOPIC)
        self.declare_parameter("selected_event_frame_visualization", DEFAULT_EVENT_FRAME_VISUALIZATION)
        self.declare_parameter("overlay_hdf5_path", "")
        self.declare_parameter("overlay_data_path", "/observations/images/rgb")
        self.declare_parameter("overlay_frame_index", 0)
        self.declare_parameter("overlay_alpha", 0.5)
        self.declare_parameter("overlay_rotate_ref_180", True)
        self.declare_parameter("rgb_rotate_180", False)
        self.declare_parameter("episode_control_topic", "/episode/control")
        self.declare_parameter("teleop_control_topic", "/teleop/control")
        self.declare_parameter("num_valid_episodes_topic", "/data_collection/num_valid_episodes")
        self.declare_parameter("start_recording_service", "/record_manager/start_recording")
        self.declare_parameter("stop_recording_service", "/record_manager/stop_recording")
        self.declare_parameter("set_debug_bypass_service", "/record_manager/set_debug_bypass_topic_presence")
        self.declare_parameter("start_event_frames_service", "/openmv_cam/start_event_frame_publishing")
        self.declare_parameter("stop_event_frames_service", "/openmv_cam/stop_event_frame_publishing")
        self.declare_parameter("set_line_params_service", "/trajectory_executor/set_line_params")
        self.declare_parameter("capture_line_center_service", "/trajectory_executor/capture_line_center")
        self.declare_parameter("trajectory_executor_action", "/trajectory_executor")
        self.declare_parameter("scene_interception_arm_service", "/interception_controller/arm")
        self.declare_parameter("scene_interception_disarm_service", "/interception_controller/disarm")
        self.declare_parameter("scene_interception_set_dry_run_service", "/interception_controller/set_dry_run")
        self.declare_parameter("scene_interception_status_topic", "/interception_controller/status")
        self.declare_parameter("rollout_interception_arm_service", "/rollout_interception_controller/arm")
        self.declare_parameter("rollout_interception_disarm_service", "/rollout_interception_controller/disarm")
        self.declare_parameter("rollout_interception_set_dry_run_service", "/rollout_interception_controller/set_dry_run")
        self.declare_parameter("rollout_interception_status_topic", "/rollout_interception_controller/status")
        self.declare_parameter("interception_arm_mode", "scene")
        self.declare_parameter("interception_arm_mode_topic", "/teleop/interception_arm_mode")
        self.declare_parameter("interception_arm_inhibit_topic", "/teleop/interception_arm_inhibit")
        self.declare_parameter("interception_status_stale_sec", 2.0)
        self.declare_parameter("reset_episode_counter_service", "/data_collection/reset_episode_counter")
        self.declare_parameter("line_ee_name", "right_fr3_hand_tcp")
        self.declare_parameter("line_profile_name", "goto_s_x_10cm")
        self.declare_parameter("service_timeout_sec", 3.0)

        self.rgb_topic = self.get_parameter("rgb_topic").value
        self.rgb_debug_topic = self.get_parameter("rgb_debug_topic").value
        self.selected_rgb_source = str(self.get_parameter("selected_rgb_source").value).strip().lower()
        if self.selected_rgb_source not in ("raw", "debug"):
            self.selected_rgb_source = "raw"

        self.event_frame_mono_topic = self.get_parameter("event_frame_mono_topic").value
        self.event_frame_3ch_topic = self.get_parameter("event_frame_3ch_topic").value
        self.selected_event_frame_visualization = str(
            self.get_parameter("selected_event_frame_visualization").value
        ).strip().lower()
        if self.selected_event_frame_visualization not in ("mono", "3ch"):
            self.selected_event_frame_visualization = DEFAULT_EVENT_FRAME_VISUALIZATION
        self.overlay_hdf5_path = str(self.get_parameter("overlay_hdf5_path").value).strip()
        self.overlay_data_path = str(self.get_parameter("overlay_data_path").value).strip()
        self.overlay_frame_index = max(0, int(self.get_parameter("overlay_frame_index").value))
        self.overlay_alpha = float(self.get_parameter("overlay_alpha").value)
        self.overlay_alpha = min(1.0, max(0.0, self.overlay_alpha))
        self.overlay_rotate_ref_180 = bool(self.get_parameter("overlay_rotate_ref_180").value)
        self.rgb_rotate_180 = bool(self.get_parameter("rgb_rotate_180").value)
        self.episode_control_topic = self.get_parameter("episode_control_topic").value
        self.teleop_control_topic = self.get_parameter("teleop_control_topic").value
        self.num_valid_episodes_topic = self.get_parameter("num_valid_episodes_topic").value
        self.start_recording_service = self.get_parameter("start_recording_service").value
        self.stop_recording_service = self.get_parameter("stop_recording_service").value
        self.set_debug_bypass_service = self.get_parameter("set_debug_bypass_service").value
        self.start_event_frames_service = self.get_parameter("start_event_frames_service").value
        self.stop_event_frames_service = self.get_parameter("stop_event_frames_service").value
        self.set_line_params_service = self.get_parameter("set_line_params_service").value
        self.capture_line_center_service = self.get_parameter("capture_line_center_service").value
        self.trajectory_executor_action = self.get_parameter("trajectory_executor_action").value
        self.line_ee_name = str(self.get_parameter("line_ee_name").value)
        self.line_profile_name = str(self.get_parameter("line_profile_name").value)
        self.scene_interception_arm_service = self.get_parameter("scene_interception_arm_service").value
        self.scene_interception_disarm_service = self.get_parameter("scene_interception_disarm_service").value
        self.scene_interception_set_dry_run_service = self.get_parameter("scene_interception_set_dry_run_service").value
        self.scene_interception_status_topic = self.get_parameter("scene_interception_status_topic").value
        self.rollout_interception_arm_service = self.get_parameter("rollout_interception_arm_service").value
        self.rollout_interception_disarm_service = self.get_parameter("rollout_interception_disarm_service").value
        self.rollout_interception_set_dry_run_service = self.get_parameter("rollout_interception_set_dry_run_service").value
        self.rollout_interception_status_topic = self.get_parameter("rollout_interception_status_topic").value
        configured_mode = str(self.get_parameter("interception_arm_mode").value).strip().lower()
        self.interception_arm_mode = self._validate_interception_arm_mode(configured_mode)
        self.interception_arm_mode_topic = self.get_parameter("interception_arm_mode_topic").value
        self.interception_arm_inhibit_topic = self.get_parameter("interception_arm_inhibit_topic").value
        self.reset_episode_counter_service = self.get_parameter("reset_episode_counter_service").value
        self._interception_transition_active = False
        self._interception_arm_inhibit_active = False

        self.rgb_buffer = LatestFrameBuffer()
        self.event_buffer = LatestFrameBuffer()
        self.overlay_ref_bgr: Optional[np.ndarray] = None
        self.overlay_status_text = "Overlay disabled (no HDF5 configured)."
        self._load_overlay_reference_image()

        self.state_lock = threading.Lock()
        self.state = TeleopState()
        self.state.interception_arm_mode = self.interception_arm_mode
        self.state.execution_mode_transition_pending = False

        self.rgb_sub = None
        self.switch_rgb_source_mode(self.selected_rgb_source)
        self._interception_guard_timer = self.create_timer(
            0.25,
            self._refresh_interception_guard_timer_cb,
        )
        self.event_frame_subscription = None
        self.switch_event_frame_visualization_mode(self.selected_event_frame_visualization)
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
        self.set_debug_bypass_client = self.create_client(SetBool, self.set_debug_bypass_service)
        self.start_event_frames_client = self.create_client(Trigger, self.start_event_frames_service)
        self.stop_event_frames_client = self.create_client(Trigger, self.stop_event_frames_service)
        self.set_line_params_client = self.create_client(SetLineParams, self.set_line_params_service)
        self.capture_line_center_client = self.create_client(CaptureLineCenter, self.capture_line_center_service)
        self.trajectory_executor_client = ActionClient(self, LineTrajectory, self.trajectory_executor_action)
        self.scene_interception_arm_client = self.create_client(Trigger, self.scene_interception_arm_service)
        self.scene_interception_disarm_client = self.create_client(Trigger, self.scene_interception_disarm_service)
        self.scene_interception_set_dry_run_client = self.create_client(
            SetBool,
            self.scene_interception_set_dry_run_service,
        )
        self.rollout_interception_arm_client = self.create_client(Trigger, self.rollout_interception_arm_service)
        self.rollout_interception_disarm_client = self.create_client(Trigger, self.rollout_interception_disarm_service)
        self.rollout_interception_set_dry_run_client = self.create_client(
            SetBool,
            self.rollout_interception_set_dry_run_service,
        )
        self.reset_episode_counter_client = self.create_client(Trigger, self.reset_episode_counter_service)

        self.scene_interception_status_sub = self.create_subscription(
            String,
            self.scene_interception_status_topic,
            self.scene_interception_status_cb,
            10,
        )
        self.rollout_interception_status_sub = self.create_subscription(
            String,
            self.rollout_interception_status_topic,
            self.rollout_interception_status_cb,
            10,
        )

        mode_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.interception_arm_mode_pub = self.create_publisher(
            String,
            self.interception_arm_mode_topic,
            mode_qos,
        )
        self.interception_arm_inhibit_pub = self.create_publisher(
            Bool,
            self.interception_arm_inhibit_topic,
            mode_qos,
        )
        self.publish_interception_arm_mode()
        self._update_interception_arm_inhibit_from_state()

        self.get_logger().info(f"RGB topic: {self.rgb_topic}")
        self.get_logger().info(f"RGB debug topic: {self.rgb_debug_topic}")
        self.get_logger().info(f"Selected RGB source: {self.selected_rgb_source}")
        self.get_logger().info(f"Event frame mono topic: {self.event_frame_mono_topic}")
        self.get_logger().info(f"Event frame 3-channel topic: {self.event_frame_3ch_topic}")
        self.get_logger().info(f"Selected event frame visualization: {self.selected_event_frame_visualization}")
        self.get_logger().info(f"Overlay path: {self.overlay_hdf5_path}")
        self.get_logger().info(f"Overlay data path: {self.overlay_data_path}")
        self.get_logger().info(f"Overlay frame index: {self.overlay_frame_index}")
        self.get_logger().info(f"Overlay alpha: {self.overlay_alpha:.3f}")
        self.get_logger().info(f"Overlay rotate ref 180: {self.overlay_rotate_ref_180}")
        self.get_logger().info(f"RGB rotate ref 180: {self.rgb_rotate_180}")
        self.get_logger().info(self.overlay_status_text)
        self.get_logger().info(f"Episode control topic: {self.episode_control_topic}")
        self.get_logger().info(f"Teleop control topic: {self.teleop_control_topic}")
        self.get_logger().info(f"Num valid episodes topic: {self.num_valid_episodes_topic}")
        self.get_logger().info(f"Start recording service: {self.start_recording_service}")
        self.get_logger().info(f"Stop recording service: {self.stop_recording_service}")
        self.get_logger().info(f"Set debug bypass service: {self.set_debug_bypass_service}")
        self.get_logger().info(f"Start event frames service: {self.start_event_frames_service}")
        self.get_logger().info(f"Stop event frames service: {self.stop_event_frames_service}")
        self.get_logger().info(f"Set line params service: {self.set_line_params_service}")
        self.get_logger().info(f"Capture line center service: {self.capture_line_center_service}")
        self.get_logger().info(f"Trajectory executor action: {self.trajectory_executor_action}")
        self.get_logger().info(f"Line EE name: {self.line_ee_name}")
        self.get_logger().info(f"Line profile name: {self.line_profile_name}")
        self.get_logger().info(f"Scene interception arm service: {self.scene_interception_arm_service}")
        self.get_logger().info(f"Scene interception disarm service: {self.scene_interception_disarm_service}")
        self.get_logger().info(f"Scene interception set_dry_run service: {self.scene_interception_set_dry_run_service}")
        self.get_logger().info(f"Scene interception status topic: {self.scene_interception_status_topic}")
        self.get_logger().info(f"Rollout interception arm service: {self.rollout_interception_arm_service}")
        self.get_logger().info(f"Rollout interception disarm service: {self.rollout_interception_disarm_service}")
        self.get_logger().info(f"Rollout interception set_dry_run service: {self.rollout_interception_set_dry_run_service}")
        self.get_logger().info(f"Rollout interception status topic: {self.rollout_interception_status_topic}")
        self.get_logger().info(f"Interception arm mode: {self.interception_arm_mode}")
        self.get_logger().info(f"Interception arm mode topic: {self.interception_arm_mode_topic}")
        self.get_logger().info(f"Interception arm inhibit topic: {self.interception_arm_inhibit_topic}")
        self.get_logger().info(f"Reset episode counter service: {self.reset_episode_counter_service}")

    def _validate_interception_arm_mode(self, mode: str) -> str:
        normalized = str(mode).strip().lower()
        if normalized in INTERCEPTION_ARM_MODES:
            return normalized
        self.get_logger().warning(
            f"Invalid interception_arm_mode '{mode}', falling back to 'scene'"
        )
        return "scene"

    def publish_interception_arm_mode(self) -> None:
        msg = String()
        msg.data = self.interception_arm_mode
        self.interception_arm_mode_pub.publish(msg)

    def publish_interception_arm_inhibit(self, inhibit: bool) -> None:
        if bool(inhibit) == bool(self._interception_arm_inhibit_active):
            return
        msg = Bool()
        msg.data = bool(inhibit)
        self.interception_arm_inhibit_pub.publish(msg)
        self._interception_arm_inhibit_active = bool(inhibit)

    def _set_transition_pending(self, pending: bool) -> None:
        with self.state_lock:
            self.state.execution_mode_transition_pending = bool(pending)
        self._interception_transition_active = bool(pending)
        self._update_interception_arm_inhibit_from_state()

    def _status_is_stale(self, last_receive_monotonic: Optional[float]) -> bool:
        if last_receive_monotonic is None:
            return True
        stale_after = max(0.2, self._get_double_param("interception_status_stale_sec"))
        return (time.monotonic() - float(last_receive_monotonic)) > stale_after

    def _derive_execution_mode_from_state_locked(self) -> str:
        if self._status_is_stale(self.state.scene_status_receive_monotonic):
            return "unknown"
        if self._status_is_stale(self.state.rollout_status_receive_monotonic):
            return "unknown"
        return derive_execution_mode(
            self.state.scene_interception_status,
            self.state.rollout_interception_status,
        )

    def _arm_target_allows_execution_mode(self, arm_mode: str, execution_mode: str) -> bool:
        arm = str(arm_mode).strip().lower()
        exec_mode = str(execution_mode).strip().lower()
        if exec_mode == "scene_wet":
            return arm in {"scene", "both"}
        if exec_mode == "rollout_wet":
            return arm in {"rollout", "both"}
        return True

    def _arm_safety_block_reason_locked(self) -> Optional[str]:
        if self.state.execution_mode_transition_pending:
            return "Execution-mode transition is in progress."

        execution_mode = self._derive_execution_mode_from_state_locked()
        if execution_mode == "unknown":
            return "Execution mode is UNKNOWN (missing/stale controller status)."
        if execution_mode == "both_wet":
            return "Execution mode is UNSAFE: BOTH WET."
        if not self._arm_target_allows_execution_mode(self.interception_arm_mode, execution_mode):
            return (
                f"Arm target '{self.interception_arm_mode}' is incompatible with "
                f"execution mode '{execution_mode}'."
            )
        return None

    def get_interception_execution_snapshot(self) -> tuple[str, bool, str]:
        with self.state_lock:
            mode = self._derive_execution_mode_from_state_locked()
            reason = self._arm_safety_block_reason_locked()
        return mode, (reason is None), "" if reason is None else reason

    def _update_interception_arm_inhibit_from_state(self) -> None:
        with self.state_lock:
            reason = self._arm_safety_block_reason_locked()
        self.publish_interception_arm_inhibit(reason is not None)

    def _refresh_interception_guard_timer_cb(self) -> None:
        self._update_interception_arm_inhibit_from_state()

    def _call_setbool_service_sync(self, client, service_name: str, value: bool) -> tuple[bool, str]:
        timeout_sec = self._get_double_param("service_timeout_sec")
        if not self._wait_for_service(client, timeout_sec):
            return False, f"service unavailable ({service_name})"

        req = SetBool.Request()
        req.data = bool(value)
        future = client.call_async(req)
        start = time.monotonic()
        while rclpy.ok() and not future.done():
            if time.monotonic() - start > timeout_sec:
                return False, f"service call timed out ({service_name})"
            time.sleep(0.02)

        result = future.result()
        if result is None:
            return False, f"service call failed ({service_name})"
        if not result.success:
            detail = str(result.message).strip()
            if detail:
                return False, detail
            return False, f"service returned failure ({service_name})"
        return True, str(result.message).strip()

    def _run_execution_mode_transition(self, target_mode: str) -> None:
        target = str(target_mode).strip().lower()
        self._set_last_service_status("Changing execution mode...")
        self._set_transition_pending(True)

        try:
            if target == "both_dry":
                scene_ok, scene_detail = self._call_setbool_service_sync(
                    self.scene_interception_set_dry_run_client,
                    self.scene_interception_set_dry_run_service,
                    True,
                )
                rollout_ok, rollout_detail = self._call_setbool_service_sync(
                    self.rollout_interception_set_dry_run_client,
                    self.rollout_interception_set_dry_run_service,
                    True,
                )
                if scene_ok and rollout_ok:
                    status = "Execution mode updated: BOTH DRY."
                elif scene_ok and not rollout_ok:
                    status = f"Scene set dry; rollout set-dry failed: {rollout_detail}"
                elif rollout_ok and not scene_ok:
                    status = f"Rollout set dry; scene set-dry failed: {scene_detail}"
                else:
                    status = f"Set-dry failed: scene={scene_detail}; rollout={rollout_detail}"
                self._set_last_service_status(status)
                return

            if target == "scene_wet":
                rollout_ok, rollout_detail = self._call_setbool_service_sync(
                    self.rollout_interception_set_dry_run_client,
                    self.rollout_interception_set_dry_run_service,
                    True,
                )
                if not rollout_ok:
                    self._set_last_service_status(
                        "Blocked SCENE WET transition: rollout must be DRY first "
                        f"({rollout_detail})"
                    )
                    return

                scene_ok, scene_detail = self._call_setbool_service_sync(
                    self.scene_interception_set_dry_run_client,
                    self.scene_interception_set_dry_run_service,
                    False,
                )
                if scene_ok:
                    self._set_last_service_status("Execution mode updated: SCENE WET.")
                else:
                    self._set_last_service_status(f"Scene set-wet failed: {scene_detail}")
                return

            if target == "rollout_wet":
                scene_ok, scene_detail = self._call_setbool_service_sync(
                    self.scene_interception_set_dry_run_client,
                    self.scene_interception_set_dry_run_service,
                    True,
                )
                if not scene_ok:
                    self._set_last_service_status(
                        "Blocked ROLLOUT WET transition: scene must be DRY first "
                        f"({scene_detail})"
                    )
                    return

                rollout_ok, rollout_detail = self._call_setbool_service_sync(
                    self.rollout_interception_set_dry_run_client,
                    self.rollout_interception_set_dry_run_service,
                    False,
                )
                if rollout_ok:
                    self._set_last_service_status("Execution mode updated: ROLLOUT WET.")
                else:
                    self._set_last_service_status(f"Rollout set-wet failed: {rollout_detail}")
                return

            self._set_last_service_status(f"Unknown execution mode request: {target}")
        except Exception as e:
            self._set_last_service_status(f"Execution mode transition failed: {e}")
        finally:
            self._set_transition_pending(False)

    def request_execution_mode_change(self, mode: str) -> bool:
        normalized = str(mode).strip().lower()
        if normalized not in INTERCEPTION_EXECUTION_MODES:
            self._set_last_service_status(f"Invalid execution mode: {mode}")
            return False

        with self.state_lock:
            if self.state.execution_mode_transition_pending:
                self.state.last_service_status = "Execution-mode transition already in progress."
                return False

            scene_active = controller_state_is_active(self.state.scene_interception_status)
            rollout_active = controller_state_is_active(self.state.rollout_interception_status)
            if scene_active or rollout_active:
                self.state.last_service_status = (
                    "Disarm interception controllers before changing execution mode."
                )
                return False

        threading.Thread(
            target=self._run_execution_mode_transition,
            args=(normalized,),
            daemon=True,
        ).start()
        return True

    def _selected_controller_keys(self):
        if self.interception_arm_mode == "rollout":
            return ["rollout"]
        if self.interception_arm_mode == "both":
            return ["scene", "rollout"]
        return ["scene"]

    def _controller_specs(self, action: str):
        specs = {
            "scene": {
                "label": "scene",
                "arm": (self.scene_interception_arm_client, self.scene_interception_arm_service),
                "disarm": (self.scene_interception_disarm_client, self.scene_interception_disarm_service),
            },
            "rollout": {
                "label": "rollout",
                "arm": (self.rollout_interception_arm_client, self.rollout_interception_arm_service),
                "disarm": (self.rollout_interception_disarm_client, self.rollout_interception_disarm_service),
            },
        }
        selected = []
        for key in self._selected_controller_keys():
            if key in specs:
                client, service_name = specs[key][action]
                selected.append((specs[key]["label"], client, service_name))
        return selected

    def _call_trigger_service_sync(self, client, service_name: str) -> tuple[bool, str]:
        timeout_sec = self._get_double_param("service_timeout_sec")
        if not self._wait_for_service(client, timeout_sec):
            return False, f"service unavailable ({service_name})"

        future = client.call_async(Trigger.Request())
        start = time.monotonic()
        while rclpy.ok() and not future.done():
            if time.monotonic() - start > timeout_sec:
                return False, f"service call timed out ({service_name})"
            time.sleep(0.02)

        result = future.result()
        if result is None:
            return False, f"service call failed ({service_name})"
        if not result.success:
            detail = str(result.message).strip()
            if detail:
                return False, detail
            return False, f"service returned failure ({service_name})"
        return True, ""

    def _run_interception_dispatch(self, action: str) -> None:
        action_name = "ARM" if action == "arm" else "DISARM"
        selections = self._controller_specs(action)
        self.get_logger().info(
            f"Requested ball interception {action_name} in mode={self.interception_arm_mode}"
        )

        if not selections:
            self._set_last_service_status("No interception controllers selected.")
            return

        def worker():
            results: dict[str, tuple[bool, str]] = {}
            for label, client, service_name in selections:
                ok, detail = self._call_trigger_service_sync(client, service_name)
                results[label] = (ok, detail)

            labels = [label for label, _client, _service in selections]
            if action == "arm":
                if labels == ["scene"]:
                    ok, detail = results["scene"]
                    status = "Armed scene controller." if ok else f"Scene arm failed: {detail}"
                elif labels == ["rollout"]:
                    ok, detail = results["rollout"]
                    status = "Armed rollout controller." if ok else f"Rollout arm failed: {detail}"
                else:
                    scene_ok, scene_detail = results.get("scene", (False, "not attempted"))
                    rollout_ok, rollout_detail = results.get("rollout", (False, "not attempted"))
                    if scene_ok and rollout_ok:
                        status = "Armed scene and rollout controllers."
                    elif scene_ok and not rollout_ok:
                        status = f"Scene armed; rollout arm failed: {rollout_detail}"
                    elif rollout_ok and not scene_ok:
                        status = f"Rollout armed; scene arm failed: {scene_detail}"
                    else:
                        status = (
                            f"Scene arm failed: {scene_detail}; "
                            f"rollout arm failed: {rollout_detail}"
                        )
            else:
                if labels == ["scene"]:
                    ok, detail = results["scene"]
                    status = "Disarmed scene controller." if ok else f"Scene disarm failed: {detail}"
                elif labels == ["rollout"]:
                    ok, detail = results["rollout"]
                    status = "Disarmed rollout controller." if ok else f"Rollout disarm failed: {detail}"
                else:
                    scene_ok, scene_detail = results.get("scene", (False, "not attempted"))
                    rollout_ok, rollout_detail = results.get("rollout", (False, "not attempted"))
                    if scene_ok and rollout_ok:
                        status = "Disarmed scene and rollout controllers."
                    elif scene_ok and not rollout_ok:
                        status = f"Scene disarmed; rollout disarm failed: {rollout_detail}"
                    elif rollout_ok and not scene_ok:
                        status = f"Rollout disarmed; scene disarm failed: {scene_detail}"
                    else:
                        status = (
                            f"Scene disarm failed: {scene_detail}; "
                            f"rollout disarm failed: {rollout_detail}"
                        )

            self._set_last_service_status(status)

        threading.Thread(target=worker, daemon=True).start()

    def request_interception_arm_mode_change(self, mode: str) -> bool:
        normalized = self._validate_interception_arm_mode(mode)
        with self.state_lock:
            if self.state.execution_mode_transition_pending:
                self.state.last_service_status = (
                    "Cannot change arm mode while execution-mode transition is active."
                )
                return False
            scene_active = controller_state_is_active(self.state.scene_interception_status)
            rollout_active = controller_state_is_active(self.state.rollout_interception_status)
            if scene_active or rollout_active:
                self.state.last_service_status = (
                    "Disarm interception controllers before changing arm mode."
                )
                return False
            self.interception_arm_mode = normalized
            self.state.interception_arm_mode = normalized
            self.state.last_service_status = f"Selected interception arm mode: {normalized}."

        self.publish_interception_arm_mode()
        self._update_interception_arm_inhibit_from_state()
        self.get_logger().info(f"Interception arm mode changed to {normalized}")
        return True

    def _load_overlay_reference_image(self):
        if not self.overlay_hdf5_path:
            self.overlay_ref_bgr = None
            self.overlay_status_text = "Overlay disabled (no overlay path configured)."
            return

        try:
            overlay_path = self.overlay_hdf5_path
            overlay_path_obj = Path(overlay_path)
            lower_path = overlay_path.lower()

            if lower_path.endswith((".jpg", ".jpeg", ".png")):
                ref_bgr = cv2.imread(overlay_path, cv2.IMREAD_COLOR)
                if ref_bgr is None:
                    raise ValueError(f"Failed to read overlay image: {overlay_path}")
                source_desc = f"{overlay_path}"
            elif lower_path.endswith((".hdf5", ".h5")):
                # If a same-basename image exists next to the HDF5, prefer it.
                paired_img_path = None
                for ext in (".jpg", ".jpeg", ".png"):
                    candidate = overlay_path_obj.with_suffix(ext)
                    if candidate.exists():
                        paired_img_path = candidate
                        break

                if paired_img_path is not None:
                    ref_bgr = cv2.imread(str(paired_img_path), cv2.IMREAD_COLOR)
                    if ref_bgr is None:
                        raise ValueError(f"Failed to read overlay image: {paired_img_path}")
                    source_desc = f"{paired_img_path} (paired with {overlay_path})"
                else:
                    with h5py.File(overlay_path, "r") as f:
                        if self.overlay_data_path not in f:
                            raise KeyError(
                                f"Data path '{self.overlay_data_path}' not found in {overlay_path}"
                            )

                        data = f[self.overlay_data_path]
                        if data.ndim == 4:
                            frame_idx = min(self.overlay_frame_index, data.shape[0] - 1)
                            img = data[frame_idx]
                        elif data.ndim == 3:
                            frame_idx = 0
                            img = data[...]
                        else:
                            raise ValueError(f"Unsupported image shape: {data.shape}")

                    img = np.asarray(img)

                    if img.ndim == 3 and img.shape[0] in (1, 3, 4) and img.shape[-1] not in (1, 3, 4):
                        img = np.transpose(img, (1, 2, 0))

                    if np.issubdtype(img.dtype, np.floating):
                        if img.max() <= 1.0:
                            img = img * 255.0
                        img = np.clip(img, 0, 255).astype(np.uint8)
                    else:
                        img = img.astype(np.uint8)

                    if img.ndim == 2:
                        ref_bgr = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
                    elif img.shape[2] == 1:
                        ref_bgr = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
                    elif img.shape[2] == 3:
                        # Match overlay.py behavior: assume HDF5 image is RGB.
                        ref_bgr = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
                    elif img.shape[2] == 4:
                        ref_bgr = cv2.cvtColor(img, cv2.COLOR_RGBA2BGR)
                    else:
                        raise ValueError(f"Unsupported channel count: {img.shape}")

                    source_desc = f"{overlay_path} [{self.overlay_data_path}] frame {frame_idx}"
            else:
                with h5py.File(overlay_path, "r") as f:
                    if self.overlay_data_path not in f:
                        raise KeyError(
                            f"Data path '{self.overlay_data_path}' not found in {overlay_path}"
                        )

                    data = f[self.overlay_data_path]
                    if data.ndim == 4:
                        frame_idx = min(self.overlay_frame_index, data.shape[0] - 1)
                        img = data[frame_idx]
                    elif data.ndim == 3:
                        frame_idx = 0
                        img = data[...]
                    else:
                        raise ValueError(f"Unsupported image shape: {data.shape}")

                img = np.asarray(img)

                if img.ndim == 3 and img.shape[0] in (1, 3, 4) and img.shape[-1] not in (1, 3, 4):
                    img = np.transpose(img, (1, 2, 0))

                if np.issubdtype(img.dtype, np.floating):
                    if img.max() <= 1.0:
                        img = img * 255.0
                    img = np.clip(img, 0, 255).astype(np.uint8)
                else:
                    img = img.astype(np.uint8)

                if img.ndim == 2:
                    ref_bgr = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
                elif img.shape[2] == 1:
                    ref_bgr = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
                elif img.shape[2] == 3:
                    # Match overlay.py behavior: assume HDF5 image is RGB.
                    ref_bgr = cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
                elif img.shape[2] == 4:
                    ref_bgr = cv2.cvtColor(img, cv2.COLOR_RGBA2BGR)
                else:
                    raise ValueError(f"Unsupported channel count: {img.shape}")

                source_desc = f"{overlay_path} [{self.overlay_data_path}] frame {frame_idx}"

            if self.overlay_rotate_ref_180:
                ref_bgr = cv2.rotate(ref_bgr, cv2.ROTATE_180)

            self.overlay_ref_bgr = ref_bgr
            self.overlay_status_text = (
                f"Overlay ready: {source_desc}, "
                f"alpha={self.overlay_alpha:.2f}"
            )

        except Exception as e:
            self.overlay_ref_bgr = None
            self.overlay_status_text = f"Overlay unavailable: {e}"
            self.get_logger().error(self.overlay_status_text)

    def overlay_available(self) -> bool:
        return self.overlay_ref_bgr is not None

    def compose_overlay_rgb(self, live_bgr: np.ndarray) -> np.ndarray:
        if self.overlay_ref_bgr is None:
            return live_bgr

        if live_bgr.shape[:2] != self.overlay_ref_bgr.shape[:2]:
            ref = cv2.resize(
                self.overlay_ref_bgr,
                (live_bgr.shape[1], live_bgr.shape[0]),
                interpolation=cv2.INTER_AREA,
            )
        else:
            ref = self.overlay_ref_bgr

        return cv2.addWeighted(
            ref,
            self.overlay_alpha,
            live_bgr,
            1.0 - self.overlay_alpha,
            0.0,
        )

    def _event_frame_topic_for_selected_mode(self) -> str:
        if self.selected_event_frame_visualization == "3ch":
            return self.event_frame_3ch_topic
        return self.event_frame_mono_topic

    def _rgb_topic_for_selected_source(self) -> str:
        if self.selected_rgb_source == "debug":
            return self.rgb_debug_topic
        return self.rgb_topic

    def switch_rgb_source_mode(self, mode: str):
        mode = str(mode).strip().lower()
        if mode not in ("raw", "debug"):
            self.get_logger().warning(f"Ignoring invalid RGB source mode: {mode}")
            return

        self.selected_rgb_source = mode
        topic = self._rgb_topic_for_selected_source()

        if self.rgb_sub is not None:
            try:
                self.destroy_subscription(self.rgb_sub)
            except Exception as e:
                self.get_logger().warning(f"Failed to destroy previous RGB subscription: {e}")
            self.rgb_sub = None

        self.rgb_buffer.clear()
        self.rgb_sub = self.create_subscription(
            Image,
            topic,
            self.rgb_cb,
            image_qos,
        )

        self.get_logger().info(
            f"RGB source switched to {mode}; subscribed topic: {topic}"
        )

    def switch_event_frame_visualization_mode(self, mode: str):
        mode = str(mode).strip().lower()
        if mode not in ("mono", "3ch"):
            self.get_logger().warning(f"Ignoring invalid visualization mode: {mode}")
            return

        self.selected_event_frame_visualization = mode
        topic = self._event_frame_topic_for_selected_mode()

        if self.event_frame_subscription is not None:
            try:
                self.destroy_subscription(self.event_frame_subscription)
            except Exception as e:
                self.get_logger().warning(f"Failed to destroy previous event frame subscription: {e}")
            self.event_frame_subscription = None

        self.event_buffer.clear()
        self.event_frame_subscription = self.create_subscription(
            Image,
            topic,
            self.event_cb,
            event_qos,
        )
        self.get_logger().info(
            f"Event frame visualization switched to {mode}; subscribed topic: {topic}"
        )

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
                    detail = str(result.message).strip()
                    reason = (
                        f"Service returned failure: {service_name}"
                        if not detail
                        else f"Service returned failure: {service_name} ({detail})"
                    )
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
            if self.rgb_rotate_180:
                img = cv2.rotate(img, cv2.ROTATE_180)
            self.rgb_buffer.set(img, is_rgb=False)
        except Exception as e:
            self.get_logger().error(f"RGB callback failed: {e}")

    def event_cb(self, msg: Image):
        try:
            img = self.bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
            encoding = str(msg.encoding).strip().lower()

            # FOR L-RGB-EVENT MOUNT; ROTATE EVENT FRAME 90° CCW
            img = cv2.rotate(img,cv2.ROTATE_90_COUNTERCLOCKWISE)


            if img.ndim == 2 or encoding == "mono8":
                disp = pad_to_square_black(img)
                is_rgb = False
            elif img.ndim == 3 and img.shape[2] == 3:
                if encoding == "rgb8":
                    disp = pad_to_square_black(img)
                    is_rgb = True
                else:
                    disp = pad_to_square_black(img)
                    is_rgb = False
            elif img.ndim == 3 and img.shape[2] == 4:
                if encoding == "rgba8":
                    disp = cv2.cvtColor(img, cv2.COLOR_RGBA2RGB)
                    disp = pad_to_square_black(disp)
                    is_rgb = True
                else:
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

    def scene_interception_status_cb(self, msg: String):
        with self.state_lock:
            self.state.scene_interception_status = str(msg.data)
            self.state.scene_status_receive_monotonic = time.monotonic()
        self._update_interception_arm_inhibit_from_state()

    def rollout_interception_status_cb(self, msg: String):
        with self.state_lock:
            self.state.rollout_interception_status = str(msg.data)
            self.state.rollout_status_receive_monotonic = time.monotonic()
        self._update_interception_arm_inhibit_from_state()

    def arm_interception(self):
        with self.state_lock:
            reason = self._arm_safety_block_reason_locked()
            if reason is not None:
                self.state.last_service_status = f"Arm blocked: {reason}"
                return
        self._run_interception_dispatch("arm")

    def disarm_interception(self):
        self._run_interception_dispatch("disarm")

    def reset_episode_counter(self):
        with self.state_lock:
            if self.state.reset_episode_counter_pending:
                return
            self.state.reset_episode_counter_pending = True

        self.get_logger().info("Requested successful episode counter reset")

        def on_success():
            with self.state_lock:
                self.state.reset_episode_counter_pending = False
                self.state.last_service_status = "Successful episode counter reset to 0."
            return self.state.last_service_status

        def on_failure(reason: str):
            with self.state_lock:
                self.state.reset_episode_counter_pending = False
                self.state.last_service_status = f"Episode counter reset failed: {reason}"
            return self.state.last_service_status

        self.call_trigger_service_async(
            self.reset_episode_counter_client,
            self.reset_episode_counter_service,
            on_success,
            on_failure,
        )

    def get_state_snapshot(self) -> TeleopState:
        with self.state_lock:
            s = self.state
            return TeleopState(
                teleop_enabled=s.teleop_enabled,
                recording_active=s.recording_active,
                session_recording_active=s.session_recording_active,
                event_frame_publishing_active=s.event_frame_publishing_active,
                debug_bypass_topic_presence=s.debug_bypass_topic_presence,
                last_service_status=s.last_service_status,
                successful_episodes=s.successful_episodes,
                episode_active=s.episode_active,
                episode_start_monotonic=s.episode_start_monotonic,
                episode_elapsed_frozen=s.episode_elapsed_frozen,
                reset_episode_counter_pending=s.reset_episode_counter_pending,
                interception_arm_mode=s.interception_arm_mode,
                scene_interception_status=s.scene_interception_status,
                rollout_interception_status=s.rollout_interception_status,
                scene_status_receive_monotonic=s.scene_status_receive_monotonic,
                rollout_status_receive_monotonic=s.rollout_status_receive_monotonic,
                execution_mode_transition_pending=s.execution_mode_transition_pending,
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

    def set_debug_bypass_topic_presence(self, enabled: bool):
        timeout_sec = self._get_double_param("service_timeout_sec")
        service_name = self.set_debug_bypass_service

        def worker():
            desired_state = bool(enabled)
            state_text = "ON" if desired_state else "OFF"
            self.get_logger().info(f"Requested debug bypass topic presence: {state_text}")

            try:
                if not self._wait_for_service(self.set_debug_bypass_client, timeout_sec):
                    reason = f"Service not available: {service_name}"
                    self.get_logger().error(reason)
                    self._set_last_service_status(reason)
                    return

                req = SetBool.Request()
                req.data = desired_state
                future = self.set_debug_bypass_client.call_async(req)

                start = time.monotonic()
                while rclpy.ok() and not future.done():
                    if time.monotonic() - start > timeout_sec:
                        reason = f"Service call timed out: {service_name}"
                        self.get_logger().error(reason)
                        self._set_last_service_status(reason)
                        return
                    time.sleep(0.02)

                result = future.result()
                if result is None or not result.success:
                    detail = "" if result is None else str(result.message).strip()
                    reason = (
                        f"Failed to set debug bypass: {service_name}"
                        if not detail
                        else f"Failed to set debug bypass: {service_name} ({detail})"
                    )
                    self.get_logger().error(reason)
                    self._set_last_service_status(reason)
                    return

                with self.state_lock:
                    self.state.debug_bypass_topic_presence = desired_state
                    self.state.last_service_status = f"Debug bypass topic presence {state_text}."

            except Exception as e:
                reason = f"Service call error for {service_name}: {e}"
                self.get_logger().error(reason)
                self._set_last_service_status(reason)

        threading.Thread(target=worker, daemon=True).start()

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

    def capture_line(self):
        timeout_sec = self._get_double_param("service_timeout_sec")

        def worker():
            try:
                self._set_last_service_status("Setting line params...")

                if not self._wait_for_service(self.set_line_params_client, timeout_sec):
                    reason = f"Line params service unavailable: {self.set_line_params_service}"
                    self.get_logger().error(reason)
                    self._set_last_service_status(reason)
                    return

                set_req = SetLineParams.Request()
                set_req.axis_x = 1.0
                set_req.axis_y = 0.0
                set_req.axis_z = 0.0
                set_req.half_length = 0.2
                set_req.safety_min_z = 0.03

                set_future = self.set_line_params_client.call_async(set_req)
                start = time.monotonic()
                while rclpy.ok() and not set_future.done():
                    if time.monotonic() - start > timeout_sec:
                        reason = f"Set line params timed out: {self.set_line_params_service}"
                        self.get_logger().error(reason)
                        self._set_last_service_status(reason)
                        return
                    time.sleep(0.02)

                set_result = set_future.result()
                if set_result is None or not bool(set_result.success):
                    detail = "" if set_result is None else str(set_result.message).strip()
                    reason = (
                        f"Set line params failed: {self.set_line_params_service}"
                        if not detail
                        else f"Set line params failed: {self.set_line_params_service} ({detail})"
                    )
                    self.get_logger().error(reason)
                    self._set_last_service_status(reason)
                    return

                self._set_last_service_status("Line params set. Capturing center...")

                if not self._wait_for_service(self.capture_line_center_client, timeout_sec):
                    reason = (
                        "Capture center service unavailable: "
                        f"{self.capture_line_center_service}"
                    )
                    self.get_logger().error(reason)
                    self._set_last_service_status(reason)
                    return

                capture_req = CaptureLineCenter.Request()
                capture_req.ee_name = self.line_ee_name
                capture_req.keep_current_orientation = True

                capture_future = self.capture_line_center_client.call_async(capture_req)
                start = time.monotonic()
                while rclpy.ok() and not capture_future.done():
                    if time.monotonic() - start > timeout_sec:
                        reason = (
                            "Capture line center timed out: "
                            f"{self.capture_line_center_service}"
                        )
                        self.get_logger().error(reason)
                        self._set_last_service_status(reason)
                        return
                    time.sleep(0.02)

                capture_result = capture_future.result()
                if capture_result is None or not bool(capture_result.success):
                    detail = "" if capture_result is None else str(capture_result.message).strip()
                    reason = (
                        f"Capture line center failed: {self.capture_line_center_service}"
                        if not detail
                        else (
                            "Capture line center failed: "
                            f"{self.capture_line_center_service} ({detail})"
                        )
                    )
                    self.get_logger().error(reason)
                    self._set_last_service_status(reason)
                    return

                self._set_last_service_status("Line center captured.")

            except Exception as e:
                reason = f"Capture line error: {e}"
                self.get_logger().error(reason)
                self._set_last_service_status(reason)

        threading.Thread(target=worker, daemon=True).start()

    def send_line_trajectory_goal(self, command: int, label: str):
        timeout_sec = self._get_double_param("service_timeout_sec")

        def worker():
            try:
                self._set_last_service_status(
                    f"{label}: waiting for action server {self.trajectory_executor_action}..."
                )

                if not self.trajectory_executor_client.wait_for_server(timeout_sec=timeout_sec):
                    reason = f"{label}: action server unavailable: {self.trajectory_executor_action}"
                    self.get_logger().error(reason)
                    self._set_last_service_status(reason)
                    return

                goal = LineTrajectory.Goal()
                goal.command = int(command)
                goal.ee_name = self.line_ee_name
                goal.profile_name = self.line_profile_name
                goal.target_s = 0.0
                goal.v_max = 1.0
                goal.a_max = 1.0
                goal.j_max = 30.0
                goal.hold_before_sec = 0.0
                goal.hold_after_sec = 1.0

                self._set_last_service_status(f"{label}: sending goal...")
                send_future = self.trajectory_executor_client.send_goal_async(goal)

                start = time.monotonic()
                while rclpy.ok() and not send_future.done():
                    if time.monotonic() - start > timeout_sec:
                        reason = f"{label}: send goal timed out"
                        self.get_logger().error(reason)
                        self._set_last_service_status(reason)
                        return
                    time.sleep(0.02)

                goal_handle = send_future.result()
                if goal_handle is None:
                    reason = f"{label}: failed to receive goal response"
                    self.get_logger().error(reason)
                    self._set_last_service_status(reason)
                    return

                if not goal_handle.accepted:
                    reason = f"{label}: goal rejected"
                    self.get_logger().warning(reason)
                    self._set_last_service_status(reason)
                    return

                self._set_last_service_status(f"{label}: goal accepted, waiting for result...")
                result_future = goal_handle.get_result_async()

                start = time.monotonic()
                while rclpy.ok() and not result_future.done():
                    if time.monotonic() - start > timeout_sec:
                        reason = f"{label}: waiting for result timed out"
                        self.get_logger().error(reason)
                        self._set_last_service_status(reason)
                        return
                    time.sleep(0.02)

                result_response = result_future.result()
                if result_response is None:
                    reason = f"{label}: action returned no result"
                    self.get_logger().error(reason)
                    self._set_last_service_status(reason)
                    return

                self._set_last_service_status(
                    f"{label}: result received (status={int(result_response.status)})"
                )

            except Exception as e:
                reason = f"{label}: action error: {e}"
                self.get_logger().error(reason)
                self._set_last_service_status(reason)

        threading.Thread(target=worker, daemon=True).start()


# ----------------------------
# Main window
# ----------------------------

class TeleopDashboardWindow(QMainWindow):
    def __init__(self, node: TeleopDashboardNode, start_fullscreen: bool = True):
        super().__init__()
        self.node = node
        self.use_overlay_rgb = False
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

        self.teleop_episode_card = TeleopEpisodeCard()
        self.rgb_mode_card = RGBModeControlCard(
            on_toggle_overlay=self.toggle_overlay_mode,
            on_select_source=self.switch_rgb_source_mode,
            overlay_available=self.node.overlay_available(),
            initial_overlay=self.use_overlay_rgb,
            initial_source=self.node.selected_rgb_source,
        )
        self.success_card = MetricCard(
            "Successful Episodes",
            "0",
            button_text="Reset Counter",
            on_button_clicked=self.node.reset_episode_counter,
        )
        self.duration_card = MetricCard("Current Episode [s]", "0.0")

        top_row.addWidget(self.teleop_episode_card, 1)
        top_row.addWidget(self.rgb_mode_card, 1)
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
            on_toggle_debug_bypass=self.node.set_debug_bypass_topic_presence,
        )
        self.event_frames_card = EventFramePublishingControlCard(
            on_start=self.node.start_event_frame_publishing,
            on_stop=self.node.stop_event_frame_publishing,
            on_select_visualization=self.switch_event_frame_visualization_mode,
            initial_mode=self.node.selected_event_frame_visualization,
        )
        self.line_motion_card = LineMotionControlCard(
            on_capture=self.node.capture_line,
            on_to_start=lambda: self.node.send_line_trajectory_goal(2, "ToStart"),
            on_to_end=lambda: self.node.send_line_trajectory_goal(3, "ToEnd"),
            on_to_center=lambda: self.node.send_line_trajectory_goal(6, "ToCenter"),
        )
        self.interception_card = InterceptionControlCard(
            on_arm=self.node.arm_interception,
            on_mode_change=self.node.request_interception_arm_mode_change,
            on_execution_mode_change=self.node.request_execution_mode_change,
            initial_mode=self.node.interception_arm_mode,
        )

        self.status_card = QFrame()
        self.status_card.setFrameShape(QFrame.StyledPanel)
        self.status_card.setStyleSheet("""
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

        status_layout = QVBoxLayout()
        status_layout.setContentsMargins(12, 12, 12, 12)
        status_layout.setSpacing(8)

        self.status_label = QLabel("Status: ")
        self.status_label.setAlignment(Qt.AlignLeft | Qt.AlignTop)
        self.status_label.setWordWrap(True)
        self.status_label.setTextFormat(Qt.PlainText)
        self.status_label.setStyleSheet("color: white; font-size: 15px;")
        # Keep label content from changing parent layout size hints.
        self.status_label.setSizePolicy(QSizePolicy.Ignored, QSizePolicy.Ignored)

        status_layout.addWidget(self.status_label, 1)
        self.status_card.setLayout(status_layout)

        # Keep controls usable by preventing status text from dictating row size.
        self.status_card.setFixedSize(420, 240)

        bottom_row.addWidget(self.session_recording_card, 1)
        bottom_row.addWidget(self.line_motion_card, 1)
        bottom_row.addWidget(self.interception_card, 1)
        bottom_row.addWidget(self.event_frames_card, 1)
        bottom_row.addWidget(self.status_card, 0)

        root.addLayout(top_row, 0)
        root.addLayout(image_row, 1)
        root.addLayout(bottom_row, 0)

        self.setStyleSheet("background-color: #1b1b1b;")
        self.resize(1600, 950)

        self._is_fullscreen = False
        self._last_event_frame_publishing_active: Optional[bool] = None

        self.gui_timer = QTimer(self)
        self.gui_timer.timeout.connect(self.refresh_gui)
        self.gui_timer.start(50)  # 20 Hz GUI refresh

        self._update_rgb_tile_title()

        if start_fullscreen:
            self.showFullScreen()
            self._is_fullscreen = True

    def _update_rgb_tile_title(self):
        source = str(self.node.selected_rgb_source).strip().lower()
        overlay = "Overlay" if self.use_overlay_rgb else "Live"
        source_label = "Debug" if source == "debug" else "Raw"
        self.rgb_tile.title_label.setText(f"RGB ({source_label}, {overlay})")

    def toggle_overlay_mode(self):
        if not self.node.overlay_available():
            self.use_overlay_rgb = False
            if hasattr(self, "rgb_mode_card"):
                self.rgb_mode_card.set_overlay_mode(False)
            self._update_rgb_tile_title()
            return

        self.use_overlay_rgb = not self.use_overlay_rgb

        if hasattr(self, "rgb_mode_card"):
            self.rgb_mode_card.set_overlay_mode(self.use_overlay_rgb)

        self._update_rgb_tile_title()

    def switch_rgb_source_mode(self, mode: str):
        self.node.switch_rgb_source_mode(mode)

        if hasattr(self, "rgb_mode_card"):
            self.rgb_mode_card.set_source_mode(self.node.selected_rgb_source)

        self.rgb_tile.clear()
        self._update_rgb_tile_title()

    def switch_event_frame_visualization_mode(self, mode: str):
        self.node.switch_event_frame_visualization_mode(mode)
        self.event_frames_card.set_visualization_mode(self.node.selected_event_frame_visualization)
        self.event_tile.clear()

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
                display_rgb_frame = rgb_frame
                if self.use_overlay_rgb and not rgb_is_rgb:
                    display_rgb_frame = self.node.compose_overlay_rgb(rgb_frame)

                qimg = np_to_qimage(display_rgb_frame, already_rgb=rgb_is_rgb)
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
        self.teleop_episode_card.set_teleop(s.teleop_enabled)
        self.teleop_episode_card.set_episode_active(s.episode_active)
        self.session_recording_card.set_active(s.session_recording_active)
        self.session_recording_card.set_debug_bypass(s.debug_bypass_topic_presence)
        self.event_frames_card.set_active(s.event_frame_publishing_active)
        self.event_frames_card.set_visualization_mode(self.node.selected_event_frame_visualization)
        self.rgb_mode_card.set_overlay_mode(self.use_overlay_rgb)
        self.rgb_mode_card.set_source_mode(self.node.selected_rgb_source)
        execution_mode, arm_allowed, arm_block_reason = self.node.get_interception_execution_snapshot()
        self.interception_card.set_controller_statuses(
            mode=s.interception_arm_mode,
            scene_status=s.scene_interception_status,
            rollout_status=s.rollout_interception_status,
            execution_mode=execution_mode,
            transition_pending=s.execution_mode_transition_pending,
            arm_allowed=arm_allowed,
        )
        self.success_card.value_label.setText(str(s.successful_episodes))
        if self.success_card.action_button is not None:
            self.success_card.action_button.setEnabled(not s.reset_episode_counter_pending)
            if s.reset_episode_counter_pending:
                self.success_card.action_button.setText("Resetting...")
            else:
                self.success_card.action_button.setText(self.success_card.default_button_text)
        overlay_mode = "Overlay" if self.use_overlay_rgb else "Live"
        self.status_label.setText(
            f"Status: {s.last_service_status}\n"
            f"Interception arm mode: {s.interception_arm_mode}\n"
            f"Interception execution mode: {execution_mode_label(execution_mode)}\n"
            f"Execution transition pending: {s.execution_mode_transition_pending}\n"
            f"Arm block reason: {arm_block_reason if arm_block_reason else '-'}\n"
            f"Rollout controller: {s.rollout_interception_status}\n"
        )

        if self._last_event_frame_publishing_active is None:
            self._last_event_frame_publishing_active = s.event_frame_publishing_active
        elif self._last_event_frame_publishing_active and not s.event_frame_publishing_active:
            self.event_tile.clear()
            self.node.event_buffer.clear()
            self._last_event_frame_publishing_active = s.event_frame_publishing_active
        else:
            self._last_event_frame_publishing_active = s.event_frame_publishing_active

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