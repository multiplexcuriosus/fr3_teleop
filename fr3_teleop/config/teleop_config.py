# Topics
EPISODE_CONTROL_TOPIC = "/episode/control"
CAMERA_INFO_TOPIC = "/camera/camera/color/camera_info"
TWIST_CMD_TOPIC = "/cartesian_cmd/twist"
GRIPPER_STATE_COMMAND_TOPIC = "/teleop/gripper_state_cmd"
GRIPPER_COMMAND_TOPIC = "/teleop/gripper_cmd"
RGB_IMAGE_TOPIC = "/camera/camera/color/image_raw"
OPENMV_MONO_TOPIC = "/openmv_cam/image"
OPENMV_3CH_TOPIC = "/openmv_cam/event_frame_3ch"
JOINT_STATES_TOPIC = "/joint_states"
TELEOP_CONTROL_TOPIC = "/teleop/control"

# Services
START_RECORDING_SERVICE = "/record_manager/start_recording"
STOP_RECORDING_SERVICE = "/record_manager/stop_recording"
SET_DEBUG_BYPASS_SERVICE = "/record_manager/set_debug_bypass_topic_presence"

# Recording topic selection
TOPICS_TO_RECORD = [
    EPISODE_CONTROL_TOPIC,
    CAMERA_INFO_TOPIC,
    TWIST_CMD_TOPIC,
    GRIPPER_STATE_COMMAND_TOPIC,
    RGB_IMAGE_TOPIC,
    #OPENMV_MONO_TOPIC,
    OPENMV_3CH_TOPIC,
    JOINT_STATES_TOPIC,
]

OPENMV_PARAMS = {
    "port": "/dev/openmvcam",
    "baud": 115200,
    "publish_fps": 30.0,
    "topic": OPENMV_MONO_TOPIC,
    "publish_3_channel_img": True,
    "topic_3_channel": OPENMV_3CH_TOPIC,
}

DASHBOARD_PARAMS = {
    "event_frame_mono_topic": OPENMV_MONO_TOPIC,
    "event_frame_3ch_topic": OPENMV_3CH_TOPIC,
    "episode_control_topic": EPISODE_CONTROL_TOPIC,
    "start_recording_service": START_RECORDING_SERVICE,
    "stop_recording_service": STOP_RECORDING_SERVICE,
    "set_debug_bypass_service": SET_DEBUG_BYPASS_SERVICE,
}

PS4_PARAMS = {
    "button_gripper_open": 2,
    "button_gripper_close": 0,
    "axis_episode": 6,
    "button_home": 1,
    "axis_teleop": 7,
    "left_stick_x_idx": 0,
    "left_stick_y_idx": 1,
    "right_stick_y_idx": 4,
    "deadzone": 0.08,
    "max_vx": 0.08,
    "max_vy": 0.08,
    "max_vz": 0.05,
    "haptic_pos_multiplier": 2.0,
    "haptic_lin_vel_multiplier": 1.0,
    "haptic_ori_multiplier": 1.0,
    "haptic_ang_vel_multiplier": 1.0,
    "twist_topic_name": TWIST_CMD_TOPIC,
    "twist_frame_id": "base_link",
    "teleop_control_topic": TELEOP_CONTROL_TOPIC,
    "teleop_action_name": "/cartesian_executor",
    "teleop_mode": 0,
    "teleop_ee_name": "right_fr3_hand_tcp",
    "teleop_move_orientation": False,
    "gripper_state_command_topic": GRIPPER_STATE_COMMAND_TOPIC,
    "home_action_name": "/fr3_move_to_joint",
    "home_joint_names": [
        "right_fr3_joint1",
        "right_fr3_joint2",
        "right_fr3_joint3",
        "right_fr3_joint4",
        "right_fr3_joint5",
        "right_fr3_joint6",
        "right_fr3_joint7",
    ],
    "home_joint_positions": [
        -0.00,
        -0.17894121136367586,
        0.0,
        -2.7817434397297225,
        0.00,
        2.62409615716241,
        0.7714418627146631,
    ],
    "home_vel_scale": 0.1,
    "home_acc_scale": 0.1,
}

GRIPPER_PARAMS = {
    "gripper_state_command_topic": GRIPPER_STATE_COMMAND_TOPIC,
    "gripper_command_topic": GRIPPER_COMMAND_TOPIC,
    "gripper_action_name": "/right_franka_gripper/move",
    "gripper_speed": 0.05,
    "gripper_open_width": 0.080,
    "gripper_close_width": 0.06,
    "min_command_interval_sec": 0.5,
}
