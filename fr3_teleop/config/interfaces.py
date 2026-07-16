"""Canonical registry of ROS 2 topic and service names for fr3_teleop.

This module is the single source of truth for interface (topic/service)
names used across the fr3_teleop nodes and their launch files. Behavioral
configuration (rates, gains, button indices, thresholds, motion limits,
etc.) belongs in teleop_config.yaml, not here.

Nodes should NOT import this module directly. The launch file is
responsible for reading these names and injecting them as ROS parameters
so that nodes remain configurable purely through their parameter
interface.
"""

TOPICS = {
    "episode_control": "/episode/control",
    "num_valid_episodes": "/data_collection/num_valid_episodes",

    "camera_info": "/top_cam/camera/color/camera_info",
    "rgb_image": "/top_cam/camera/color/image_raw",
    "rgb_debug_image": "/scene_localizer_debug/top_cam/debug_image",

    "openmv_mono": "/openmv_cam/image",
    "openmv_3ch": "/openmv_cam/event_frame_3ch",

    "twist_cmd": "/cartesian_cmd/twist",
    "joint_states": "/joint_states",
    "robot_state": "/right_franka_robot_state_broadcaster/robot_state",
    "external_wrenches": "/right_franka/external_wrenches",

    "gripper_state_command": "/teleop/gripper_state_cmd",
    "gripper_command": "/teleop/gripper_cmd",
    "gripper_inhibit": "/teleop/gripper_inhibit",

    "teleop_control": "/teleop/control",
    "spacemouse_cmd": "/spacemouse_cmd",
    "spacemouse_buttons": "/spacemouse_buttons",

    "ball_2d_px": "/ball_tracker2/ball_2d_px",
    "ball_3d_table": "/scene_localizer/top_cam/ball_3d_table",
    "ball_trajectory": "/scene/ball_trajectory_table",
    "intercept_pose": "/scene/intercept_pose_robot_base",
    "interception_status": "/interception_controller/status",
    "trajectory_executor_status": "/trajectory_executor/status",
    "executed_goto_s": "/trajectory_executor/executed_goto_s",
    "executed_goto_s_target_base":
        "/trajectory_executor/executed_goto_s_target_base",
}

SERVICES = {
    "start_recording": "/record_manager/start_recording",
    "stop_recording": "/record_manager/stop_recording",
    "set_debug_bypass":
        "/record_manager/set_debug_bypass_topic_presence",

    "reset_episode_counter":
        "/data_collection/reset_episode_counter",

    "start_event_frames":
        "/openmv_cam/start_event_frame_publishing",
    "stop_event_frames":
        "/openmv_cam/stop_event_frame_publishing",
    "start_raw_event_recording":
        "/openmv_cam/start_raw_event_recording",
    "stop_raw_event_recording":
        "/openmv_cam/stop_raw_event_recording",

    "interception_arm":
        "/interception_controller/arm",
    "interception_disarm":
        "/interception_controller/disarm",
}

# ---------------------------------------------------------------------------
# Recording policy (which topics get recorded, and which must be present)
# ---------------------------------------------------------------------------

RECORD_TOPIC_KEYS = [
    "episode_control",
    "camera_info",
    "rgb_image",
    "twist_cmd",
    "gripper_state_command",
    "joint_states",
    "ball_2d_px",
    "ball_3d_table",
    "ball_trajectory",
    "intercept_pose",
    "interception_status",
    "trajectory_executor_status",
    "executed_goto_s_target_base",
    "executed_goto_s"
]

TOPICS_TO_RECORD = [
    TOPICS[key] for key in RECORD_TOPIC_KEYS
]

REQUIRED_RECORD_TOPIC_KEYS = [
    "episode_control",
    "camera_info",
    "rgb_image",
    "joint_states",
    "ball_2d_px",
    "ball_3d_table",
    "executed_goto_s_target_base",
]

REQUIRED_TOPICS_TO_RECORD = [
    TOPICS[key] for key in REQUIRED_RECORD_TOPIC_KEYS
]

# ---------------------------------------------------------------------------
# Per-node parameter dictionaries, injected by the launch file
# ---------------------------------------------------------------------------

RECORD_MANAGER_INTERFACE_PARAMS = {
    "topics_to_record": TOPICS_TO_RECORD,
    "required_topics": REQUIRED_TOPICS_TO_RECORD,
    "openmv_start_service":
        SERVICES["start_raw_event_recording"],
    "openmv_stop_service":
        SERVICES["stop_raw_event_recording"],
}

DASHBOARD_INTERFACE_PARAMS = {
    "rgb_topic": TOPICS["rgb_image"],
    "rgb_debug_topic": TOPICS["rgb_debug_image"],
    "event_frame_mono_topic": TOPICS["openmv_mono"],
    "event_frame_3ch_topic": TOPICS["openmv_3ch"],
    "episode_control_topic": TOPICS["episode_control"],
    "teleop_control_topic": TOPICS["teleop_control"],
    "num_valid_episodes_topic":
        TOPICS["num_valid_episodes"],

    "start_recording_service":
        SERVICES["start_recording"],
    "stop_recording_service":
        SERVICES["stop_recording"],
    "set_debug_bypass_service":
        SERVICES["set_debug_bypass"],
    "reset_episode_counter_service":
        SERVICES["reset_episode_counter"],

    "start_event_frames_service":
        SERVICES["start_event_frames"],
    "stop_event_frames_service":
        SERVICES["stop_event_frames"],
    "start_raw_event_recording_service":
        SERVICES["start_raw_event_recording"],
    "stop_raw_event_recording_service":
        SERVICES["stop_raw_event_recording"],
}

PS4_INTERFACE_PARAMS = {
    "reset_episode_counter_service":
        SERVICES["reset_episode_counter"],
    "interception_arm_service":
        SERVICES["interception_arm"],
    "interception_disarm_service":
        SERVICES["interception_disarm"],

    "twist_topic_name": TOPICS["twist_cmd"],
    "teleop_control_topic": TOPICS["teleop_control"],
    "gripper_state_command_topic":
        TOPICS["gripper_state_command"],
    "spacemouse_topic_name": TOPICS["spacemouse_cmd"],
    "spacemouse_button_topic_name":
        TOPICS["spacemouse_buttons"],
}

GRIPPER_INTERFACE_PARAMS = {
    "gripper_state_command_topic":
        TOPICS["gripper_state_command"],
    "gripper_command_topic":
        TOPICS["gripper_command"],
    "gripper_inhibit_topic":
        TOPICS["gripper_inhibit"],
    "teleop_control_topic":
        TOPICS["teleop_control"],
}

WRENCH_INTERFACE_PARAMS = {
    "robot_state_topic": TOPICS["robot_state"],
    "output_topic": TOPICS["external_wrenches"],
}
