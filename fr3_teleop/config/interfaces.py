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
    # Episode/data collection
    "episode_control": "/episode/control",
    "num_valid_episodes": "/data_collection/num_valid_episodes",

    # Camera inputs
    "camera_info": "/top_cam/camera/color/camera_info",
    "rgb_image": "/top_cam/camera/color/image_raw",
    "rgb_debug_image": "/scene_localizer_debug/top_cam/debug_image",
    "openmv_mono": "/openmv_cam/image",
    "openmv_3ch": "/openmv_cam/event_frame_3ch",

    # Robot commands and measurements
    "twist_cmd": "/cartesian_cmd/twist",
    "joint_states": "/joint_states",
    "robot_state": "/right_franka_robot_state_broadcaster/robot_state",
    "external_wrenches": "/right_franka/external_wrenches",

    # Gripper
    "gripper_state_command": "/teleop/gripper_state_cmd",
    "gripper_command": "/teleop/gripper_cmd",
    "gripper_inhibit": "/teleop/gripper_inhibit",

    # Teleoperation
    "teleop_control": "/teleop/control",
    "spacemouse_cmd": "/spacemouse_cmd",
    "spacemouse_buttons": "/spacemouse_buttons",

    # ACT policy
    "act_intercept_prediction": "/act/intercept_prediction",

    # Scene estimation
    "ball_2d_px": "/ball_tracker2/ball_2d_px",
    "ball_3d_table": "/scene_localizer/top_cam/ball_3d_table",
    "ball_trajectory": "/scene/ball_trajectory_table",
    "middle_line_intersection_pose":
        "/scene/middle_line_intersection_pose_robot_base",

    # Retained for nodes still using the older interface.
    # This is not part of the updated recording list.
    "intercept_pose": "/scene/intercept_pose_robot_base",

    # Scene interception controller (SIC)
    "scene_interception_status": "/interception_controller/status",
    "scene_interception_selected_goto_s":
        "/interception_controller/selected_goto_s",
    "scene_interception_commanded_target_table":
        "/interception_controller/commanded_target_table",

    # Rollout interception controller (RIC)
    "rollout_interception_status":
        "/rollout_interception_controller/status",
    "rollout_interception_selected_goto_s":
        "/rollout_interception_controller/selected_goto_s",
    "rollout_interception_commanded_target_table":
        "/rollout_interception_controller/commanded_target_table",

    # Continuous tracker
    "cont_tracker_target_s": "/cont_tracker/target_s",
    "cont_tracker_track_s": "/cont_tracker/track_s",
    "cont_tracker_accepted_target_s":
        "/cont_tracker/accepted_target_s",
    "cont_tracker_accepted_target_base":
        "/cont_tracker/accepted_target_base",
    "current_tcp_s": "/middle_line/current_tcp_s",

    # Hidden continuous-tracker action topics
    "cont_tracker_action_feedback":
        "/cont_tracker/_action/feedback",
    "cont_tracker_action_status":
        "/cont_tracker/_action/status",

    # Executor/controller state
    "trajectory_middle_line_state":
        "/trajectory_executor/middle_line_state",
    "trajectory_executor_status":
        "/trajectory_executor/status",
    "executed_goto_s":
        "/trajectory_executor/executed_goto_s",
    "executed_goto_s_target_base":
        "/trajectory_executor/executed_goto_s_target_base",
    "cartesian_executor_status":
        "/cartesian_executor/status",

    # Experimental configuration
    "interception_arm_mode":
        "/teleop/interception_arm_mode",
    "interception_arm_inhibit":
        "/teleop/interception_arm_inhibit",

    # Coordinate transforms
    "table_pose_robot_base":
        "/scene_localizer/table_pose_robot_base",
    "camera_pose_robot_base":
        "/scene_localizer/top_cam/camera_pose_robot_base",
    "table_pose_frozen": "/scene_localizer/table_pose_frozen",
    "tf": "/tf",
    "tf_static": "/tf_static",

    # ROS diagnostics
    "rosout": "/rosout",
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

    "scene_interception_arm":
        "/interception_controller/arm",
    "scene_interception_disarm":
        "/interception_controller/disarm",
    "scene_interception_set_dry_run":
        "/interception_controller/set_dry_run",
    "rollout_interception_arm":
        "/rollout_interception_controller/arm",
    "rollout_interception_disarm":
        "/rollout_interception_controller/disarm",
    "rollout_interception_set_dry_run":
        "/rollout_interception_controller/set_dry_run",
    "freeze_table_pose": "/scene_localizer/freeze_table_pose",
    "reacquire_table_pose": "/scene_localizer/reacquire_table_pose",
}

# ---------------------------------------------------------------------------
# Recording policy (which topics get recorded, and which must be present)
# ---------------------------------------------------------------------------

VISIBLE_RECORD_TOPIC_KEYS = [
    # Episode boundaries
    "episode_control",

    # ACT policy output
    "act_intercept_prediction",

    # Classical scene estimation
    "ball_2d_px",
    "ball_3d_table",
    "ball_trajectory",
    "middle_line_intersection_pose",

    # Scene interception controller
    "scene_interception_status",
    "scene_interception_selected_goto_s",
    "scene_interception_commanded_target_table",

    # Rollout interception controller
    "rollout_interception_status",
    "rollout_interception_selected_goto_s",
    "rollout_interception_commanded_target_table",

    # Continuous tracker
    "cont_tracker_target_s",
    "cont_tracker_track_s",
    "cont_tracker_accepted_target_s",
    "cont_tracker_accepted_target_base",
    "current_tcp_s",

    # Executor/controller state
    "trajectory_middle_line_state",
    "executed_goto_s",
    "executed_goto_s_target_base",
    "cartesian_executor_status",
    "twist_cmd",

    # Robot measurements
    "joint_states",

    # Experimental configuration
    "interception_arm_mode",
    "interception_arm_inhibit",

    # Coordinate transforms
    "table_pose_robot_base",
    "camera_pose_robot_base",
    "tf",
    "tf_static",

    # Diagnostics
    "rosout",

    # Camera images: enabled for the main MVP recording profile
    "rgb_image",
    "openmv_mono",
    "camera_info",
]

# These are useful but must never prevent recording if the action server is
# temporarily absent. The recorder should include them only when available if
# it supports the same discovery behavior as record_sic_vs_ric.sh.
OPTIONAL_HIDDEN_RECORD_TOPIC_KEYS = [
    "cont_tracker_action_feedback",
    "cont_tracker_action_status",
]

RECORD_TOPIC_KEYS = (
    VISIBLE_RECORD_TOPIC_KEYS
    + OPTIONAL_HIDDEN_RECORD_TOPIC_KEYS
)

TOPICS_TO_RECORD = [
    TOPICS[key] for key in RECORD_TOPIC_KEYS
]

REQUIRED_RECORD_TOPIC_KEYS = [
    # Episode structure
    "episode_control",

    # Training observations
    "camera_info",
    "rgb_image",
    "openmv_mono",
    "joint_states",

    # Policy and scene outputs
    "ball_2d_px",
    "ball_3d_table",
    "ball_trajectory",
    "middle_line_intersection_pose",

    # SIC-versus-RIC comparison
    "scene_interception_selected_goto_s",
    "interception_arm_mode",

    # Continuous-tracking command, reference, and measurement
    "cont_tracker_target_s",
    "cont_tracker_track_s",
    "current_tcp_s",

    # Geometry and discrete-executor cross-check
    "trajectory_middle_line_state",
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
    "table_pose_frozen_topic": TOPICS["table_pose_frozen"],

    "start_recording_service":
        SERVICES["start_recording"],
    "stop_recording_service":
        SERVICES["stop_recording"],
    "set_debug_bypass_service":
        SERVICES["set_debug_bypass"],
    "reset_episode_counter_service":
        SERVICES["reset_episode_counter"],
    "freeze_table_pose_service": SERVICES["freeze_table_pose"],
    "reacquire_table_pose_service": SERVICES["reacquire_table_pose"],

    "start_event_frames_service":
        SERVICES["start_event_frames"],
    "stop_event_frames_service":
        SERVICES["stop_event_frames"],
    "start_raw_event_recording_service":
        SERVICES["start_raw_event_recording"],
    "stop_raw_event_recording_service":
        SERVICES["stop_raw_event_recording"],

    "scene_interception_arm_service":
        SERVICES["scene_interception_arm"],
    "scene_interception_disarm_service":
        SERVICES["scene_interception_disarm"],
    "scene_interception_set_dry_run_service":
        SERVICES["scene_interception_set_dry_run"],
    "scene_interception_status_topic":
        TOPICS["scene_interception_status"],
    "rollout_interception_arm_service":
        SERVICES["rollout_interception_arm"],
    "rollout_interception_disarm_service":
        SERVICES["rollout_interception_disarm"],
    "rollout_interception_set_dry_run_service":
        SERVICES["rollout_interception_set_dry_run"],
    "rollout_interception_status_topic":
        TOPICS["rollout_interception_status"],
    "interception_arm_mode": "scene",
    "interception_arm_mode_topic":
        TOPICS["interception_arm_mode"],
    "interception_arm_inhibit_topic":
        TOPICS["interception_arm_inhibit"],
}

PS4_INTERFACE_PARAMS = {
    "reset_episode_counter_service":
        SERVICES["reset_episode_counter"],
    "scene_interception_arm_service":
        SERVICES["scene_interception_arm"],
    "scene_interception_disarm_service":
        SERVICES["scene_interception_disarm"],
    "rollout_interception_arm_service":
        SERVICES["rollout_interception_arm"],
    "rollout_interception_disarm_service":
        SERVICES["rollout_interception_disarm"],
    "interception_arm_mode_topic":
        TOPICS["interception_arm_mode"],
    "interception_arm_inhibit_topic":
        TOPICS["interception_arm_inhibit"],

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
