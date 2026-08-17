"""Canonical registry of ROS 2 topic and service names for fr3_teleop.

This module is the single source of truth for interface names used across
fr3_teleop nodes, launch files, and recording policies.

Behavioral configuration belongs in teleop_config.yaml.
"""

TOPICS = {
    # ------------------------------------------------------------------
    # Episode and data-collection state
    # ------------------------------------------------------------------
    "episode_control": "/episode/control",
    "num_valid_episodes": "/data_collection/num_valid_episodes",

    # ------------------------------------------------------------------
    # RGB and event camera topics
    # ------------------------------------------------------------------
    "camera_info": "/top_cam/camera/color/camera_info",
    "rgb_image": "/top_cam/camera/color/image_raw",

    # Dashboard/legacy compatibility topics
    "rgb_debug_image": "/scene_localizer_debug/top_cam/debug_image",
    "openmv_mono": "/openmv_cam/image",
    "openmv_3ch": "/openmv_cam/event_frame_3ch",

    # Optional visual debug topics
    "rgb_ball_debug_image": "/ball_tracker2/debug_image",
    "rgb_ball_mask": "/ball_tracker2/mask",
    "event_tracker_debug_image":
        "/openmv_cam/event_tracker/debug_image",
    "event_tracker_debug_activity":
        "/openmv_cam/event_tracker/debug/activity",
    "event_tracker_debug_contours":
        "/openmv_cam/event_tracker/debug/contours",
    "event_tracker_debug_event_frame":
        "/openmv_cam/event_tracker/debug/event_frame_33ms",
    "event_tracker_debug_threshold":
        "/openmv_cam/event_tracker/debug/threshold",
    "event_tracker_debug_tracking":
        "/openmv_cam/event_tracker/debug/tracking",

    # ------------------------------------------------------------------
    # Native OpenMV event tracker outputs
    # ------------------------------------------------------------------
    "event_tracker_position":
        "/openmv_cam/event_tracker/ball_2d_px",
    "event_tracker_velocity":
        "/openmv_cam/event_tracker/ball_velocity_px_s",
    "event_tracker_valid":
        "/openmv_cam/event_tracker/valid",

    # ------------------------------------------------------------------
    # RGB ball tracking and scene estimation
    # ------------------------------------------------------------------
    "ball_2d_px":
        "/ball_tracker2/ball_2d_px",
    "ball_3d_camera":
        "/scene_localizer/top_cam/ball_3d_camera",
    "ball_3d_table":
        "/scene_localizer/top_cam/ball_3d_table",
    "ball_trajectory":
        "/scene/ball_trajectory_table",
    "ball_trajectory_marker":
        "/scene/ball_trajectory_marker",
    "middle_line_intersection_pose":
        "/scene/middle_line_intersection_pose_robot_base",

    # Retained for older nodes using the previous interface
    "intercept_pose":
        "/scene/intercept_pose_robot_base",

    # ------------------------------------------------------------------
    # ACT and policy outputs
    # ------------------------------------------------------------------
    "act_intercept_prediction":
        "/act/intercept_prediction",
    "act_intercept_prediction_current_abs_s":
        "/act/intercept_prediction_current_abs_s",
    "act_intercept_prediction_chunk_abs_s":
        "/act/intercept_prediction_chunk_abs_s",

    # ------------------------------------------------------------------
    # Classical scene interception controller
    # ------------------------------------------------------------------
    "scene_interception_status":
        "/interception_controller/status",
    "scene_interception_selected_goto_s":
        "/interception_controller/selected_goto_s",
    "scene_interception_commanded_target_table":
        "/interception_controller/commanded_target_table",

    # ------------------------------------------------------------------
    # Rollout interception controller
    # ------------------------------------------------------------------
    "rollout_interception_status":
        "/rollout_interception_controller/status",
    "rollout_interception_selected_goto_s":
        "/rollout_interception_controller/selected_goto_s",
    "rollout_interception_commanded_target_table":
        "/rollout_interception_controller/commanded_target_table",

    # ------------------------------------------------------------------
    # Continuous tracker
    # ------------------------------------------------------------------
    "cont_tracker_target_s":
        "/cont_tracker/target_s",
    "cont_tracker_track_s":
        "/cont_tracker/track_s",
    "cont_tracker_accepted_target_s":
        "/cont_tracker/accepted_target_s",
    "cont_tracker_accepted_target_base":
        "/cont_tracker/accepted_target_base",
    "current_tcp_s":
        "/middle_line/current_tcp_s",

    # Hidden continuous-tracker action topics
    "cont_tracker_action_feedback":
        "/cont_tracker/_action/feedback",
    "cont_tracker_action_status":
        "/cont_tracker/_action/status",

    # ------------------------------------------------------------------
    # Robot and executor measurements
    # ------------------------------------------------------------------
    "twist_cmd":
        "/cartesian_cmd/twist",
    "joint_states":
        "/joint_states",
    "right_fr3_joint_states":
        "/right_fr3/joint_states",

    "robot_state":
        "/right_franka_robot_state_broadcaster/robot_state",
    "external_wrenches":
        "/right_franka/external_wrenches",

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
    "trajectory_execution_event":
        "/trajectory_execution_event",

    # ------------------------------------------------------------------
    # Teleoperation and arm-state context
    # ------------------------------------------------------------------
    "teleop_control":
        "/teleop/control",
    "interception_arm_mode":
        "/teleop/interception_arm_mode",
    "interception_arm_inhibit":
        "/teleop/interception_arm_inhibit",
    "gripper_state_command":
        "/teleop/gripper_state_cmd",
    "gripper_command":
        "/teleop/gripper_cmd",
    "gripper_inhibit":
        "/teleop/gripper_inhibit",

    "spacemouse_cmd":
        "/spacemouse_cmd",
    "spacemouse_buttons":
        "/spacemouse_buttons",

    # ------------------------------------------------------------------
    # Calibration and geometry provenance
    # ------------------------------------------------------------------
    "aruco_detections":
        "/aruco_top_cam/aruco_detections",
    "aruco_debug":
        "/aruco_top_cam/aruco_top_cam/debug",

    "table_pose_robot_base":
        "/scene_localizer/table_pose_robot_base",
    "camera_pose_robot_base":
        "/scene_localizer/top_cam/camera_pose_robot_base",
    "table_pose_camera":
        "/scene_localizer/top_cam/table_pose_camera",
    "table_pose_frozen":
        "/scene_localizer/table_pose_frozen",

    "tf":
        "/tf",
    "tf_static":
        "/tf_static",

    # ------------------------------------------------------------------
    # Latency and pipeline traces
    # ------------------------------------------------------------------
    "trace_ball_tracker2":
        "/intercept_trace/ball_tracker2",
    "trace_event_2d_detection":
        "/intercept_trace/event_2d_ball_detection",
    "trace_localization_2d_to_3d":
        "/intercept_trace/localization_2d_to_3d",
    "trace_trajectory_estimation":
        "/intercept_trace/trajectory_estimation",
    "trace_controller":
        "/intercept_trace/controller",

    # ------------------------------------------------------------------
    # Diagnostics
    # ------------------------------------------------------------------
    "rosout":
        "/rosout",
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

    "freeze_table_pose":
        "/scene_localizer/freeze_table_pose",
    "reacquire_table_pose":
        "/scene_localizer/reacquire_table_pose",
}


# ---------------------------------------------------------------------------
# Recording policy
# ---------------------------------------------------------------------------

# Default profile for the next base-dataset collection.
#
# Includes:
# - RGB images and RGB ball positions;
# - live event tracker position, velocity, and validity;
# - scene/localization outputs;
# - robot/action sources;
# - controller/executor state;
# - calibration provenance;
# - latency trace topics.
#
# Large debug-image topics are intentionally excluded.
VISIBLE_RECORD_TOPIC_KEYS = [
    # Episode boundaries and collection progress
    "episode_control",
    "num_valid_episodes",

    # RGB observations and RGB tracking
    "camera_info",
    "rgb_image",
    "ball_2d_px",

    # Live event tracker
    "event_tracker_position",
    "event_tracker_velocity",
    "event_tracker_valid",

    # Scene estimation
    "ball_3d_camera",
    "ball_3d_table",
    "ball_trajectory",
    "middle_line_intersection_pose",

    # Classical interception controller
    "scene_interception_status",
    "scene_interception_selected_goto_s",
    "scene_interception_commanded_target_table",

    # Continuous tracker
    "cont_tracker_target_s",
    "cont_tracker_track_s",
    "cont_tracker_accepted_target_s",
    "cont_tracker_accepted_target_base",
    "current_tcp_s",

    # Executor and low-level state
    "trajectory_middle_line_state",
    "trajectory_executor_status",
    "executed_goto_s",
    "executed_goto_s_target_base",
    "cartesian_executor_status",
    "trajectory_execution_event",

    # Robot measurements
    "joint_states",

    # Teleoperation and arming context
    "teleop_control",
    "interception_arm_mode",
    "interception_arm_inhibit",
    "gripper_inhibit",

    # Calibration and geometry provenance
    "aruco_detections",
    "table_pose_frozen",
    "table_pose_robot_base",
    "camera_pose_robot_base",
    "table_pose_camera",
    "tf",
    "tf_static",

    # Pipeline latency traces
    "trace_ball_tracker2",
    "trace_event_2d_detection",
    "trace_localization_2d_to_3d",
    "trace_trajectory_estimation",
    "trace_controller",

    # Diagnostics
    "rosout",
]


# These are useful for future rollout/debug recordings but are not required
# for the no-rollout base-dataset profile.
OPTIONAL_ROLLOUT_RECORD_TOPIC_KEYS = [
    "act_intercept_prediction",
    "act_intercept_prediction_current_abs_s",
    "act_intercept_prediction_chunk_abs_s",

    "rollout_interception_status",
    "rollout_interception_selected_goto_s",
    "rollout_interception_commanded_target_table",
]


# Large debug or diagnostic topics. Add these explicitly only for a
# dedicated debugging recording.
OPTIONAL_DEBUG_RECORD_TOPIC_KEYS = [
    "rgb_ball_debug_image",
    "rgb_ball_mask",

    "event_tracker_debug_image",
    "event_tracker_debug_activity",
    "event_tracker_debug_contours",
    "event_tracker_debug_event_frame",
    "event_tracker_debug_threshold",
    "event_tracker_debug_tracking",

    "openmv_mono",
    "openmv_3ch",

    "right_fr3_joint_states",
    "robot_state",
    "external_wrenches",

    "ball_trajectory_marker",
    "aruco_debug",
]


# Hidden action topics are included only when available.
OPTIONAL_HIDDEN_RECORD_TOPIC_KEYS = [
    "cont_tracker_action_feedback",
    "cont_tracker_action_status",
]


# The default recording profile deliberately excludes rollout and debug topics.
RECORD_TOPIC_KEYS = (
    VISIBLE_RECORD_TOPIC_KEYS
    + OPTIONAL_HIDDEN_RECORD_TOPIC_KEYS
)


TOPICS_TO_RECORD = [
    TOPICS[key]
    for key in RECORD_TOPIC_KEYS
]


# Topics whose absence prevents starting a base recording.
#
# The live event tracker topics are intentionally not required here:
# raw-event HDF5 recording plus offline OpenMV tracking is the authoritative
# event pipeline. Their presence is still recorded whenever available.
REQUIRED_RECORD_TOPIC_KEYS = [
    # Episode structure
    "episode_control",

    # Training observations
    "camera_info",
    "rgb_image",
    "joint_states",
    "current_tcp_s",

    # RGB 2D source
    "ball_2d_px",

    # Scene/action context
    "ball_3d_table",
    "ball_trajectory",
    "middle_line_intersection_pose",

    # Continuous tracking
    "cont_tracker_target_s",
    "cont_tracker_track_s",

    # Executor provenance
    "trajectory_middle_line_state",
    "executed_goto_s_target_base",
]


REQUIRED_TOPICS_TO_RECORD = [
    TOPICS[key]
    for key in REQUIRED_RECORD_TOPIC_KEYS
]


# ---------------------------------------------------------------------------
# Per-node parameter dictionaries
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
    "rgb_topic":
        TOPICS["rgb_image"],
    "rgb_debug_topic":
        TOPICS["rgb_debug_image"],
    "event_frame_mono_topic":
        TOPICS["openmv_mono"],
    "event_frame_3ch_topic":
        TOPICS["openmv_3ch"],
    "episode_control_topic":
        TOPICS["episode_control"],
    "teleop_control_topic":
        TOPICS["teleop_control"],
    "num_valid_episodes_topic":
        TOPICS["num_valid_episodes"],
    "table_pose_frozen_topic":
        TOPICS["table_pose_frozen"],

    "start_recording_service":
        SERVICES["start_recording"],
    "stop_recording_service":
        SERVICES["stop_recording"],
    "set_debug_bypass_service":
        SERVICES["set_debug_bypass"],
    "reset_episode_counter_service":
        SERVICES["reset_episode_counter"],
    "freeze_table_pose_service":
        SERVICES["freeze_table_pose"],
    "reacquire_table_pose_service":
        SERVICES["reacquire_table_pose"],

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

    "twist_topic_name":
        TOPICS["twist_cmd"],
    "teleop_control_topic":
        TOPICS["teleop_control"],
    "gripper_state_command_topic":
        TOPICS["gripper_state_command"],
    "spacemouse_topic_name":
        TOPICS["spacemouse_cmd"],
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
    "robot_state_topic":
        TOPICS["robot_state"],
    "output_topic":
        TOPICS["external_wrenches"],
}