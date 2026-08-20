from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import UnlessCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from ament_index_python.packages import get_package_share_directory
import os

from fr3_teleop.config.interfaces import TOPICS
from fr3_teleop.config.teleop_config import OPENMV_PARAMS


def generate_launch_description():
    fr3_teleop_pkg_share = get_package_share_directory("fr3_teleop")
    openmv_pkg_share = get_package_share_directory("openmv_cam")
    openmv_launch_path = os.path.join(openmv_pkg_share, "launch", "openmv.launch.py")
    scene_localizer_launch_path = os.path.join(
        get_package_share_directory("scene_localizer"),
        "launch",
        "all_scene_localizer_nodes.launch.py"
    )

    realsense_launch_path = os.path.join(
        get_package_share_directory("realsense2_camera"),
        "launch",
        "rs_launch.py"
    )
    realsense_config_path = os.path.join(
        fr3_teleop_pkg_share,
        "config",
        "realsense_no_theora.yaml",
    )

    event_frame_ch0_ms_arg = DeclareLaunchArgument(
        "event_frame_ch0_ms",
        default_value=str(OPENMV_PARAMS.get("event_frame_ch0_ms", 50.0)),
    )
    event_frame_ch1_ms_arg = DeclareLaunchArgument(
        "event_frame_ch1_ms",
        default_value=str(OPENMV_PARAMS.get("event_frame_ch1_ms", 250.0)),
    )
    event_frame_ch2_ms_arg = DeclareLaunchArgument(
        "event_frame_ch2_ms",
        default_value=str(OPENMV_PARAMS.get("event_frame_ch2_ms", 1000.0)),
    )
    event_output_mode_arg = DeclareLaunchArgument(
        "event_output_mode",
        default_value="legacy_flags",
    )
    event_wire_mode_arg = DeclareLaunchArgument(
        "event_wire_mode",
        default_value="processed_evt1",
    )
    event_voxel_bin_ms_arg = DeclareLaunchArgument(
        "event_voxel_bin_ms",
        default_value="1.0",
    )
    event_voxel_temporal_bins_arg = DeclareLaunchArgument(
        "event_voxel_temporal_bins",
        default_value="9",
    )
    event_voxel_publish_fps_arg = DeclareLaunchArgument(
        "event_voxel_publish_fps",
        default_value="30.0",
    )
    event_diagnostics_enabled_arg = DeclareLaunchArgument(
        "event_diagnostics_enabled",
        default_value="false",
    )

    event_frame_ch0_ms = LaunchConfiguration("event_frame_ch0_ms")
    event_frame_ch1_ms = LaunchConfiguration("event_frame_ch1_ms")
    event_frame_ch2_ms = LaunchConfiguration("event_frame_ch2_ms")
    event_output_mode = LaunchConfiguration("event_output_mode")
    event_wire_mode = LaunchConfiguration("event_wire_mode")
    event_voxel_bin_ms = LaunchConfiguration("event_voxel_bin_ms")
    event_voxel_temporal_bins = LaunchConfiguration("event_voxel_temporal_bins")
    event_voxel_publish_fps = LaunchConfiguration("event_voxel_publish_fps")
    event_diagnostics_enabled = LaunchConfiguration("event_diagnostics_enabled")

    openmv_publish_mono_image_arg = DeclareLaunchArgument(
        "openmv_publish_mono_image",
        default_value="false",
        description="Publish the OpenMV mono event image on /openmv_cam/image",
    )
    openmv_publish_mono_image = LaunchConfiguration(
        "openmv_publish_mono_image"
    )

    event_tracker_debug_enabled_arg = DeclareLaunchArgument(
        "event_tracker_debug_enabled",
        default_value="true",
    )
    event_tracker_debug_clip_count_arg = DeclareLaunchArgument(
        "event_tracker_debug_clip_count",
        default_value="2",
    )
    event_tracker_debug_enabled = LaunchConfiguration(
        "event_tracker_debug_enabled"
    )
    event_tracker_debug_clip_count = LaunchConfiguration(
        "event_tracker_debug_clip_count"
    )

    enable_latency_trace_arg = DeclareLaunchArgument(
        "enable_latency_trace",
        default_value="false",
        description="Enable latency tracing in the vision pipeline",
    )
    enable_latency_trace = LaunchConfiguration("enable_latency_trace")

    ball_tracker_publish_mask_arg = DeclareLaunchArgument(
        "ball_tracker_publish_mask",
        default_value="false",
        description="Publish the ball tracker binary mask image",
    )
    ball_tracker_publish_mask = LaunchConfiguration("ball_tracker_publish_mask")

    aruco_top_cam_publish_debug_arg = DeclareLaunchArgument(
        "aruco_top_cam_publish_debug",
        default_value="false",
        description="Expose the top-camera ArUco debug image topic",
    )
    aruco_top_cam_publish_debug = LaunchConfiguration(
        "aruco_top_cam_publish_debug"
    )

    inhibit_scene_localizer_arg = DeclareLaunchArgument(
        "inhibit_scene_localizer",
        default_value="false",
    )
    inhibit_scene_localizer_debug_arg = DeclareLaunchArgument(
        "inhibit_scene_localizer_debug",
        default_value="false",
    )
    inhibit_ball_3d_pose_estimator_arg = DeclareLaunchArgument(
        "inhibit_ball_3d_pose_estimator",
        default_value="false",
    )
    inhibit_ball_trajectory_estimator_arg = DeclareLaunchArgument(
        "inhibit_ball_trajectory_estimator",
        default_value="false",
    )

    inhibit_scene_localizer = LaunchConfiguration("inhibit_scene_localizer")
    inhibit_scene_localizer_debug = LaunchConfiguration("inhibit_scene_localizer_debug")
    inhibit_ball_3d_pose_estimator = LaunchConfiguration("inhibit_ball_3d_pose_estimator")
    inhibit_ball_trajectory_estimator = LaunchConfiguration("inhibit_ball_trajectory_estimator")

    openmv_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(openmv_launch_path),
        launch_arguments={
            "event_frame_ch0_ms": event_frame_ch0_ms,
            "event_frame_ch1_ms": event_frame_ch1_ms,
            "event_frame_ch2_ms": event_frame_ch2_ms,
            "event_output_mode": event_output_mode,
            "event_wire_mode": event_wire_mode,
            "event_voxel_bin_ms": event_voxel_bin_ms,
            "event_voxel_temporal_bins": event_voxel_temporal_bins,
            "event_voxel_publish_fps": event_voxel_publish_fps,
            "event_diagnostics_enabled": event_diagnostics_enabled,
            "publish_mono_img": openmv_publish_mono_image,
            "event_tracker_debug_enabled": event_tracker_debug_enabled,
            "event_tracker_debug_clip_count": event_tracker_debug_clip_count,
            "publish_latency_traces": enable_latency_trace,
        }.items(),
    )

    ball_tracker_node = Node(
        package="ball_tracker",
        executable="ball_tracker2",
        name="ball_tracker2",
        output="screen",
        parameters=[
            {
                "input_image_topic": TOPICS["rgb_image"],
                "enable_latency_trace": enable_latency_trace,
                "publish_mask": ball_tracker_publish_mask,
            }
        ]
    )

    realsense_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(realsense_launch_path),
        launch_arguments={
            "serial_no": "_243722074377", # lab: _243722074377, meine: _017322074405
            "camera_namespace": "top_cam",
            "camera_name": "camera",
            "config_file": realsense_config_path,

            "enable_color": "true",
            "rgb_camera.color_profile": "1280x720x30",

            "enable_depth": "false",
            "enable_infra": "false",
            "enable_infra1": "false",
            "enable_infra2": "false",
            "enable_gyro": "false",
            "enable_accel": "false",

            "pointcloud.enable": "false",
            "align_depth.enable": "false",
            # "rgb_camera.enable_auto_exposure": "false",
            # "rgb_camera.exposure": "3000",
            # "rgb_camera.gain": "64",
            # "rgb_camera.auto_exposure_priority": "false",
            # "rgb_camera.enable_auto_white_balance": "false",
            # "rgb_camera.white_balance": "4500",
        }.items(),
    )

    scene_localizer_nodes = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(scene_localizer_launch_path),
        launch_arguments={
            "inhibit_scene_localizer": inhibit_scene_localizer,
            "inhibit_scene_localizer_debug": inhibit_scene_localizer_debug,
            "inhibit_ball_3d_pose_estimator": inhibit_ball_3d_pose_estimator,
            "inhibit_ball_trajectory_estimator": inhibit_ball_trajectory_estimator,
            "enable_latency_trace": enable_latency_trace,
        }.items(),
    )

    top_aruco = Node(
        package='aruco_opencv',
        executable='aruco_tracker_autostart',
        name='aruco_top_cam',
        namespace='/aruco_top_cam',
        output='screen',
        parameters=[{
            'cam_base_topic': TOPICS["rgb_image"],
            'marker_size': 0.0982,
            'marker_dict': '6X6_50'
        }],
        remappings=[
            (
                '~/debug',
                PythonExpression([
                    "'/aruco_top_cam/aruco_top_cam/debug' if '",
                    aruco_top_cam_publish_debug,
                    "'.lower() in ('true', '1', 'yes', 'on') else "
                    "'/_disabled/aruco_top_cam/debug'",
                ]),
            )
        ],
    )


    return LaunchDescription([
        event_frame_ch0_ms_arg,
        event_frame_ch1_ms_arg,
        event_frame_ch2_ms_arg,
        event_output_mode_arg,
        event_wire_mode_arg,
        event_voxel_bin_ms_arg,
        event_voxel_temporal_bins_arg,
        event_voxel_publish_fps_arg,
        event_diagnostics_enabled_arg,
        openmv_publish_mono_image_arg,
        event_tracker_debug_enabled_arg,
        event_tracker_debug_clip_count_arg,
        enable_latency_trace_arg,
        ball_tracker_publish_mask_arg,
        aruco_top_cam_publish_debug_arg,
        inhibit_scene_localizer_arg,
        inhibit_scene_localizer_debug_arg,
        inhibit_ball_3d_pose_estimator_arg,
        inhibit_ball_trajectory_estimator_arg,
        openmv_node,
        ball_tracker_node,
        scene_localizer_nodes,
        realsense_node,
        top_aruco
    ])
