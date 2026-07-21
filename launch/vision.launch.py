from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import UnlessCondition
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

from fr3_teleop.config.interfaces import TOPICS
from fr3_teleop.config.teleop_config import OPENMV_PARAMS


def generate_launch_description():
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

    event_frame_ch0_ms = LaunchConfiguration("event_frame_ch0_ms")
    event_frame_ch1_ms = LaunchConfiguration("event_frame_ch1_ms")
    event_frame_ch2_ms = LaunchConfiguration("event_frame_ch2_ms")

    openmv_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(openmv_launch_path),
        launch_arguments={
            "event_frame_ch0_ms": event_frame_ch0_ms,
            "event_frame_ch1_ms": event_frame_ch1_ms,
            "event_frame_ch2_ms": event_frame_ch2_ms
        }.items(),
    )

    ball_tracker_node = Node(
        package="ball_tracker",
        executable="ball_tracker2",
        name="ball_tracker2",
        output="screen",
        parameters=[
            {
                "input_image_topic": TOPICS["rgb_image"]
            }
        ]
    )

    realsense_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(realsense_launch_path),
        launch_arguments={
            "serial_no": "_243722074377", # my-own: _017322074405
            "camera_namespace": "top_cam",
            "camera_name": "camera",

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
        PythonLaunchDescriptionSource(scene_localizer_launch_path)
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
        }]
    )


    return LaunchDescription([
        event_frame_ch0_ms_arg,
        event_frame_ch1_ms_arg,
        event_frame_ch2_ms_arg,
        openmv_node,
        ball_tracker_node,
        scene_localizer_nodes,
        realsense_node,
        top_aruco
    ])