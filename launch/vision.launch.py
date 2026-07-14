from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import UnlessCondition
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

from fr3_teleop.config.teleop_config import OPENMV_PARAMS


def generate_launch_description():
    openmv_pkg_share = get_package_share_directory("openmv_cam")
    openmv_launch_path = os.path.join(openmv_pkg_share, "launch", "openmv.launch.py")
    scene_localizer_launch_path = os.path.join(
        get_package_share_directory("scene_localizer"),
        "launch",
        "all_scene_localizer_nodes.launch.py",
    )

    realsense_launch_path = os.path.join(
        get_package_share_directory("realsense2_camera"),
        "launch",
        "rs_launch.py",
    )

    no_cams_arg = DeclareLaunchArgument(
        "no_cams",
        default_value="false",
        description="Disable vision launch",
    )
    event_frame_mode_arg = DeclareLaunchArgument(
        "event_frame_mode",
        default_value=str(OPENMV_PARAMS.get("event_frame_mode", "cumulative")),
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

    no_cams = LaunchConfiguration("no_cams")
    event_frame_mode = LaunchConfiguration("event_frame_mode")
    event_frame_ch0_ms = LaunchConfiguration("event_frame_ch0_ms")
    event_frame_ch1_ms = LaunchConfiguration("event_frame_ch1_ms")
    event_frame_ch2_ms = LaunchConfiguration("event_frame_ch2_ms")

    openmv_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(openmv_launch_path),
        launch_arguments={
            "event_frame_mode": event_frame_mode,
            "event_frame_ch0_ms": event_frame_ch0_ms,
            "event_frame_ch1_ms": event_frame_ch1_ms,
            "event_frame_ch2_ms": event_frame_ch2_ms,
        }.items(),
        condition=UnlessCondition(no_cams),
    )

    ball_tracker_node = Node(
        package="ball_tracker",
        executable="ball_tracker2",
        name="ball_tracker2",
        output="screen",
        parameters=[
            {
                "input_image_topic": "/top_cam/camera/color/image_raw",
            }
        ],
        condition=UnlessCondition(no_cams),
    )

    realsense_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(realsense_launch_path),
        launch_arguments={
            "serial_no": "_243722074377",
            "camera_namespace": "top_cam",
            "enable_depth": "false",
            "enable_infra1": "false",
            "enable_infra2": "false",
            "pointcloud.enable": "false",
            "align_depth.enable": "false",
            # "rgb_camera.enable_auto_exposure": "false",
            # "rgb_camera.exposure": "3000",
            # "rgb_camera.gain": "64",
            # "rgb_camera.auto_exposure_priority": "false",
            # "rgb_camera.enable_auto_white_balance": "false",
            # "rgb_camera.white_balance": "4500",
        }.items(),
        condition=UnlessCondition(no_cams),
    )

    scene_localizer_nodes = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(scene_localizer_launch_path),
        launch_arguments={}.items(),
        condition=UnlessCondition(no_cams),
    )

    top_aruco = Node(
        package='aruco_opencv',
        executable='aruco_tracker_autostart',
        name='aruco_top_cam',
        namespace='/aruco_top_cam',
        output='screen',
        parameters=[{
            'cam_base_topic': '/top_cam/camera/color/image_raw',
            'marker_size': 0.0982,
            'marker_dict': '6X6_50'
        }],
    )


    return LaunchDescription([
        no_cams_arg,
        event_frame_mode_arg,
        event_frame_ch0_ms_arg,
        event_frame_ch1_ms_arg,
        event_frame_ch2_ms_arg,
        openmv_node,
        ball_tracker_node,
        scene_localizer_nodes,
        realsense_node,
        top_aruco
    ])