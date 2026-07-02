from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import UnlessCondition
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

from fr3_teleop.config.teleop_config import (
    TOPICS_TO_RECORD,
)

def generate_launch_description():
    pkg_share = get_package_share_directory("fr3_teleop")
    teleop_config_yaml = os.path.join(pkg_share, "config", "teleop_config.yaml")
    dashboard_script = "/home/jau/dyros/src/fr3_teleop/helpers/dashboard.py"

    print(f"[teleop.launch] Recording topics: {TOPICS_TO_RECORD}")

    openmv_pkg_share = get_package_share_directory("openmv_cam")
    camera_launch_file = os.path.join(openmv_pkg_share, "launch", "both_cams.launch.py")

    no_cams_arg = DeclareLaunchArgument(
        "no_cams",
        default_value="false",
        description="Disable camera launch",
    )
    bag_name_arg = DeclareLaunchArgument(
        "bag_name",
        default_value="",
        description="Fixed rosbag name (empty = auto-generated timestamp name)",
    )
    target_dir_arg = DeclareLaunchArgument(
        "target_dir",
        default_value="/home/jau/data/bags/",
        description="Directory where bags and raw-event HDF5 files are written",
    )
    prefix_arg = DeclareLaunchArgument(
        "prefix",
        default_value="",
        description="Name prefix used when bag_name is empty",
    )
    record_raw_events_arg = DeclareLaunchArgument(
        "record_raw_events",
        default_value="true",
        description="Also record raw OpenMV event HDF5 alongside the rosbag",
    )
    openmv_node_name_arg = DeclareLaunchArgument(
        "openmv_node_name",
        default_value="/openmv_event_cam",
        description="ROS node name of the OpenMV event camera node",
    )
    event_frame_mode_arg = DeclareLaunchArgument(
        "event_frame_mode",
        default_value="shifted",
        description="Event frame mode for openmv_cam-node",
    )
    event_frame_ch0_ms_arg = DeclareLaunchArgument(
        "event_frame_ch0_ms",
        default_value="50.0",
        description="Channel-0 frame integration window (ms) for openmv_cam-node",
    )
    event_frame_ch1_ms_arg = DeclareLaunchArgument(
        "event_frame_ch1_ms",
        default_value="250.0",
        description="Channel-1 frame integration window (ms) for openmv_cam-node",
    )
    event_frame_ch2_ms_arg = DeclareLaunchArgument(
        "event_frame_ch2_ms",
        default_value="1500.0",
        description="Channel-2 frame integration window (ms) for openmv_cam-node",
    )

    no_cams = LaunchConfiguration("no_cams")
    bag_name = LaunchConfiguration("bag_name")
    target_dir = LaunchConfiguration("target_dir")
    prefix = LaunchConfiguration("prefix")
    record_raw_events = LaunchConfiguration("record_raw_events")
    openmv_node_name = LaunchConfiguration("openmv_node_name")
    event_frame_mode = LaunchConfiguration("event_frame_mode")
    event_frame_ch0_ms = LaunchConfiguration("event_frame_ch0_ms")
    event_frame_ch1_ms = LaunchConfiguration("event_frame_ch1_ms")
    event_frame_ch2_ms = LaunchConfiguration("event_frame_ch2_ms")

    dashboard_process = ExecuteProcess(
        cmd=["python3", dashboard_script, "--ros-args", "--params-file", teleop_config_yaml],
        output="screen",
    )

    record_manager_node = Node(
        package="fr3_teleop",
        executable="record_manager_node.py",
        name="record_manager",
        output="screen",
        parameters=[
            teleop_config_yaml,
            {
                "topics_to_record": TOPICS_TO_RECORD,
            },
            {
                "bag_name": bag_name,
                "target_dir": target_dir,
                "prefix": prefix,
                "record_raw_events": record_raw_events,
                "openmv_node_name": openmv_node_name,
            },
        ],
    )

    return LaunchDescription([
        no_cams_arg,
        bag_name_arg,
        target_dir_arg,
        prefix_arg,
        record_raw_events_arg,
        openmv_node_name_arg,
        event_frame_mode_arg,
        event_frame_ch0_ms_arg,
        event_frame_ch1_ms_arg,
        event_frame_ch2_ms_arg,
        record_manager_node,
        Node(
            package="joy",
            executable="joy_node",
            name="joy_node",
            output="screen",
        ),
        Node(
            package="fr3_teleop",
            executable="ps4_input_manager",
            name="ps4_input_manager",
            output="screen",
            parameters=[teleop_config_yaml],
        ),
        Node(
            package="fr3_teleop",
            executable="gripper_manager",
            name="gripper_manager",
            output="screen",
            parameters=[teleop_config_yaml],
        ),
        Node(
            package="fr3_teleop",
            executable="franka_wrench_extractor",
            name="franka_wrench_extractor",
            output="screen",
            parameters=[teleop_config_yaml],
        ),
        dashboard_process,
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(camera_launch_file),
            launch_arguments={
                "event_frame_mode": event_frame_mode,
                "event_frame_ch0_ms": event_frame_ch0_ms,
                "event_frame_ch1_ms": event_frame_ch1_ms,
                "event_frame_ch2_ms": event_frame_ch2_ms,
            }.items(),
            condition=UnlessCondition(no_cams),
        ),
    ])
