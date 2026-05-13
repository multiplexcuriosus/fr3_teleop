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
    DASHBOARD_PARAMS,
    PS4_PARAMS,
    GRIPPER_PARAMS,
)


def _format_ros_arg_value(value):
    if isinstance(value, bool):
        return "true" if value else "false"
    return str(value)


def _dashboard_ros_args(params):
    args = ["--ros-args"]
    for key, value in params.items():
        if isinstance(value, (str, bool, int, float)):
            args.extend(["-p", f"{key}:={_format_ros_arg_value(value)}"])
    return args


def generate_launch_description():
    pkg_share = get_package_share_directory("fr3_teleop")
    dashboard_script = "/home/jau/dyros/src/fr3_teleop/helpers/dashboard.py"

    print(f"[teleop.launch] Recording topics: {TOPICS_TO_RECORD}")
    print(f"[teleop.launch] Dashboard event mono topic: {DASHBOARD_PARAMS['event_frame_mono_topic']}")
    print(f"[teleop.launch] Dashboard event 3ch topic: {DASHBOARD_PARAMS['event_frame_3ch_topic']}")

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
        default_value="data/bags",
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

    no_cams = LaunchConfiguration("no_cams")
    bag_name = LaunchConfiguration("bag_name")
    target_dir = LaunchConfiguration("target_dir")
    prefix = LaunchConfiguration("prefix")
    record_raw_events = LaunchConfiguration("record_raw_events")
    openmv_node_name = LaunchConfiguration("openmv_node_name")

    dashboard_process = ExecuteProcess(
        cmd=["python3", dashboard_script, *_dashboard_ros_args(DASHBOARD_PARAMS)],
        output="screen",
    )

    record_manager_node = Node(
        package="fr3_teleop",
        executable="record_manager_node.py",
        name="record_manager",
        output="screen",
        parameters=[
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
            parameters=[PS4_PARAMS],
        ),
        Node(
            package="fr3_teleop",
            executable="gripper_manager",
            name="gripper_manager",
            output="screen",
            parameters=[GRIPPER_PARAMS],
        ),
        dashboard_process,
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(camera_launch_file),
            condition=UnlessCondition(no_cams),
        ),
    ])
