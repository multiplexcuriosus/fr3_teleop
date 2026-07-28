from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os

from fr3_teleop.config.interfaces import (
    RECORD_MANAGER_INTERFACE_PARAMS,
    DASHBOARD_INTERFACE_PARAMS,
    PS4_INTERFACE_PARAMS,
    GRIPPER_INTERFACE_PARAMS,
    WRENCH_INTERFACE_PARAMS,
)

def generate_launch_description():
    pkg_share = get_package_share_directory("fr3_teleop")
    teleop_config_yaml = os.path.join(pkg_share, "config", "teleop_config.yaml")
    interception_launch_path = os.path.join(
        get_package_share_directory("ball_interception_controller"),
        "launch",
        "interception_controller.launch.py",
    )

    print(
        "[teleop.launch] Recording topics: "
        f"{RECORD_MANAGER_INTERFACE_PARAMS['topics_to_record']}"
    )
    print(
        "[teleop.launch] Required (presence-gated) topics: "
        f"{RECORD_MANAGER_INTERFACE_PARAMS['required_topics']}"
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
    bag_name = LaunchConfiguration("bag_name")
    target_dir = LaunchConfiguration("target_dir")
    prefix = LaunchConfiguration("prefix")

    dashboard_node = Node(
        package="fr3_teleop",
        executable="teleop_dashboard",
        name="teleop_dashboard",
        output="screen",
        parameters=[
            teleop_config_yaml,
            DASHBOARD_INTERFACE_PARAMS,
        ],
    )

    record_manager_node = Node(
        package="fr3_teleop",
        executable="record_manager_node.py",
        name="record_manager",
        output="screen",
        parameters=[
            teleop_config_yaml,
            RECORD_MANAGER_INTERFACE_PARAMS,
            {
                "bag_name": bag_name,
                "target_dir": target_dir,
                "prefix": prefix,
            },
        ],
    )

    scene_interception_controller_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(interception_launch_path),
        launch_arguments={
            "node_name": "interception_controller",
            "command_source": "scene",
            "execution_backend": "cont_tracker",
            "vmax": "1.2",
        }.items(),
    )

    rollout_interception_controller_node = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(interception_launch_path),
        launch_arguments={
            "node_name": "rollout_interception_controller",
            "command_source": "rollout",
            "execution_backend": "cont_tracker",
            "dry_run": "true",
            "rollout_prediction_topic": "/act/intercept_prediction_current_abs_s",
            "current_tcp_s_topic": "/middle_line/current_tcp_s",
            "max_current_tcp_s_age_sec": "0.15",
            "rollout_prediction_timeout_sec": "0.25",
            "rollout_min_target_s_m": "-0.15",
            "rollout_max_target_s_m": "0.15",
        }.items(),
    )

    return LaunchDescription([
        bag_name_arg,
        target_dir_arg,
        prefix_arg,
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
            parameters=[
                teleop_config_yaml,
                PS4_INTERFACE_PARAMS,
            ],
        ),
        Node(
            package="fr3_teleop",
            executable="gripper_manager",
            name="gripper_manager",
            output="screen",
            parameters=[
                teleop_config_yaml,
                GRIPPER_INTERFACE_PARAMS,
            ],
        ),
        Node(
            package="fr3_teleop",
            executable="franka_wrench_extractor",
            name="franka_wrench_extractor",
            output="screen",
            parameters=[
                teleop_config_yaml,
                WRENCH_INTERFACE_PARAMS,
            ],
        ),
        dashboard_node,
        scene_interception_controller_node,
        rollout_interception_controller_node,

        #vision_node,
    ])

