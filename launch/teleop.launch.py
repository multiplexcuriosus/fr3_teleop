from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory("fr3_teleop")
    ps4_params = os.path.join(pkg_share, "config", "ps4_input_manager.yaml")
    dashboard_script = "/home/jau/dyros/src/fr3_teleop/dashboard.py"

    dashboard_process = ExecuteProcess(
        cmd=["python3", dashboard_script],
        output="screen",
    )

    return LaunchDescription([
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
            parameters=[ps4_params],
        ),
        Node(
            package="fr3_teleop",
            executable="gripper_manager",
            name="gripper_manager",
            output="screen",
            parameters=[ps4_params],
        ),
        dashboard_process,
    ])