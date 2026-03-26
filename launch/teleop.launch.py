from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
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
            parameters=[{
                "button_gripper_open": 2,
                "button_gripper_close": 0,
                "button_episode_start": 9,
                "button_episode_stop": 8,
                "button_home": 1,
                "gripper_action_name": "/right_franka_gripper/move",
                "home_action_name": "/fr3_move_to_joint",
                "gripper_open_width": 0.080,
                "gripper_close_width": 0.06,
                "gripper_speed": 0.05,
            }],
        ),
    ])