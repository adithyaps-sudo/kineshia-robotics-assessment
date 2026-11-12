"""Launch the 3-DOF planar arm controller and GUI."""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="planar_arm_control",
            executable="controller_node",
            name="controller_node",
            output="screen",
        ),
        Node(
            package="planar_arm_control",
            executable="gui_node",
            name="gui_node",
            output="screen",
            additional_env={
                "QT_QPA_PLATFORM": "xcb",
            },
        ),
    ])
