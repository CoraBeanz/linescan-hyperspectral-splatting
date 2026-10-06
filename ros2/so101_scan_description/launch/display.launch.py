"""Show the arm and scanner head in RViz, with sliders for the joints. No hardware.

    ros2 launch so101_scan_description display.launch.py

Arguments: gui (sliders, default true), rviz (default true).
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    share = FindPackageShare("so101_scan_description")
    gui = LaunchConfiguration("gui")
    robot_description = ParameterValue(
        Command(["xacro ", PathJoinSubstitution([share, "urdf", "so101_scan.urdf.xacro"]),
                 " ros2_control:=false"]),
        value_type=str)

    return LaunchDescription([
        DeclareLaunchArgument("gui", default_value="true", description="joint sliders"),
        DeclareLaunchArgument("rviz", default_value="true"),
        Node(package="robot_state_publisher", executable="robot_state_publisher",
             parameters=[{"robot_description": robot_description, "publish_frequency": 50.0}]),
        Node(package="joint_state_publisher_gui", executable="joint_state_publisher_gui",
             condition=IfCondition(gui)),
        Node(package="joint_state_publisher", executable="joint_state_publisher",
             condition=UnlessCondition(gui)),
        Node(package="rviz2", executable="rviz2", output="log",
             arguments=["-d", PathJoinSubstitution([share, "rviz", "so101_scan.rviz"])],
             condition=IfCondition(LaunchConfiguration("rviz"))),
    ])
