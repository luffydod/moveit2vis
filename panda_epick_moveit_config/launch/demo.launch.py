from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_demo_launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    moveit_config = MoveItConfigsBuilder("panda_epick", package_name="panda_epick_moveit_config").to_moveit_configs()
    return generate_demo_launch(moveit_config)
