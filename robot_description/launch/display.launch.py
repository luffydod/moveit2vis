import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
import yaml

def generate_launch_description():
    # 声明参数
    gui_arg = DeclareLaunchArgument(name='gui', default_value='true', 
                                   description='Flag to enable joint_state_publisher_gui')
    rviz_arg = DeclareLaunchArgument(name='rviz', default_value='true',
                                    description='Flag to enable RViz2')

    # 获取参数
    gui = LaunchConfiguration('gui')
    rviz = LaunchConfiguration('rviz')
    
    # 获取包路径
    pkg_path = get_package_share_directory('robot_description')
    
    # 机器人描述
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([pkg_path, "urdf", "panda_epick.urdf.xacro"]),
        ]
    )
    robot_description = {"robot_description": ParameterValue(robot_description_content, value_type=str)}
    
    # RViz配置
    rviz_config_file = PathJoinSubstitution(
        [pkg_path, "rviz", "display.rviz"]
    )
    
    # 节点
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description],
    )
    
    joint_state_publisher_node = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="joint_state_publisher",
    )
    
    joint_state_publisher_gui_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        name="joint_state_publisher_gui",
        condition=IfCondition(gui),
    )
    
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        condition=IfCondition(rviz),
    )
    
    return LaunchDescription([
        gui_arg,
        rviz_arg,
        robot_state_publisher_node,
        joint_state_publisher_node,
        joint_state_publisher_gui_node,
        rviz_node,
    ])