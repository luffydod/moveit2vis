from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import yaml
from launch_param_builder import load_xacro
from pathlib import Path
from launch_param_builder import get_package_share_directory
from launch.actions import OpaqueFunction


def get_xacro_content(context,
    xacro_file=Path(get_package_share_directory('robot_description')) / 'urdf' / 'panda_epick.urdf.xacro', 
    **kwargs):
    xacro_file = Path(xacro_file.perform(context)) if isinstance(xacro_file, LaunchConfiguration) else Path(xacro_file) if isinstance(xacro_file, str) else xacro_file
    
    def get_param_str(param):
        val = param if isinstance(param, str) else 'false' if param == False else 'true' if param == True else (param.perform(context) if context is not None else param) if isinstance(param, LaunchConfiguration) else str(param)
        return val if not val else val[1:-1] if isinstance(val, str) and val[0] in ['"', '\''] and val[-1] in ['"', '\''] else val

    mappings = {}
    for k, v in kwargs.items():
        mappings[k] = get_param_str(v)
    return load_xacro(xacro_file, mappings=mappings)

def launch_setup(context, *args, **kwargs):
    # 设置机器人描述参数
    robot_description = {
        "robot_description":
        get_xacro_content(
            context=context,
            xacro_file=PathJoinSubstitution(
                [FindPackageShare('robot_description'), 'urdf', 'panda_epick.urdf.xacro']
            ),
        )
    }
    
    # 配置rviz
    rviz_config_file = PathJoinSubstitution(
        [FindPackageShare('robot_description'), "rviz", "display.rviz"]
    )
    
    # 创建节点
    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[robot_description],
    )
    
    # 添加静态变换发布器
    static_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_transform_publisher',
        output='screen',
        arguments=['0', '0', '0', '0', '0', '0', 'world', 'panda_link0']
    )
    
    joint_state_publisher_gui_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        output="screen",
    )
    
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config_file],
    )
    
    return [
        robot_state_publisher_node,
        joint_state_publisher_gui_node,
        # static_tf_node,
        rviz_node,
    ]

def generate_launch_description():
    return LaunchDescription([
        OpaqueFunction(function=launch_setup)
    ])