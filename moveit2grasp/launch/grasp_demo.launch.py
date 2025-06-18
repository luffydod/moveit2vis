import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():
    # 命令行参数
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="True",
        description="使用仿真时间而非系统时间",
    )
    use_sim_time = LaunchConfiguration("use_sim_time")

    # MoveIt配置项
    moveit_config = (
        MoveItConfigsBuilder("panda")
        .robot_description(
            file_path="config/panda_gz.urdf.xacro",
        )
        .robot_description_semantic(file_path="config/panda.srdf")
        .trajectory_execution(file_path="config/gripper_moveit_controllers.yaml")
        .moveit_cpp(file_path="config/controller_setting.yaml")
        # .moveit_cpp(os.path.join(get_package_share_directory('panda_moveit_config'), "config", "controller_setting.yaml"))
        .robot_description_kinematics(
            file_path="config/kinematics.yaml"
        )
        .to_moveit_configs()
    )

    return LaunchDescription([
        use_sim_time_arg,
        Node(
            package='moveit2grasp',
            executable='grasp_demo',
            name='grasp_demo',
            output='screen',
            parameters=[
                moveit_config.to_dict(),
                {"use_sim_time": use_sim_time},
            ],
        )
    ])