import os
import xacro
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
import yaml

from launch import LaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.actions import RegisterEventHandler, SetEnvironmentVariable
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.conditions import IfCondition, UnlessCondition
from moveit_configs_utils import MoveItConfigsBuilder

# 这些节点启动是有顺序的
def generate_launch_description():
    robot_description_path = os.path.join(
        get_package_share_directory('robot_description'),)
    
    # 找到panda_moveit_config功能包
    arm_robot_sim_path = os.path.join(
        get_package_share_directory('panda_epick_moveit_config'))
    

    # Set gazebo sim resource path
    gazebo_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=[
            os.path.join(arm_robot_sim_path, 'worlds'), ':' +
            str(Path(robot_description_path).parent.resolve())
            ]
        )

    # Load the ROS2 control parameters from the YAML file
    ros2_control_params_file = os.path.join(arm_robot_sim_path, 'config', 'ros2_controllers.yaml')
    ros2_control_params = {"use_sim_time": True}
    
    with open(ros2_control_params_file, 'r') as file:
        ros2_control_params.update(yaml.safe_load(file))

    ros_namespace = LaunchConfiguration('ros_namespace', default='')

    arguments = LaunchDescription([
                DeclareLaunchArgument('world', default_value='arm_on_the_table',
                          description='Gz sim World'),
           ]
    )

    gazebo = IncludeLaunchDescription(
                PythonLaunchDescriptionSource([os.path.join(
                    get_package_share_directory('ros_gz_sim'), 'launch'), '/gz_sim.launch.py']),
                launch_arguments=[
                    ('gz_args', [LaunchConfiguration('world'),
                                 '.sdf',
                                 ' -v 4',
                                 ' -r',
                                 ' --physics-engine gz-physics-bullet-featherstone-plugin']
                    )
                ]
             )

    xacro_file = os.path.join(arm_robot_sim_path,
                              'config',
                              'panda_epick.urdf.xacro')

    doc = xacro.process_file(xacro_file, mappings={'use_sim' : 'true'})

    robot_desc = doc.toprettyxml(indent='  ')
    
    gz_spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=['-string', robot_desc,
                   '-x', '0.05',
                   '-y', '0.0',
                   '-z', '1.02',
                   '-R', '0.0',
                   '-P', '0.0',
                   '-Y', '0.0',
                   '-name', 'arm_robot',
                   '-allow_renaming', 'false'],
    )

    moveit_config = (
        MoveItConfigsBuilder(robot_name="panda_epick")
        .robot_description(file_path="config/panda_epick.urdf.xacro")
        .robot_description_semantic(file_path="config/panda_epick.srdf")
        .trajectory_execution(file_path="config/moveit_controllers.yaml")
        .moveit_cpp(arm_robot_sim_path + "/config/controller_setting.yaml")
        .to_moveit_configs()
    )
    # .joint_limits(file_path="config/joint_limits.yaml")
    # .robot_description_kinematics(file_path="config/kinematics.yaml")

    rviz_config_file = os.path.join(
        arm_robot_sim_path,
        "rviz",
        "motion_planning.rviz",
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            {"use_sim_time": True},
        ],
    )

    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="log",
        arguments=["--frame-id", "world", "--child-frame-id", "panda_link0"],
        parameters=[{"use_sim_time": True},],
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="log",
        parameters=[moveit_config.robot_description,
                    {"use_sim_time": True},
                    ],
    )

    # Bridge
    img_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/image_raw@sensor_msgs/msg/Image@gz.msgs.Image'], 
        output='screen'
    )

    depth_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/camera/depth/image_raw@sensor_msgs/msg/Image@gz.msgs.Image'
        ],
        output='screen'
    )
    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        parameters=[{"use_sim_time": True}],
        output='screen'
    )

    # ros2 control node
    ros2_control_node = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[
            moveit_config.robot_description,
            ros2_control_params,
        ],
        output='screen',
    )
    joint_state_broadcaster = Node(
        package='controller_manager',
        executable='spawner',
        output='screen',
        arguments=[
            'joint_state_broadcaster',
            '--controller-manager', '{}/controller_manager'.format(ros_namespace)
        ],
    )

    # Load controllers
    controller_nodes = []
    controller_manager_arg = PathJoinSubstitution([
        ros_namespace,  # 确保已声明此Launch参数
        'controller_manager'
    ])
    for controller in [
        "panda_arm_controller",
        "epick_gripper_controller",
    ]:
        controller_nodes.append(Node(
            package='controller_manager',
            executable='spawner',
            output='screen',
            arguments=[
                controller,
                '--controller-manager',
                controller_manager_arg
            ],
        ))
    
    # Load  ExecuteTaskSolutionCapability so we can execute found solutions in simulation
    move_group_capabilities = {
        "capabilities": "move_group/ExecuteTaskSolutionCapability"
    }

    # Start the actual move_group node/action server
    run_move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            move_group_capabilities,
            {"use_sim_time": True},
        ],
    )

    return LaunchDescription(
        [
            gazebo_resource_path,
            arguments,
            gazebo,
            gz_spawn_entity,
            robot_state_publisher,
            # bridge,
            img_bridge,
            depth_bridge,
            clock_bridge,
            # rviz_node,
            # static_tf,
            run_move_group_node,
            ros2_control_node,
            joint_state_broadcaster,
        ] + controller_nodes
    )
