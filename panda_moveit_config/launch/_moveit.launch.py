import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.conditions import IfCondition
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder

def generate_launch_description():

    # Command-line arguments
    use_sim_time_arg = DeclareLaunchArgument(
        "use_sim_time",
        default_value="False",
        description="Use simulation time if true, wall clock time otherwise",
    )
    use_sim_time = LaunchConfiguration("use_sim_time")


    rviz_config_arg = DeclareLaunchArgument(
        "rviz_config",
        default_value="moveit.rviz",
        description="RViz configuration file",
    )

    moveit_config = (
        MoveItConfigsBuilder("panda")
        .robot_description(
            file_path="config/panda_gz.urdf.xacro",
        )
        .robot_description_semantic(file_path="config/panda.srdf")
        .trajectory_execution(file_path="config/gripper_moveit_controllers.yaml")
        .moveit_cpp(file_path="config/controller_setting.yaml")
        .robot_description_kinematics(
            file_path="config/kinematics.yaml"
        )
        .to_moveit_configs()
    )

    # Start the actual move_group node/action server
    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[moveit_config.to_dict()],
        arguments=["--ros-args", "--log-level", "info"],
    )

    # RViz
    rviz_base = LaunchConfiguration("rviz_config")
    rviz_config = PathJoinSubstitution(
        [FindPackageShare("panda_moveit_config"), "launch", rviz_base]
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.planning_pipelines,
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits,
            {"use_sim_time": use_sim_time},
        ],
    )

    # Static TF
    static_tf_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="log",
        arguments=["--x", "0.05", "--y", "0.0", "--z", "1.02",
                   "--yaw", "0.0", "--pitch", "0.0", "--roll", "0.0",
                   "--frame-id", "world",
                   "--child-frame-id", "panda_link0"],
        parameters=[{"use_sim_time": use_sim_time}],
    )
    
    static_tf_node2 = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_transform_publisher",
        output="log",
        arguments=["--x", "0.2", "--y", "0.6", "--z", "0.7",
                   "--yaw", "-1.5708", "--pitch", "0.0", "--roll", "3.1416",
                   "--frame-id", "panda_link0",
                   "--child-frame-id", "camera_norm_link"],
        parameters=[{"use_sim_time": use_sim_time}],
    )
    
    # Publish TF
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="both",
        parameters=[
            moveit_config.robot_description, 
            {"use_sim_time": use_sim_time}
        ],
    )

    # Add ros2_control node
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            moveit_config.robot_description,
            {"use_sim_time": use_sim_time}
        ],
        output="screen",
    )

    controller_nodes = []

    for controller in [
        "panda_arm_controller",
        "panda_hand_controller",
        "joint_state_broadcaster",
    ]:
        controller_nodes.append(
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=[
                    controller,
                    "--controller-manager",
                    "/controller_manager",
                ],
                parameters=[{"use_sim_time": use_sim_time}],
                output="log",
            )
        )

    # controller_load_event = RegisterEventHandler(
    #     OnProcessStart(
    #         target_action=robot_state_publisher,
    #         on_start=[
    #             ros2_control_node,
    #             joint_state_broadcaster_spawner,
    #             panda_arm_controller_spawner,
    #             panda_hand_controller_spawner,
    #         ]
    #     )
    # )

    # 暂时不通过这里启动
    grasp_demo = Node(
            package='moveit2grasp',
            executable='grasp_demo',
            name='grasp_demo',
            output='screen',
            parameters=[
                moveit_config.to_dict(),
                {"use_sim_time": use_sim_time},
            ],
        )
    
    """
    ros2_control_node如果启动,会出现
    [ros2_control_node-7] [WARN] [1749542742.750309028] [controller_manager]: 
    Waiting for data on 'robot_description' topic to finish initialization
    频繁的警告信息,目前推测是ros2_control和gz_ros2_control冲突,
    所以暂时不通过这里启动
    """
    
    return LaunchDescription(
        [
            rviz_config_arg,
            use_sim_time_arg,
            robot_state_publisher,
            static_tf_node,
            static_tf_node2,
            
            rviz_node,
            # move_group_node,
            # ros2_control_node,
        ] + controller_nodes
    )
    