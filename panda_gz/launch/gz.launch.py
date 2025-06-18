import os
import xacro
from launch import LaunchDescription
from launch.actions import SetEnvironmentVariable, LogInfo
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
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

    world_arg = DeclareLaunchArgument(
        "world",
        default_value="arm_on_the_table.sdf",
        description="Gazebo世界文件",
    )
    
    rviz_config_arg = DeclareLaunchArgument(
        "rviz_config",
        default_value="moveit_empty.rviz",
        description="RViz配置文件",
    )
    
    # 添加机器人位姿参数
    robot_x_arg = DeclareLaunchArgument("robot_x", default_value="0.05", description="机器人X坐标")
    robot_y_arg = DeclareLaunchArgument("robot_y", default_value="0.0", description="机器人Y坐标")
    robot_z_arg = DeclareLaunchArgument("robot_z", default_value="1.02", description="机器人Z坐标")
    
    # 添加物理引擎参数
    physics_engine_arg = DeclareLaunchArgument(
        "physics_engine", 
        default_value="gz-physics-bullet-featherstone-plugin",
        description="Gazebo物理引擎"
    )

    # 获取世界文件路径
    world_base = LaunchConfiguration("world")
    panda_gz_pkg = get_package_share_directory("panda_gz")
    panda_desc_pkg = get_package_share_directory("panda_description")
    world_path = PathJoinSubstitution([panda_gz_pkg, "worlds", world_base])

    # 设置Gazebo资源路径
    gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=[
            os.path.join(panda_desc_pkg, ".."),
            ':',
            os.path.join(panda_gz_pkg, 'worlds'),
            ':',
            os.path.join(panda_gz_pkg, 'models'),
            ':',
            os.path.join(panda_gz_pkg, "..", "..")
        ]
    )
    
    # 检查世界文件是否存在
    world_file_check = LogInfo(msg=["正在加载世界文件: ", world_path])
    
    # 设置Gazebo模型路径
    gz_model_path = SetEnvironmentVariable(
        name='GZ_SIM_MODEL_PATH',
        value=[
            panda_desc_pkg,
            ':',
            os.path.join(panda_gz_pkg, 'models')
        ]
    )
    
    # 构建正确的命令行参数列表
    gz_args = [
        world_path,
        " -r",
        " -v 4",
        " --physics-engine ",
        LaunchConfiguration("physics_engine")
    ]

    # 启动Gz sim仿真
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            FindPackageShare("ros_gz_sim"), "/launch/gz_sim.launch.py"
        ]),
        launch_arguments={
            "gz_args": gz_args,
            "use_sim_time": use_sim_time
        }.items(),
    )

    # 加载URDF模型
    try:
        xacro_file = os.path.join(get_package_share_directory('panda_moveit_config'),
                                'config',
                                'panda_gz.urdf.xacro')
        doc = xacro.process_file(xacro_file, mappings={'use_sim' : 'true'})
        robot_desc = doc.toprettyxml(indent='  ')
    except Exception as e:
        robot_desc = Command(['xacro ', xacro_file, ' use_sim:=true'])
    
    # 加载机器人模型到Gazebo
    spawn_entity = Node(
        package="ros_gz_sim",
        executable="create",
        arguments=[
            '-string', robot_desc,
            '-x', LaunchConfiguration("robot_x"),
            '-y', LaunchConfiguration("robot_y"),
            '-z', LaunchConfiguration("robot_z"),
            '-R', '0.0',
            '-P', '0.0',
            '-Y', '0.0',
            '-name', 'arm_robot',
            '-allow_renaming', 'false',
        ],
        output="screen",
    )
    # 使用topic方式加载机器人模型
    # spawn_entity = Node(
    #     package="ros_gz_sim",
    #     executable="create",
    #     arguments=[
    #         '-topic', 'robot_description',
    #         '-x', LaunchConfiguration("robot_x"),
    #         '-y', LaunchConfiguration("robot_y"),
    #         '-z', LaunchConfiguration("robot_z"),
    #         '-R', '0.0',
    #         '-P', '0.0',
    #         '-Y', '0.0',
    #         '-name', 'arm_robot',
    #         '-allow_renaming', 'false',
    #     ],
    #     output="screen",
    # )

    # 创建ROS-Gazebo桥接节点函数
    def create_bridge_node(topic, msg_type, direction='@'):
        return Node(
            package='ros_gz_bridge',
            executable='parameter_bridge',
            arguments=[f'{topic}{direction}{msg_type}'],
            output='screen'
        )
    
    # 创建各种桥接节点
    img_bridge = create_bridge_node('/image_raw', 'sensor_msgs/msg/Image@gz.msgs.Image')
    depth_bridge = create_bridge_node('/camera/depth/image_raw', 'sensor_msgs/msg/Image@gz.msgs.Image')
    clock_bridge = create_bridge_node('/clock', 'rosgraph_msgs/msg/Clock[gz.msgs.Clock')

    # 获取包路径
    panda_moveit_config_path = FindPackageShare("panda_moveit_config")

    # 包含MoveIt配置启动文件
    moveit_config = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([panda_moveit_config_path, "launch", "_moveit.launch.py"])
        ]),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "rviz_config": LaunchConfiguration("rviz_config"),
        }.items(),
    )

    return LaunchDescription([
        # 参数声明
        use_sim_time_arg,
        world_arg,
        rviz_config_arg,
        robot_x_arg,
        robot_y_arg,
        robot_z_arg,
        physics_engine_arg,
        
        # 环境设置
        gz_resource_path,
        gz_model_path,
        world_file_check,

        # 启动组件
        moveit_config,
        gz_sim,
        spawn_entity,
        
        # 桥接节点
        img_bridge,
        depth_bridge,
        clock_bridge,
    ])