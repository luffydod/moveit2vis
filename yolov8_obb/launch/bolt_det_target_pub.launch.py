from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='yolov8_obb',
            executable='bolt_det_pub',
            name='bolt_detector',
            output='screen',
            # parameters=[],
        ),
        Node(
            package='yolov8_obb',
            executable='target_pos_pub',
            name='target_pose_publisher',
            output='screen',
            # parameters=[],
        )
    ])
