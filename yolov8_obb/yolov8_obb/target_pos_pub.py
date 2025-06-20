#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from perception_interfaces.msg import Detection2DWithDepthArray
from geometry_msgs.msg import PoseStamped, Pose
import numpy as np
import math
import tf2_ros
import tf2_geometry_msgs
from tf2_ros import TransformException
from tf_transformations import quaternion_from_euler

NAME_SPACE = "/perception"

class TargetPosePublisher(Node):
    def __init__(self):
        super().__init__('target_pose_publisher')
        
        # 创建订阅器，订阅检测结果话题
        self.subscription = self.create_subscription(
            Detection2DWithDepthArray,
            f'{NAME_SPACE}/det_2d_d',
            self.detection_callback,
            10)
        
        # 创建发布器，发布目标位姿
        self.target_pose_publisher = self.create_publisher(
            PoseStamped,
            f'{NAME_SPACE}/target_pose',
            10)
        
        # 相机内参 (需要根据实际相机参数调整)
        self.fx = 253.93635749816895  # 焦距x
        self.fy = 253.93635749816895  # 焦距y
        self.cx = 320.0  # 光心x
        self.cy = 240.0  # 光心y
        
        # 创建TF缓冲区和监听器
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        
        self.camera_frame = "camera_norm_link"  # 相机坐标系
        self.base_frame = "panda_link0"  # 更改为固定的基坐标系，避免使用动态的末端执行器坐标系
        
        self.get_logger().info('目标位姿发布节点已启动')

    def detection_callback(self, msg):
        """处理检测结果, 提取3D位置并发布目标位姿"""
        if not msg.detections:
            self.get_logger().info('没有检测到目标')
            return
        
        # 获取置信度最高的检测结果（通常是第一个）
        detection = msg.detections[0]
        
        # 提取2D像素坐标和深度值
        pixel_x = detection.detection.bbox.center.position.x
        pixel_y = detection.detection.bbox.center.position.y
        # 方向角度theta，这里假设是绕Z轴的旋转角度
        theta = detection.detection.bbox.center.theta
        
        depth = detection.depth_center
        
        if math.isnan(depth) or depth <= 0:
            self.get_logger().warn(f'无效的深度值: {depth}')
            return
        
        # 步骤1: 将像素坐标转换为标准相机坐标系（Z轴指向前方，X轴向右，Y轴向下）
        # 注意：像素坐标系原点在左上角，而相机坐标系原点在中心
        x_std = (pixel_x - self.cx) * depth / self.fx  # 向右为正
        y_std = (pixel_y - self.cy) * depth / self.fy  # 向下为正
        z_std = depth                                   # 向前为正
        
        self.get_logger().info(f'标准相机坐标系: x={x_std:.3f}, y={y_std:.3f}, z={z_std:.3f}')
        
        # 创建相机坐标系下的目标位姿
        camera_pose = PoseStamped()
        camera_pose.header.stamp = msg.header.stamp  # 使用检测消息的时间戳
        camera_pose.header.frame_id = self.camera_frame
        camera_pose.pose.position.x = x_std
        camera_pose.pose.position.y = y_std
        camera_pose.pose.position.z = z_std
        
        self.get_logger().info(f'目标方向角: theta={theta:.3f} rad')
        
        try:
            # 直接从相机坐标系转换到基坐标系
            transform = self.tf_buffer.lookup_transform(
                self.base_frame,
                self.camera_frame,
                msg.header.stamp,  # 使用检测消息的时间戳
                timeout=rclpy.duration.Duration(seconds=0.5))
            
            # 执行位姿变换
            base_pose = tf2_geometry_msgs.do_transform_pose(
                camera_pose.pose, transform
            )
            
            target_pose = PoseStamped()
            target_pose.header.stamp = msg.header.stamp
            target_pose.header.frame_id = self.base_frame
            target_pose.pose = base_pose
            
            # 修改：对于Z轴方向相反的情况，需要将角度取反，并且调整欧拉角
            # 在基座坐标系下，Z轴指向上，而在末端执行器坐标系下，Z轴指向下
            # 添加PI旋转将使orientation正确面向目标
            q = quaternion_from_euler(math.pi, 0, -theta+0.3825)  # 注意角度取反，并添加X轴上的180度旋转
            target_pose.pose.orientation.x = q[0]
            target_pose.pose.orientation.y = q[1]
            target_pose.pose.orientation.z = q[2]
            target_pose.pose.orientation.w = q[3]
            
            # 发布基坐标系下的目标位姿
            self.target_pose_publisher.publish(target_pose)
            
            self.get_logger().info(f'基坐标系下的位置: x={base_pose.position.x:.3f}, ' +
                                   f'y={base_pose.position.y:.3f}, z={base_pose.position.z:.3f}')
            
        except TransformException as ex:
            self.get_logger().warn(f'无法获取坐标转换: {ex}')
            # 如果转换失败，发布相机坐标系下的位姿
            self.target_pose_publisher.publish(camera_pose)
            self.get_logger().info('发布相机坐标系下的位姿')

def main():
    rclpy.init()
    node = TargetPosePublisher()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()