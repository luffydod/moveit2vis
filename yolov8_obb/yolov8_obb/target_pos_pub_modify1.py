#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from perception_interfaces.msg import Detection2DWithDepthArray
from geometry_msgs.msg import PoseStamped, Point
import numpy as np
import math
import tf2_ros
import tf2_geometry_msgs
from tf2_ros import TransformException
from geometry_msgs.msg import Quaternion
from geometry_msgs.msg import TransformStamped
from tf_transformations import quaternion_from_euler, quaternion_multiply

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
            f'{NAME_SPACE}/target_point',
            10)
        
        # 添加预抓取位姿发布器
        self.pre_grasp_pose_publisher = self.create_publisher(
            PoseStamped,
            f'{NAME_SPACE}/pre_grasp_pose',
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
        self.target_frame = "panda_link0"  # 目标坐标系（通常是机器人基座）
        
        self.get_logger().info('目标位姿发布节点已启动')

    def detection_callback(self, msg):
        """处理检测结果，提取3D位置并发布目标位姿"""
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
            # self.get_logger().warn(f'无效的深度值: {depth}')
            return
        
        # 步骤1: 将像素坐标转换为标准相机坐标系（Z轴指向前方，X轴向右，Y轴向下）
        # 注意：像素坐标系原点在左上角，而相机坐标系原点在中心
        x_std = (pixel_x - self.cx) * depth / self.fx  # 向右为正
        y_std = (pixel_y - self.cy) * depth / self.fy  # 向下为正
        z_std = depth                                   # 向前为正
        
        self.get_logger().info(f'标准相机坐标系: x={x_std:.3f}, y={y_std:.3f}, z={z_std:.3f}')
        
        # 步骤2：计算标准相机坐标系下的目标位姿
        
        # 创建目标位姿消息
        target_pose = PoseStamped()
        target_pose.header.stamp = self.get_clock().now().to_msg()
        target_pose.header.frame_id = self.camera_frame
        
        # 设置位置
        target_pose.pose.position.x = x_std
        target_pose.pose.position.y = y_std
        target_pose.pose.position.z = z_std
        
        # 计算四元数 - 根据theta角度构建绕z轴旋转theta的四元数
        q = quaternion_from_euler(0, 0, theta)
        target_pose.pose.orientation.x = q[0]
        target_pose.pose.orientation.y = q[1]
        target_pose.pose.orientation.z = q[2]
        target_pose.pose.orientation.w = q[3]
        
        # 尝试将位姿从相机坐标系转换到目标坐标系(机器人基座)
        try:
            # 获取从相机坐标系到目标坐标系的变换
            transform = self.tf_buffer.lookup_transform(
                self.target_frame,           # 目标坐标系
                self.camera_frame,           # 源坐标系
                rclpy.time.Time(),           # 获取最新可用的变换
                timeout=rclpy.duration.Duration(seconds=1.0))  # 超时时间
            
            # 使用tf2_geometry_msgs的do_transform_pose函数进行正确的坐标变换
            t_pose = target_pose.pose
            transformed_pose = PoseStamped()
            transformed_pose.header.stamp = self.get_clock().now().to_msg()
            transformed_pose.header.frame_id = self.target_frame
            
            transformed_pose.pose = tf2_geometry_msgs.do_transform_pose(t_pose, transform)
            
            # 发布转换后的目标位姿
            self.target_pose_publisher.publish(transformed_pose)
            
            # 计算垂直抓取姿态
            self.calculate_grasp_poses(transformed_pose)
            
            self.get_logger().info(f'目标坐标系下的位置: x={transformed_pose.pose.position.x:.3f}, ' +
                                  f'y={transformed_pose.pose.position.y:.3f}, z={transformed_pose.pose.position.z:.3f}')
            
        except TransformException as ex:
            self.get_logger().warn(f'无法获取坐标转换: {ex}')
            # 如果转换失败，仍然发布相机坐标系下的位姿
            self.target_pose_publisher.publish(target_pose)

    def calculate_grasp_poses(self, target_pose):
        """计算基于目标位姿的抓取姿态"""
        # 1. 计算预抓取位姿 (在物体上方)
        pre_grasp = PoseStamped()
        pre_grasp.header = target_pose.header
        pre_grasp.pose.position.x = target_pose.pose.position.x
        pre_grasp.pose.position.y = target_pose.pose.position.y
        pre_grasp.pose.position.z = target_pose.pose.position.z + 0.33  # 预抓取高度，在物体上方30cm
        
        q = quaternion_from_euler(math.pi/2, 0, 0)
        
        # pre_grasp.pose.orientation.x = q[0]
        # pre_grasp.pose.orientation.y = q[1]
        # pre_grasp.pose.orientation.z = q[2]
        # pre_grasp.pose.orientation.w = q[3]
        pre_grasp.pose.orientation.x = 0.0
        pre_grasp.pose.orientation.y = 0.0
        pre_grasp.pose.orientation.z = 0.0
        pre_grasp.pose.orientation.w = 1.0
        
        # 发布预抓取位姿
        self.pre_grasp_pose_publisher.publish(pre_grasp)
        self.get_logger().info(f'已发布预抓取位姿: 位置(x={pre_grasp.pose.position.x:.3f}, ' +
                              f'y={pre_grasp.pose.position.y:.3f}, z={pre_grasp.pose.position.z:.3f})')

def main():
    rclpy.init()
    node = TargetPosePublisher()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()