#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import os

class DepthImageSaver(Node):
    def __init__(self):
        super().__init__('depth_image_saver')
        self.bridge = CvBridge()
        
        # 订阅深度图像话题
        self.subscription = self.create_subscription(
            Image,
            '/camera/depth/image_raw',
            self.depth_callback,
            1)
        
        self.get_logger().info("深度图像保存节点已启动，等待接收深度图像...")
        
        # 创建保存目录
        os.makedirs("depth_images", exist_ok=True)

    def depth_callback(self, msg):
        try:
            # 将ROS图像消息转换为OpenCV格式
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            
            # 检查是否为深度图像 (16位整数或32位浮点数)
            if cv_image.dtype == np.uint16:
                # 16位深度图直接保存
                filename = "depth_16bit.png"
                cv2.imwrite(filename, cv_image)
                self.get_logger().info(f"保存了16位深度图: {filename}")
            
            elif cv_image.dtype == np.float32:
                # 保存为浮点TIFF (保持精度)
                filename = "depth_32float.tiff"
                cv2.imwrite(filename, cv_image)
                self.get_logger().info(f"保存了32位浮点深度图: {filename}")
            else:
                self.get_logger().error(f"不支持的深度图像格式: {cv_image.dtype}")
                return
            
        except Exception as e:
            self.get_logger().error(f"处理深度图像时出错: {str(e)}")

def main(args=None):
    rclpy.init(args=args)
    node = DepthImageSaver()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()