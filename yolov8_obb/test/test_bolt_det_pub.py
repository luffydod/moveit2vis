#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
from perception_interfaces.msg import Detection2DWithDepthArray
import os
from ament_index_python.packages import get_package_share_directory

bridge = CvBridge()

class BoltDetectionTest(Node):
    def __init__(self):
        super().__init__('bolt_detection_test_node')
        
        # 创建图像发布器
        self.rgb_pub = self.create_publisher(Image, '/image_raw', 10)
        self.depth_pub = self.create_publisher(Image, '/camera/depth/image_raw', 10)
        
        # 创建检测结果订阅器
        self.det_sub = self.create_subscription(
            Detection2DWithDepthArray,
            '/perception/det_2d_d',
            self.detection_callback,
            10
        )
        
        # 读取测试图像
        package_share_directory = get_package_share_directory('yolov8_obb')
        rgb_img_path = os.path.join(package_share_directory, 'img', 'bolt_img_raw.png')
        depth_img_path = os.path.join(package_share_directory, 'img', 'bolt_depth_32float.tiff')
        self.rgb_img = cv2.imread(rgb_img_path)
        self.depth_img = cv2.imread(depth_img_path, cv2.IMREAD_UNCHANGED)
        
        if self.rgb_img is None or self.depth_img is None:
            self.get_logger().error('无法读取测试图像，请检查文件路径')
            return
            
        # 创建定时器，定期发布图像
        self.timer = self.create_timer(1.0, self.timer_callback)
        
    def timer_callback(self):
        """定时发布RGB和深度图像"""
        # 发布RGB图像
        rgb_msg = bridge.cv2_to_imgmsg(self.rgb_img, encoding='bgr8')
        self.rgb_pub.publish(rgb_msg)
        
        # 发布深度图像
        depth_msg = bridge.cv2_to_imgmsg(self.depth_img, encoding='passthrough')
        self.depth_pub.publish(depth_msg)
        
    def detection_callback(self, msg):
        """处理检测结果的回调函数"""
        # 创建可视化图像的副本
        vis_img = self.rgb_img.copy()
        
        # 添加坐标轴刻度
        height, width = vis_img.shape[:2]
        # 每100像素绘制一个刻度
        for x in range(0, width, 100):
            cv2.putText(vis_img, str(x), (x, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
            cv2.line(vis_img, (x, 0), (x, 10), (0, 255, 0), 1)
            
        for y in range(0, height, 100):
            cv2.putText(vis_img, str(y), (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
            cv2.line(vis_img, (0, y), (10, y), (0, 255, 0), 1)
            
        for detection in msg.detections:
            # 获取检测结果
            cx = int(detection.detection.bbox.center.position.x)
            cy = int(detection.detection.bbox.center.position.y)
            theta = float(detection.detection.bbox.center.theta)
            width = float(detection.detection.bbox.size_x)
            height = float(detection.detection.bbox.size_y)
            depth = float(detection.depth_center)
            
            # 绘制中心点
            cv2.circle(vis_img, (cx, cy), 5, (0, 0, 255), -1)
            
            # 绘制方向箭头
            arrow_length = 50
            end_x = int(cx + arrow_length * np.cos(theta))
            end_y = int(cy + arrow_length * np.sin(theta))
            cv2.arrowedLine(vis_img, (cx, cy), (end_x, end_y), (255, 0, 0), 2)
            
            # 添加文本信息
            cv2.putText(vis_img, f'Center: ({cx}, {cy})', (cx+10, cy-20), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
            cv2.putText(vis_img, f'Angle: {theta*180/np.pi:.1f}deg', (cx+10, cy), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
            cv2.putText(vis_img, f'Depth: {depth:.3f}m', (cx+10, cy+20), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
            
        # 显示结果
        cv2.imshow('Detection Results', vis_img)
        cv2.waitKey(1)

def main():
    rclpy.init()
    node = BoltDetectionTest()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()