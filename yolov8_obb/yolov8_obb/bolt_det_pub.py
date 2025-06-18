#!/usr/bin/env python3

from ultralytics import YOLO
import os
import copy
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from perception_interfaces.msg import Detection2DWithDepth, Detection2DWithDepthArray

import message_filters  # 新增消息过滤器
import numpy as np
from ament_index_python.packages import get_package_share_directory
import os
bridge = CvBridge()

NAME_SPACE = "/perception"

"""
(0,0) ------ x (width/columns) ---> shape[1]
  |
  |
  y (height/rows)
  |
  |
  v
shape[0]
"""

class BoltDetection(Node):

    def __init__(self):
        super().__init__('bolt_detection_node')

        # Get the package directory and build path to the model
        package_share_directory = get_package_share_directory('yolov8_obb')
        bolt_model_path = os.path.join(package_share_directory, 'ckpt', 'bolt_det.pt')
        self.model = YOLO(bolt_model_path)
        self.det2dd_array = Detection2DWithDepthArray()

         # 创建同步订阅器
        rgb_sub = message_filters.Subscriber(self, Image, '/image_raw')
        depth_sub = message_filters.Subscriber(self, Image, '/camera/depth/image_raw')
        
        # 使用近似时间同步（时间差0.1秒）
        self.ts = message_filters.ApproximateTimeSynchronizer(
            [rgb_sub, depth_sub], queue_size=10, slop=0.1)
        self.ts.registerCallback(self.sync_callback)  # 替换原有回调
        
        '''
        self.subscription = self.create_subscription(
            Image,
            '/image_raw',
            self.camera_callback,
            10)
        self.subscription 
        '''
        self.det2dd_pub = self.create_publisher(Detection2DWithDepthArray, f"{NAME_SPACE}/det_2d_d", 1)
        self.img_pub = self.create_publisher(Image, f"{NAME_SPACE}/det_img", 1)
    '''
    def camera_callback(self, data):
    '''
    @staticmethod
    def compute_center_depth(depth_img, cx, cy):
        """计算目标中心点的深度值
        Args:
            depth_img: 深度图像
            cx, cy: 目标中心点坐标
        Returns:
            depth_value: 深度值（米）
        """
        # 使用更大的窗口以获取更稳定的深度估计
        window_size = 11  # 增大窗口尺寸
        half_size = window_size // 2

        # 获取中心点周围的窗口
        x_start = max(0, int(cx) - half_size)
        y_start = max(0, int(cy) - half_size)
        x_end = min(depth_img.shape[1], int(cx) + half_size + 1)
        y_end = min(depth_img.shape[0], int(cy) + half_size + 1)

        # 提取窗口区域
        window = depth_img[y_start:y_end, x_start:x_end]

        # 统计分析
        valid_depths = window[window > 0.01]  # 过滤无效值
        if len(valid_depths) > 0:
            # 计算统计值
            median_depth = np.median(valid_depths)
            mean_depth = np.mean(valid_depths)
            std_depth = np.std(valid_depths)

            # 使用3-sigma原则过滤异常值
            valid_mask = np.abs(valid_depths - median_depth) < 3 * std_depth
            filtered_depths = valid_depths[valid_mask]

            if len(filtered_depths) > 0:
                # 如果深度值差异过大，使用中值而不是平均值
                if std_depth / median_depth > 0.1:  # 10%的变异系数
                    depth_value = np.median(filtered_depths)
                else:
                    depth_value = np.mean(filtered_depths)
                
                # 限制深度值范围（根据实际应用场景调整）
                depth_value = np.clip(depth_value, 0.1, 10.0)
                return round(depth_value, 3)
        
        return np.nan
    
    def sync_callback(self, rgb_msg, depth_msg):
        """同步处理RGB和深度图像的回调函数"""
        rgb_img = bridge.imgmsg_to_cv2(rgb_msg, "bgr8")
        depth_img = bridge.imgmsg_to_cv2(depth_msg, desired_encoding="passthrough")
        
        results = self.model(rgb_img, conf=0.90, verbose=False)

        self.det2dd_array.header.stamp = self.get_clock().now().to_msg()

        for res in results:
            if(res.obb is not None):
                boxes = res.obb
                for box in boxes:
                    self.det2dd = Detection2DWithDepth()
                    # 直接使用YOLO输出的有序点集
                    points = box.xyxyxyxy[0].to('cpu').detach().numpy().copy().reshape(4, 2)
                    
                    # 计算中心点
                    center = np.mean(points, axis=0)
                    cx, cy = center[0], center[1]

                    # 使用YOLO给出的第一条边作为主方向（通常是长边）
                    dx = points[1][0] - points[0][0]
                    dy = points[1][1] - points[0][1]
                    theta = np.arctan2(dy, dx)
                    
                    # 计算宽度和高度
                    width = np.linalg.norm(points[1] - points[0])
                    height = np.linalg.norm(points[2] - points[1])

                    # 如果需要限制角度范围
                    if theta > np.pi/2:
                        theta -= np.pi
                    elif theta < -np.pi/2:
                        theta += np.pi
                    
                    # self.get_logger().info(f"计算得到的方向角: {theta:.4f} 弧度 ({theta*180/np.pi:.2f}°)")

                    self.det2dd.detection.id = str(0)
                    self.det2dd.detection.bbox.center.position.x = np.float64(cx)
                    self.det2dd.detection.bbox.center.position.y = np.float64(cy)
                    self.det2dd.detection.bbox.center.theta = np.float64(theta)
                    self.det2dd.detection.bbox.size_x = np.float64(width)
                    self.det2dd.detection.bbox.size_y = np.float64(height)
                    depth_value = self.compute_center_depth(depth_img, cx, cy)
                    if not np.isnan(depth_value):
                        # self.get_logger().info(f"深度值计算成功: {depth_value:.3f} 米")
                        # 添加深度值的可靠性评估
                        window = depth_img[max(0, int(cy)-5):min(depth_img.shape[0], int(cy)+6),
                                         max(0, int(cx)-5):min(depth_img.shape[1], int(cx)+6)]
                        valid_ratio = np.sum(window > 0.01) / window.size
                        # self.get_logger().info(f"深度值有效率: {valid_ratio:.2%}")
                    else:
                        # self.get_logger().warn("深度值计算失败，可能原因：区域内无有效深度值")
                        depth_value = 0.0
                    self.det2dd.depth_center = np.float64(depth_value)
                    self.det2dd_array.detections.append(self.det2dd)
            else:
                self.get_logger().info(f"No OBB detected in the result.")
                
        # 发布检测结果
        self.det2dd_pub.publish(self.det2dd_array)

        # 清空检测数组
        self.det2dd_array.detections.clear()

        # 发布带标注的图像
        annotated_frame = results[0].plot(boxes=True)
        self.img_pub.publish(bridge.cv2_to_imgmsg(annotated_frame))
        
def main():
    rclpy.init(args=None)
    bolt_det = BoltDetection()
    rclpy.spin(bolt_det)
    rclpy.shutdown()    
if __name__ == '__main__':
    main()