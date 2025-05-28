#!/usr/bin/env python3

from ultralytics import YOLO
import os
import copy
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from yolov8_msgs.msg import InferenceResult
from yolov8_msgs.msg import Yolov8Inference
import message_filters  # 新增消息过滤器
import numpy as np
bridge = CvBridge()

class Camera_subscriber(Node):

    def __init__(self):
        super().__init__('camera_subscriber')

        self.model = YOLO(os.environ['HOME'] + '/moveit2_yolobb_ws/src/yolov8_obb/scripts/best.pt')
        self.yolov8_inference = Yolov8Inference()

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
        self.yolov8_pub = self.create_publisher(Yolov8Inference, "/Yolov8_Inference", 1)
        self.img_pub = self.create_publisher(Image, "/inference_result", 1)
    '''
    def camera_callback(self, data):
    '''
    
    def sync_callback(self, rgb_msg, depth_msg):
        """同步处理RGB和深度图像的回调函数"""
        rgb_img = bridge.imgmsg_to_cv2(rgb_msg, "bgr8")
        depth_img = bridge.imgmsg_to_cv2(depth_msg, desired_encoding="passthrough")
        
        #img = bridge.imgmsg_to_cv2(data, "bgr8")
        results = self.model(rgb_img, conf = 0.90)

        #self.yolov8_inference.header.frame_id = "inference"
        self.yolov8_inference.header.stamp = camera_subscriber.get_clock().now().to_msg()

        for r in results:
            if(r.obb is not None):
                boxes = r.obb
                for box in boxes:
                    self.inference_result = InferenceResult()
                    b = box.xyxyxyxy[0].to('cpu').detach().numpy().copy()

                    # 重塑为四个点 (x, y)
                    points = b.reshape(4, 2)

                    # 计算中心点（直接取均值）
                    center = points.mean(axis=0)
                    cx=center[0]
                    cy=center[1]

                    x_floor = int(np.floor(cx))
                    y_floor = int(np.floor(cy))
                    x_ceil = x_floor + 1
                    y_ceil = y_floor + 1
                    
                    # 检查边界
                    if x_floor < 0 or y_floor < 0 or x_ceil >= depth_img.shape[1] or y_ceil >= depth_img.shape[0]:
                        return np.nan
                    
                    # 计算权重
                    dx = cx - x_floor
                    dy = cy - y_floor
                    
                    # 四个相邻像素的深度值
                    top_left = depth_img[y_floor, x_floor]
                    top_right = depth_img[y_floor, x_ceil]
                    bottom_left = depth_img[y_ceil, x_floor]
                    bottom_right = depth_img[y_ceil, x_ceil]
                    
                    # 双线性插值公式
                    depth_value = (1 - dx) * (1 - dy) * top_left + \
                                   dx * (1 - dy) * top_right + \
                                   (1 - dx) * dy * bottom_left + \
                                   dx * dy * bottom_right
                    
                    # 提取深度值（处理无效值）
                    #depth_value = depth_img[cy, cx] if (0 <= cx < depth_img.shape[1] and 0 <= cy < depth_img.shape[0]) else np.nan
                    #depth_value = bilinear_interpolation(depth_img, cx, cy)
 
                    # 打印深度信息
                    self.get_logger().info(
                        f"Depth: {depth_value:.2f}m"
                    )
                    c = box.cls
                    self.inference_result.class_name = self.model.names[int(c)]
                    #self.inference_result.confidence = float(box.conf)
                    a = b.reshape(1,8)
                    self.inference_result.coordinates = copy.copy(a[0].tolist())
                    self.inference_result.depth_value = np.float64(round(depth_value,2))
                    self.yolov8_inference.yolov8_inference.append(self.inference_result)
            else:
                camera_subscriber.get_logger().info(f"no_results")

        self.yolov8_pub.publish(self.yolov8_inference)
        self.yolov8_inference.yolov8_inference.clear()

        # 发布带标注的图像
        annotated_frame = results[0].plot(boxes=True)
        self.img_pub.publish(bridge.cv2_to_imgmsg(annotated_frame))

        
        '''
        annotated_frame = results[0].plot()
        img_msg = bridge.cv2_to_imgmsg(annotated_frame)  
        self.img_pub.publish(img_msg)
        '''
if __name__ == '__main__':
    rclpy.init(args=None)
    camera_subscriber = Camera_subscriber()
    rclpy.spin(camera_subscriber)
    rclpy.shutdown()
