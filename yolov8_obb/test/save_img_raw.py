import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os
class ImageSaver(Node):
    def __init__(self):
        super().__init__('image_saver_node')
        # 订阅图像话题，这里以/camera/image_raw为例
        self.subscription = self.create_subscription(
            Image,
            '/image_raw',
            self.listener_callback,
            1)
        self.br = CvBridge()
        self.get_logger().info("Image saver node started...")

    def listener_callback(self, msg):
        try:
            # 将ROS图像消息转换为OpenCV图像
            current_frame = self.br.imgmsg_to_cv2(msg, "bgr8")
        except Exception as e:
            self.get_logger().error('Error converting image: %s' % str(e))
            return

        # 保存图像
        filename = f"image_{msg.header.stamp.sec}_{msg.header.stamp.nanosec}.png"
        cv2.imwrite(filename, current_frame)
        self.get_logger().info(f"Saved image: {filename}")

def main(args=None):
    rclpy.init(args=args)
    image_saver_node = ImageSaver()
    try:
        rclpy.spin(image_saver_node)
    except KeyboardInterrupt:
        image_saver_node.get_logger().info('Node stopped by keyboard interrupt')
    finally:
        image_saver_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()