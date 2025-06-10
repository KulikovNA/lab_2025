#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ultralytics import YOLO
import cv2

class YOLONode(Node):
    def __init__(self):
        super().__init__('inference_yolo_node')
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',  # Измените на ваш топик
            self.image_callback,
            10)
        self.bridge = CvBridge()
        self.model = YOLO('yolov8n.pt')  # Автозагрузка модели (сохраняется в ~/.cache/ultralytics)
        self.get_logger().info("YOLO node started")

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            results = self.model(cv_image)  # Инференс
            
            # Визуализация (опционально)
            annotated_frame = results[0].plot()
            cv2.imshow('YOLO Inference', annotated_frame)
            cv2.waitKey(1)
            
        except Exception as e:
            self.get_logger().error(f'Error: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = YOLONode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
