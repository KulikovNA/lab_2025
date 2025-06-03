#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist, Point
from visualization_msgs.msg import Marker
from cv_bridge import CvBridge
import cv2
import os
import numpy as np
from cv_basics.sort import Sort
from ultralytics import YOLO

from ament_index_python.packages import get_package_share_directory

class ObjectTracker(Node):
    def __init__(self):
        super().__init__('object_tracker')
        self.bridge = CvBridge()
        self.tracker = Sort(max_age=20, min_hits=3, iou_threshold=0.3)

        package_share_directory = get_package_share_directory('cv_basics')
        weights_path = os.path.join(package_share_directory, 'data', 'yolo11n.pt')
        self.model = YOLO(weights_path)
        
        # Подписки/публикации
        self.image_sub = self.create_subscription(
            Image, '/camera/image_raw', self.image_callback, 10)
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.marker_pub = self.create_publisher(Marker, '/object_marker', 10)
        self.obj_pose_pub = self.create_publisher(Point, '/object_pose', 10)
        
        # Параметры управления
        self.kp = 0.05
        self.kd = 0.001
        self.target_class = 'person'
        self.frame_width = 640
        self.frame_height = 480
        self.frame_center_x = self.frame_width // 2
        self.frame_center_y = self.frame_height // 2
        self.last_error = 0
        self.last_time = self.get_clock().now()
        
        # Для визуализации
        self.marker_id = 0

    def image_callback(self, msg):
        try:
            current_time = self.get_clock().now()
            dt = (current_time - self.last_time).nanoseconds / 1e9
            self.last_time = current_time
            
            cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
            
            # Детекция объектов
            results = self.model.predict(
                cv_image,
                conf=0.6,
                classes=[0],  # Только люди
                verbose=False
            )
            
            # Формируем детекции для SORT
            detections = []
            for result in results:
                for box in result.boxes:
                    if int(box.cls) == 0:  # Проверяем класс 'person'
                        x1, y1, x2, y2 = map(int, box.xyxy[0])
                        conf = float(box.conf)
                        detections.append([x1, y1, x2, y2, conf])
            
            # Обновляем трекер (с обработкой пустого массива)
            if len(detections) > 0:
                tracked_objects = self.tracker.update(np.array(detections))
            else:
                tracked_objects = np.empty((0, 5))
            
            if tracked_objects.shape[0] > 0:
                # Выбираем объект с наибольшей площадью
                main_obj = max(tracked_objects, key=lambda x: (x[2]-x[0])*(x[3]-x[1]))
                
                # Визуализация
                x1, y1, x2, y2 = map(int, main_obj[:4])
                cv2.rectangle(cv_image, (x1, y1), (x2, y2), (0, 255, 0), 2)
                cv2.putText(cv_image, "PERSON", (x1, y1-10),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0,255,0), 2)
                
                # Управление платформой
                self.control_platform(main_obj, dt)
                self.publish_marker(main_obj)
                self.publish_object_pose(main_obj)
            else:
                self.stop_platform()
            
            # Отображение центра
            cv2.circle(cv_image, (self.frame_center_x, self.frame_center_y), 5, (0,0,255), -1)
            #cv2.imshow("Person Tracking", cv_image)
            #cv2.waitKey(1)

        except Exception as e:
            self.get_logger().error(f'Error: {str(e)}')
            self.stop_platform()

    def control_platform(self, bbox, dt):
        cmd = Twist()
        object_center_x = (bbox[0] + bbox[2]) / 2
        error = (object_center_x - self.frame_center_x) / self.frame_center_x
        
        # PD-регулятор
        derivative = (error - self.last_error) / dt if dt > 0 else 0
        cmd.angular.z = -self.kp * error - self.kd * derivative
        
        # Ограничение скорости
        cmd.angular.z = np.clip(cmd.angular.z, -1.5, 1.5)
        
        self.last_error = error
        self.cmd_vel_pub.publish(cmd)
        
        # Отладочный вывод
        self.get_logger().info(
            f"Error: {error:.2f}, Speed: {cmd.angular.z:.2f}",
            throttle_duration_sec=0.5
        )

    def publish_marker(self, bbox):
        marker = Marker()
        marker.header.frame_id = "base_link"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.id = self.marker_id
        self.marker_id += 1
        
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        
        # Позиция маркера
        marker.pose.position.x = 1.0  # 1 метр перед камерой
        marker.pose.position.y = -(bbox[0] + bbox[2] - self.frame_width) / self.frame_width
        marker.pose.position.z = 0.0
        
        marker.scale.x = 0.2
        marker.scale.y = 0.2
        marker.scale.z = 0.2
        
        marker.color.g = 1.0
        marker.color.a = 1.0
        
        #self.marker_pub.publish(marker)

    def publish_object_pose(self, bbox):
        pose = Point()
        pose.x = (bbox[0] + bbox[2]) / 2  # X-координата центра
        pose.y = (bbox[1] + bbox[3]) / 2  # Y-координата центра
        #self.obj_pose_pub.publish(pose)

    def stop_platform(self):
        cmd = Twist()
        cmd.angular.z = 0.0
        self.cmd_vel_pub.publish(cmd)
        self.last_error = 0

def main(args=None):
    rclpy.init(args=args)
    node = ObjectTracker()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()