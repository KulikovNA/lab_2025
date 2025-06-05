import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Int32MultiArray
from cv_bridge import CvBridge
import cv2
import os
from ultralytics import YOLO
from ament_index_python.packages import get_package_share_directory

class MyNodes(Node):
    def __init__(self):
        super().__init__('yolov11_node')
         
        # Получаем путь к весам
        package_share_directory = get_package_share_directory('cv_basics')
        weights_path = os.path.join(package_share_directory, 'data', 'yolo11n.pt')
        
        # Загружаем модель YOLOv11
        self.model = YOLO(weights_path)
        self.model.model.to('cpu')
        
        # Создаем паблишер для публикации изображений с предиктами
        self.publisher_ = self.create_publisher(Image, 'video_with_predict', 30)
        self.pub_center = self.create_publisher(Int32MultiArray, '/object_center', 10)
        # Подписка на топик с изображениями
        self.subscription = self.create_subscription(Image, '/camera/image_raw', self.listener_callback, 30)
        # Таймер для периодической публикации результатов (0.5 с, можно настроить под себя)
        self.timer = self.create_timer(0.5, self.timer_callback)

        # Объект для конвертации между ROS Image и OpenCV изображениями
        self.br = CvBridge()
        # Хранение последнего полученного изображения
        self.cv_image = None

    def listener_callback(self, msg: Image):
        try:
            self.cv_image = self.br.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error('Failed to convert image message to OpenCV: {}'.format(str(e)))
            self.cv_image = None

    def inference_yolo_background(self):
        if self.cv_image is None:
            return None, None
        
        current_image = self.cv_image.copy()
        results = self.model.predict(
            current_image,
            conf=0.5,
            save=False,
            save_txt=False,
            verbose=False
        )

        if len(results) == 0 or results[0].boxes.xyxy is None or len(results[0].boxes.xyxy) == 0:
            return current_image, None

        boxes = results[0].boxes
        classes = boxes.cls.cpu().numpy()
        xyxy = boxes.xyxy.cpu().numpy()

        # Ищем только людей (class == 0)
        person_indices = [i for i, c in enumerate(classes) if int(c) == 0]
        if len(person_indices) == 0:
            return current_image, None

        img_h, img_w, _ = current_image.shape
        center_x_frame = img_w / 2
        center_y_frame = img_h / 2

        # Выбираем ближайшего к центру кадра
        best_i = None
        best_distance = float('inf')
        best_cx, best_cy = None, None

        for i in person_indices:
            x1, y1, x2, y2 = xyxy[i]
            cx = (x1 + x2) / 2
            cy = (y1 + y2) / 2
            dist = ((cx - center_x_frame) ** 2 + (cy - center_y_frame) ** 2) ** 0.5
            if dist < best_distance:
                best_distance = dist
                best_i = i
                best_cx, best_cy = int(cx), int(cy)

        if best_i is not None:
            frame_with_predict = results[0].plot()
            return frame_with_predict, (best_cx, best_cy)
        else:
            return current_image, None



    def timer_callback(self):
        frame_with_result, center = self.inference_yolo_background()
        if frame_with_result is not None:
            img_msg = self.br.cv2_to_imgmsg(frame_with_result)
            self.publisher_.publish(img_msg)
            self.get_logger().info('Pub frame')
            
        if center is not None:
            msg = Int32MultiArray()
            msg.data = [center[0], center[1]]
            self.pub_center.publish(msg)
            self.get_logger().info(f'Published object center at: {center}')
        

def main(args=None):
    rclpy.init(args=args)
    node = MyNodes()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
  
if __name__ == '__main__':
    main()
