import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os
from ultralytics import YOLO
from ament_index_python.packages import get_package_share_directory
from std_msgs.msg import Float32MultiArray

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
        # Подписка на топик с изображениями
        self.subscription = self.create_subscription(Image, '/camera/image_raw', self.listener_callback, 30)
        # Таймер для периодической публикации результатов (0.5 с, можно настроить под себя)
        self.timer = self.create_timer(0.5, self.timer_callback)

        # Паблишер для публикации боксов YOLO
        self.yolo_pub = self.create_publisher(Float32MultiArray, 'yolo_boxes', 10)
        
        # Объект для конвертации между ROS Image и OpenCV изображениями
        self.br = CvBridge()
        # Хранение последнего полученного изображения
        self.cv_image = None

    def listener_callback(self, msg: Image):
        try:
            self.cv_image = self.br.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error('Failed to convert image message to OpenCV: {}'.format(str(e)))

    def publish_yolo_boxes(self, boxes):
        msg = Float32MultiArray()
        data = []
        for box in boxes:
            coords = box.xyxy[0].tolist()
            class_id = float(box.cls[0]) 
            confidence = float(box.conf[0])
            x_center = float((coords[0] + coords[2]) / 2)
            y_center = float((coords[1] + coords[3]) / 2)
            data.extend([x_center, y_center, class_id, confidence])
        msg.data = data
        self.yolo_pub.publish(msg)
        
    def inference_yolo_background(self):
        if self.cv_image is None:
            return None
        
        # Создаем копию, чтобы не влиять на оригинальное изображение
        current_image = self.cv_image.copy()
        # Обратите внимание: conf должен быть значением между 0 и 1, например, 0.5
        results = self.model.predict(
            current_image,
            conf=0.5,
            save=False,
            save_txt=False,
            verbose=False
        )
        all_boxes = []
        for result in results:
            boxes = result.boxes
            for box in boxes:
                all_boxes.append(box)
            
        self.publish_yolo_boxes(all_boxes)
        # Пример обработки: получение аннотированного изображения
        frame_with_predict = results[0].plot()  # или любая другая обработка
        return frame_with_predict

    def timer_callback(self):
        frame_with_result = self.inference_yolo_background()
        if frame_with_result is not None:
            img_msg = self.br.cv2_to_imgmsg(frame_with_result)
            self.publisher_.publish(img_msg)
            self.get_logger().info('Pub frame')
        

def main(args=None):
    rclpy.init(args=args)
    node = MyNodes()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
  
if __name__ == '__main__':
    main()