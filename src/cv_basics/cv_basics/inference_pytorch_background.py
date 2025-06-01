import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose
from geometry_msgs.msg import Pose2D
from cv_bridge import CvBridge
import cv2
import os
import numpy as np
import onnxruntime as ort
from ament_index_python.packages import get_package_share_directory

class YOLOONNXNode(Node):
    def __init__(self):
        super().__init__('yolov11_onnx_node')
        
        # Получаем путь к ONNX модели
        package_share_dir = get_package_share_directory('cv_basics')
        onnx_path = os.path.join(package_share_dir, 'data', 'yolo11n.onnx')
        
        # Инициализация ONNX Runtime
        self.session = ort.InferenceSession(onnx_path)
        self.input_name = self.session.get_inputs()[0].name
        
        # Параметры модели
        self.input_size = (640, 640)  # YOLO input size
        self.conf_threshold = 0.5
        
        # Инициализация ROS
        self.subscription = self.create_subscription(
            Image, 
            '/camera/image_raw', 
            self.image_callback, 
            10
        )
        self.detection_pub = self.create_publisher(
            Detection2DArray, 
            '/detections', 
            10
        )
        self.br = CvBridge()
        self.get_logger().info('ONNX node initialized with detection publishing')

    def preprocess(self, image):
        # Преобразование изображения для модели
        image = cv2.resize(image, self.input_size)
        image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)  # Конвертация в RGB
        image = image.transpose((2, 0, 1))  # HWC to CHW
        image = np.expand_dims(image, axis=0).astype(np.float32) / 255.0
        return image

    def postprocess(self, outputs, original_image):
        # Обработка выходных данных YOLO
        predictions = outputs[0].squeeze()
        boxes = []
        confidences = []
        class_ids = []
        
        # Получаем размеры оригинального изображения
        orig_height, orig_width = original_image.shape[:2]
        
        # Коэффициенты масштабирования
        width_scale = orig_width / self.input_size[0]
        height_scale = orig_height / self.input_size[1]
        
        # Фильтрация результатов
        for pred in predictions:
            confidence = pred[4]
            if confidence > self.conf_threshold:
                # Извлечение координат (в нормализованном формате)
                cx, cy, w, h = pred[:4]
                
                # Конвертация в абсолютные координаты оригинального изображения
                x1 = int((cx - w/2) * width_scale)
                y1 = int((cy - h/2) * height_scale)
                x2 = int((cx + w/2) * width_scale)
                y2 = int((cy + h/2) * height_scale)
                
                # Ограничение координат в пределах изображения
                x1 = max(0, min(x1, orig_width))
                y1 = max(0, min(y1, orig_height))
                x2 = max(0, min(x2, orig_width))
                y2 = max(0, min(y2, orig_height))
                
                class_id = np.argmax(pred[5:])
                boxes.append([x1, y1, x2, y2])
                confidences.append(float(confidence))
                class_ids.append(class_id)
        
        # Применение Non-Max Suppression
        indices = cv2.dnn.NMSBoxes(boxes, confidences, self.conf_threshold, 0.4)
        
        # Создаем сообщение Detection2DArray
        detection_array = Detection2DArray()
        
        # Открываем файл для записи детекций
        with open("detections.txt", "w") as f:
            # Добавляем детекции
            for i in indices:
                # Обработка разных форматов возврата NMSBoxes
                if isinstance(i, np.ndarray) or isinstance(i, list):
                    idx = i[0]
                else:
                    idx = i
                    
                box = boxes[idx]
                confidence = confidences[idx]
                class_id = class_ids[idx]
                
                # Создаем Detection2D
                detection = Detection2D()
                
                # Заполняем bounding box
                detection.bbox.center.position.x = (box[0] + box[2]) / 2.0
                detection.bbox.center.position.y = (box[1] + box[3]) / 2.0
                detection.bbox.size_x = float(box[2] - box[0])
                detection.bbox.size_y = float(box[3] - box[1])
                
                # Создаем гипотезу
                hypothesis = ObjectHypothesisWithPose()
                hypothesis.hypothesis.class_id = str(class_id)
                hypothesis.hypothesis.score = confidence
                
                # Добавляем гипотезу в детекцию
                detection.results.append(hypothesis)
                
                # Информация о детекции
                info = f"Class: {hypothesis.hypothesis.class_id}, Confidence: {confidence:.2f}, Bounding Box: ({detection.bbox.center.position.x}, {detection.bbox.center.position.y}, {detection.bbox.size_x}, {detection.bbox.size_y})"
                
                # Вывод в терминал
                print(info)
                
                # Запись в файл
                f.write(info + "\n")
                
                # Добавляем детекцию в массив
                detection_array.detections.append(detection)
        
        return detection_array

    def image_callback(self, msg):
        try:
            # Конвертация ROS Image -> OpenCV
            cv_image = self.br.imgmsg_to_cv2(msg, "bgr8")
            
            # Препроцессинг
            input_tensor = self.preprocess(cv_image)
            
            # Инференс
            outputs = self.session.run(None, {self.input_name: input_tensor})
            
            # Постпроцессинг
            detection_array = self.postprocess(outputs, cv_image)
            detection_array.header = msg.header
            
            # Публикация результатов
            self.detection_pub.publish(detection_array)
            self.get_logger().info(f'Published {len(detection_array.detections)} detections')
            
        except Exception as e:
            self.get_logger().error(f'Processing error: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    node = YOLOONNXNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()