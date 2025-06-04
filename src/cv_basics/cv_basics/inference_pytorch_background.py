import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import os
import numpy as np
import onnxruntime as ort
from ament_index_python.packages import get_package_share_directory
import time

class YOLOv11Node(Node):
    def __init__(self):
        super().__init__('yolov11_node')
        
        # Получаем путь к весам модели
        package_share_directory = get_package_share_directory('cv_basics')
        onnx_path = os.path.join(package_share_directory, 'data', 'yolo11n.onnx')
        
        # Загружаем модель YOLOv11
        self.model = ort.InferenceSession(onnx_path, providers=['CPUExecutionProvider'])
        self.get_logger().info(f"Model loaded from {onnx_path}")
        
        # Анализируем входные параметры модели
        self.input_name = self.model.get_inputs()[0].name
        self.input_shape = self.model.get_inputs()[0].shape
        self.input_size = self.input_shape[2]  # Размер входа модели (640)
        
        self.output_name = self.model.get_outputs()[0].name
        self.output_shape = self.model.get_outputs()[0].shape
        self.get_logger().info(f"Input: {self.input_shape}, Output: {self.output_shape}")
        
        # Параметры обработки
        self.conf_threshold = 0.25
        self.iou_threshold = 0.45
        
        # ROS2 коммуникация
        self.publisher_ = self.create_publisher(Image, '/detections', 10)
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )
        self.bridge = CvBridge()
        self.latest_image = None
        self.get_logger().info(f"Node initialized. Input size: {self.input_size}")

    def image_callback(self, msg):
        try:
            # Конвертируем и сохраняем изображение
            self.latest_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            self.get_logger().info("Received new image", throttle_duration_sec=1.0)
        except Exception as e:
            self.get_logger().error(f"Image conversion error: {str(e)}")

    def preprocess(self, image):
        orig_h, orig_w = image.shape[:2]
        
        # Рассчитываем scale с плавающей точкой
        scale = min(self.input_size / orig_w, self.input_size / orig_h)
        new_w = int(round(orig_w * scale))
        new_h = int(round(orig_h * scale))
        
        # Изменяем размер
        resized = cv2.resize(image, (new_w, new_h), interpolation=cv2.INTER_LANCZOS4)
        
        # Рассчитываем смещения как float
        pad_w = self.input_size - new_w
        pad_h = self.input_size - new_h
        dw = pad_w / 2.0
        dh = pad_h / 2.0
        
        # Создаем холст и вставляем изображение
        canvas = np.full((self.input_size, self.input_size, 3), 114, dtype=np.uint8)
        top = int(round(dh))
        left = int(round(dw))
        canvas[top:top+new_h, left:left+new_w] = resized
        
        # Возвращаем float-смещения в meta
        rgb_image = cv2.cvtColor(canvas, cv2.COLOR_BGR2RGB)
        normalized = rgb_image.astype(np.float32) / 255.0
        blob = normalized.transpose(2, 0, 1)[np.newaxis]
        return blob, (scale, dw, dh, orig_w, orig_h)

    def postprocess(self, outputs, meta):
        """Постобработка для формата вывода YOLOv8 (1, 84, 8400)"""
        scale, dw, dh, orig_w, orig_h = meta
        
        # Получаем вывод модели
        predictions = outputs[0]  # Форма (1, 84, 8400)
        self.get_logger().info(f"Raw output shape: {predictions.shape}")
        
        # Транспонируем в формат (1, 8400, 84)
        predictions = predictions.transpose(0, 2, 1)  # Теперь (1, 8400, 84)
        predictions = predictions[0]  # Убираем размерность батча (8400, 84)
        
        # Разделяем на координаты и вероятности классов
        boxes = predictions[:, 0:4]  # [cx, cy, w, h] в координатах 640x640
        class_probs = predictions[:, 4:]  # Вероятности классов
        
        # Вычисляем уверенность для каждого класса
        max_scores = np.max(class_probs, axis=1)
        class_ids = np.argmax(class_probs, axis=1)
        
        # Фильтрация по уверенности
        conf_mask = max_scores > self.conf_threshold
        boxes = boxes[conf_mask]
        class_ids = class_ids[conf_mask]
        max_scores = max_scores[conf_mask]
        
        if boxes.shape[0] == 0:
            return []
        
        # Ключевое исправление: Преобразование координат
        # Конвертируем (cx, cy, w, h) в (xmin, ymin, xmax, ymax)
        boxes_transformed = np.zeros_like(boxes)
        boxes_transformed[:, 0] = boxes[:, 0] - boxes[:, 2] / 2  # xmin
        boxes_transformed[:, 1] = boxes[:, 1] - boxes[:, 3] / 2  # ymin
        boxes_transformed[:, 2] = boxes[:, 0] + boxes[:, 2] / 2  # xmax
        boxes_transformed[:, 3] = boxes[:, 1] + boxes[:, 3] / 2  # ymax
        
        # Масштабирование координат к исходному изображению
        boxes_transformed[:, [0, 2]] = (boxes_transformed[:, [0, 2]] - dw) / scale
        boxes_transformed[:, [1, 3]] = (boxes_transformed[:, [1, 3]] - dh) / scale
        
        # Ограничение координат
        boxes_transformed[:, [0, 2]] = np.clip(boxes_transformed[:, [0, 2]], 0, orig_w)
        boxes_transformed[:, [1, 3]] = np.clip(boxes_transformed[:, [1, 3]], 0, orig_h)
        
        # Преобразование в формат (x, y, width, height) для NMS
        boxes_xywh = np.zeros_like(boxes_transformed)
        boxes_xywh[:, 0] = boxes_transformed[:, 0]  # x
        boxes_xywh[:, 1] = boxes_transformed[:, 1]  # y
        boxes_xywh[:, 2] = boxes_transformed[:, 2] - boxes_transformed[:, 0]  # width
        boxes_xywh[:, 3] = boxes_transformed[:, 3] - boxes_transformed[:, 1]  # height
        
        # Non-Maximum Suppression (NMS)
        try:
            # Используем только валидные боксы
            valid_indices = []
            for i in range(boxes_xywh.shape[0]):
                w, h = boxes_xywh[i, 2], boxes_xywh[i, 3]
                if w > 1 and h > 1:  # Проверка минимального размера
                    valid_indices.append(i)
            
            if not valid_indices:
                return []
            
            valid_boxes = boxes_xywh[valid_indices]
            valid_scores = max_scores[valid_indices]
            
            indices = cv2.dnn.NMSBoxes(
                bboxes=valid_boxes.tolist(),
                scores=valid_scores.tolist(),
                score_threshold=self.conf_threshold,
                nms_threshold=self.iou_threshold
            )
        except Exception as e:
            self.get_logger().error(f"NMS error: {str(e)}")
            return []
        
        # Формируем детекции после NMS
        detections = []
        
        if indices is not None and len(indices) > 0:
            # Преобразование индексов в плоский массив целых чисел
            if isinstance(indices, np.ndarray):
                indices = indices.flatten()
            else:
                indices = np.array(indices).flatten()
            
            for idx in indices:
                idx = int(idx)
                i = valid_indices[idx]  # Оригинальный индекс
                xmin, ymin, xmax, ymax = boxes_transformed[i]
                
                # Проверка валидности бокса
                width = xmax - xmin
                height = ymax - ymin
                if width < 1 or height < 1:
                    self.get_logger().warn(f"Invalid box skipped: {xmin},{ymin},{xmax},{ymax}")
                    continue
                    
                detections.append((
                    xmin, ymin, xmax, ymax,
                    float(max_scores[i]),
                    int(class_ids[i])
                ))
        
        return detections

    def draw_detections(self, image, detections):
        """Визуализация детекций"""
        for det in detections:
            xmin, ymin, xmax, ymax, conf, class_id = det
            
            # Преобразуем координаты в целые числа
            xmin, ymin, xmax, ymax = int(xmin), int(ymin), int(xmax), int(ymax)
            
            # Пропускаем невалидные координаты
            if xmin >= xmax or ymin >= ymax:
                continue
                
            # Генерируем цвет на основе класса
            np.random.seed(class_id)
            color = tuple(np.random.randint(0, 255, size=3).tolist())
            
            # Рисуем bounding box
            cv2.rectangle(image, (xmin, ymin), (xmax, ymax), color, 2)
            
            # Рисуем фон для текста
            label = f"{class_id}: {conf:.2f}"
            (text_width, text_height), _ = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)
            
            # Корректировка положения текста
            text_y = max(ymin - 10, 10)
            cv2.rectangle(image, 
                         (xmin, text_y - text_height - 2), 
                         (xmin + text_width, text_y), 
                         color, -1)
            
            # Подпись
            cv2.putText(image, label, 
                       (xmin, text_y - 2), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, 
                       (255, 255, 255), 1)
        return image

    def process_image(self):
        if self.latest_image is None:
            return None
            
        try:
            # Сохраняем оригинал
            orig_image = self.latest_image.copy()
            
            # Препроцессинг
            blob, meta = self.preprocess(orig_image)
            
            # Вывод модели
            start_time = time.time()
            outputs = self.model.run([self.output_name], {self.input_name: blob})
            inference_time = time.time() - start_time
            self.get_logger().info(f"Inference time: {inference_time:.3f}s")
            
            # Постобработка
            detections = self.postprocess(outputs, meta)
            
            # Визуализация
            result_image = orig_image.copy()
            if detections:
                result_image = self.draw_detections(result_image, detections)
                self.get_logger().info(f"Detected {len(detections)} objects")
            else:
                # Яркое сообщение об отсутствии детекций
                cv2.putText(result_image, "No detections", (20, 40), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
            
            return result_image
        except Exception as e:
            self.get_logger().error(f"Processing error: {str(e)}", throttle_duration_sec=1.0)
            return None

    def timer_callback(self):
        if self.latest_image is None:
            self.get_logger().info("Waiting for images...", throttle_duration_sec=2.0)
            return
            
        result_image = self.process_image()
        if result_image is not None:
            try:
                # Правильная публикация изображения
                img_msg = self.bridge.cv2_to_imgmsg(result_image, "bgr8")
                img_msg.header.stamp = self.get_clock().now().to_msg()
                img_msg.header.frame_id = "camera_frame"
                self.publisher_.publish(img_msg)
                self.get_logger().info("Published detections", throttle_duration_sec=1.0)
            except Exception as e:
                self.get_logger().error(f"Publishing error: {str(e)}")

def main(args=None):
    rclpy.init(args=args)
    node = YOLOv11Node()
    node.create_timer(0.1, node.timer_callback)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()