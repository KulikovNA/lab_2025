#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import torch
from torchvision import transforms

class SegmentationNode(Node):
    def __init__(self):
        super().__init__('segmentation_node')

        # Параметры
        self.declare_parameter('model_path', '/path/to/your/model.pth')
        self.declare_parameter('input_topic', '/camera/image_raw')
        self.declare_parameter('output_topic', '/segmentation/mask')
        self.declare_parameter('device', 'cuda' if torch.cuda.is_available() else 'cpu')

        # Загрузка параметров
        model_path = self.get_parameter('model_path').get_parameter_value().string_value
        input_topic = self.get_parameter('input_topic').get_parameter_value().string_value
        output_topic = self.get_parameter('output_topic').get_parameter_value().string_value
        self.device = self.get_parameter('device').get_parameter_value().string_value

        # Инициализация модели (замените на вашу модель)
        self.model = self.load_model(model_path)
        self.model.eval()
        self.model.to(self.device)

        # CvBridge для преобразования ROS Image <-> OpenCV
        self.bridge = CvBridge()

        # Подписка на входной топик с изображениями
        self.subscription = self.create_subscription(
            Image,
            input_topic,
            self.image_callback,
            10
        )

        # Публикация результата сегментации
        self.publisher = self.create_publisher(
            Image,
            output_topic,
            10
        )

        # Преобразование для входных данных модели
        self.transform = transforms.Compose([
            transforms.ToTensor(),
            transforms.Normalize(mean=[0.485, 0.456, 0.406], std=[0.229, 0.224, 0.225])
        ])

        self.get_logger().info('Segmentation node initialized')

    def load_model(self, model_path):
        # Загрузка модели (пример для PyTorch, замените на вашу архитектуру)
        model = torch.load(model_path, map_location=self.device)
        return model

    def preprocess_image(self, cv_image):
        # Преобразование изображения для модели
        img = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
        img = cv2.resize(img, (512, 512))  # Укажите нужный размер
        img_tensor = self.transform(img).unsqueeze(0).to(self.device)
        return img_tensor

    def postprocess_mask(self, output):
        # Постобработка выхода модели (предполагается, что модель возвращает logits)
        output = torch.softmax(output, dim=1)
        mask = torch.argmax(output, dim=1).squeeze().cpu().numpy()
        mask = mask.astype(np.uint8) * 255  # Для визуализации
        mask = cv2.resize(mask, (640, 480))  # Вернуть к исходному размеру
        return mask

    def image_callback(self, msg):
        try:
            # Преобразование ROS Image в OpenCV
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # Предобработка изображения
            input_tensor = self.preprocess_image(cv_image)

            # Инференс модели
            with torch.no_grad():
                output = self.model(input_tensor)

            # Постобработка маски
            mask = self.postprocess_mask(output)

            # Преобразование маски в ROS Image
            mask_msg = self.bridge.cv2_to_imgmsg(mask, encoding='mono8')
            mask_msg.header = msg.header  # Сохраняем заголовок для синхронизации

            # Публикация результата
            self.publisher.publish(mask_msg)
            self.get_logger().info('Published segmentation mask')

        except Exception as e:
            self.get_logger().error(f'Error processing image: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    node = SegmentationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
