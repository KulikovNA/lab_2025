
import rclpy # Python Client Library for ROS 2
from rclpy.node import Node # Handles the creation of nodes
from sensor_msgs.msg import Image # Image is the message type
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images

import sys 
import cv2
import threading
import numpy as np
import time

from .subscrib import Service # local module


class MyNodes(Node):
    # Конструктор класса, инициализирующий узел ROS
    def __init__(self, obj_service: Service):
        """
        Инициализирует узел ROS для подписки и публикации изображений, используя сервис обработки.

        Args:
            obj_service (Service): Сервис для обработки и анализа изображений.
        """
        # Вызов конструктора базового класса для инициализации узла с заданным именем
        super().__init__('my_publisher_subscriber_node')
        # Создание публикатора для отправки обработанных изображений
        self.publisher_ = self.create_publisher(Image, 'video_with_predict', 30)
        # Создание подписки на входящие изображения с камеры
        self.subscription = self.create_subscription(Image, '/camera/image_raw', self.listener_callback, 30)
        # Установка таймера для периодического вызова функции обратного вызова
        self.timer = self.create_timer(0.5, self.timer_callback)
        # Сохранение ссылки на переданный сервис обработки изображений
        self.obj_service = obj_service
        # Инициализация объекта для конвертации изображений между ROS и OpenCV
        self.br = CvBridge()
    
    def listener_callback(self, msg: Image):
        """
        Callback функция для обработки входящих сообщений изображений. Преобразует изображения из формата ROS в OpenCV и отправляет их на дальнейшую обработку.

        Args:
            msg (Image): Сообщение с изображением из ROS.
        
        Raises:
            Exception: В случае ошибок преобразования изображения.
        """
        try:
            # Конвертация изображения из формата ROS в формат OpenCV
            self.cv_image = self.br.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            # Передача изображения в сервис для дополнительной обработки
            self.obj_service.dump_out_image(self.cv_image)
        except Exception as e:
            # Логирование ошибки, если произошла ошибка конвертации
            self.get_logger().error('Failed to convert image message to OpenCV: {}'.format(str(e)))

    def get_img(self) -> np.ndarray:
        """
        Извлекает изображение с предиктами из сервиса.

        Returns:
            np.ndarray: Обработанное изображение в формате OpenCV.
        """
        return self.obj_service.get_image_whith_predict()

    def timer_callback(self):
        """
        Callback функция, вызываемая таймером. Публикует изображения с предиктами.

        """
        # Получение изображения с предиктами
        frame = self.get_img()
        if frame is not None:
            # Publish the image.
            # The 'cv2_to_imgmsg' method converts an OpenCV
            # image to a ROS 2 image message
            self.publisher_.publish(self.br.cv2_to_imgmsg(frame))
        

def service_thread(obj_service: Service):
    """
    Запускает сервис обработки в отдельном потоке.

    Args:
        obj_service (Service): Сервис для запуска.
    """
    obj_service.forward()


def main(args=None):
    """
    Основная функция для инициализации и запуска ROS-узла.

    Args:
        args: Аргументы командной строки (необязательно).
        Можно использовать argparse для получения аргументов из командной строки.
    """
    # Инициализация библиотеки rclpy с возможными аргументами командной строки
    rclpy.init(args=args)
    # Создание объекта сервиса для обработки изображений
    obj_service = Service()
    # Создание и запуск потока для выполнения сервисной логики
    thread = threading.Thread(target=service_thread, args=(obj_service,))
    # Запуск потока
    thread.start()
    #time.sleep(1)
    # Создание объекта узла для подписки и публикации изображений
    node = MyNodes(obj_service)
    # Вход в бесконечный цикл обработки сообщений ROS
    rclpy.spin(node)
    # Явное уничтожение узла после выхода из цикла
    node.destroy_node()
    # Завершение работы с библиотекой rclpy
    rclpy.shutdown()
  
if __name__ == '__main__':
  main()
