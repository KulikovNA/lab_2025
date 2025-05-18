import numpy as np # library designed to work with multidimensional arrays
# base64 - This module provides functions for encoding binary data to printable ASCII 
# characters and decoding such encodings back to binary data.
import base64 
import zmq # An open-source universal messaging library.
from sensor_msgs.msg import Image # Image is the message type
from cv_bridge import CvBridge # Package to convert between ROS and OpenCV Images

import cv2 # OpenCV librar
import time # This module provides various time-related functions.
import math # This module provides access to the mathematical functions defined by the C standard

        
class Publisher:
    
    def __init__(self, ip, port):
        """
        Инициализация издателя.

        Args:
            ip (str): IP-адрес для подключения.
            port (int): Порт для подключения.
        """
        self.ip = ip
        self.port = port
        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.PUB)
        self.socket.bind(f"tcp://{self.ip}:{self.port}")

    def publish_multipart(self, info, *args):
        """
        Публикация многочастного сообщения.

        Args:
            info (str): Информационное сообщение.
            *args (Any): Дополнительные данные для отправки.

        Raises:
            Exception: В случае ошибок при отправке сообщения.
        """
        multipart_data = [info.encode('utf-8')] + list(args)
        self.socket.send_multipart(multipart_data)
        time.sleep(0.1)

    def publish_string(self, string):
        """
        Публикация строки.

        Args:
            string (str): Строка для публикации.

        Raises:
            Exception: В случае ошибок при отправке сообщения.
        """
        self.socket.send_string(string)

    def close(self):
        """
        Закрытие сокета и завершение контекста.

        Raises:
            Exception: В случае ошибок при закрытии сокета или завершении контекста.
        """
        self.socket.close()
        self.context.term()


class Subscriber:
    
    def __init__(self, ip, port):
        """
        Инициализация подписчика.

        Args:
            ip (str): IP-адрес для подключения.
            port (int): Порт для подключения.
        """
        self.ip = ip
        self.port = port
        self.context = zmq.Context()
        self.socket = self.context.socket(zmq.SUB)
        self.socket.connect(f"tcp://{self.ip}:{self.port}")
        self.socket.setsockopt_string(zmq.SUBSCRIBE, '')

    def subscribe_string(self):
        """
        Подписка на строковые сообщения.

        Returns:
            str: Полученное строковое сообщение.

        Raises:
            Exception: В случае ошибок при получении сообщения.
        """
        return self.socket.recv_string()

    def subscribe_multipart(self):
        """
        Подписка на многочастные сообщения.

        Returns:
            List[bytes]: Список полученных частей сообщения.

        Raises:
            Exception: В случае ошибок при получении сообщения.
        """
        return self.socket.recv_multipart()

    def close(self):
        """
        Закрытие сокета и завершение контекста.

        Raises:
            Exception: В случае ошибок при закрытии сокета или завершении контекста.
        """
        self.socket.close()
        self.context.term()


class VideoPublisher(Publisher):
    def publish_video_frame(self, frame):
        """
            Здесь нужно написать код, который отправляет кадр видео.
        """

class VideoSubscriber(Subscriber):
    def subscribe_video_frame(self):
        """
            Здесь нужно написать код, который получает кадр видео.
        """
    
class Service:
    def __init__(self):
        self.out_image = None
        self.image_with_predict = None 
    

    def forward(self):
        video_publisher = VideoPublisher('127.0.0.1', 5556) 
        video_subscriber = VideoSubscriber('127.0.0.1', 5557)
        
        while self.out_image is None:
            time.sleep(0.05)
        
        try:
            while True:
                frame = self.out_image  # Получение кадра для отправки
                video_publisher.publish_video_frame(frame)  # Отправка кадра
                time.sleep(0.01)  # Добавлено для замедления цикла
                
                frame_input = video_subscriber.subscribe_video_frame()
                self.image_with_predict = frame_input
                
        
        except KeyboardInterrupt:
            print("Прерывание с клавиатуры, выход из цикла...")
        finally:
            video_publisher.close()  # Закрытие публикатора и освобождение ресурсов

    def get_image_whith_predict(self):
        return self.image_with_predict

    def dump_out_image(self, image):
        self.out_image = image

