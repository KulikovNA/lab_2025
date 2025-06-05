#!/usr/bin/env python3
import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

class CameraController(Node):
    def __init__(self):
        super().__init__('camera_rotator')
        # Контроллер ожидает команды для группы суставов
        topic = '/camera_position_controller/commands'
        self.publisher = self.create_publisher(Float64MultiArray, topic, 10)
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.angle = 0.0  # текущий угол вращения

    def timer_callback(self):
        msg = Float64MultiArray()
        msg.data = [self.angle]  # Одно значение для camera_joint

        self.publisher.publish(msg)
        self.get_logger().info(f"Publishing camera position: {self.angle:.2f} rad")

        # Увеличиваем угол для следующей итерации
        self.angle += 0.05  # Уменьшил шаг для более плавного вращения

        # Ограничиваем угол, чтобы он не рос бесконечно
        if self.angle > 2 * math.pi:
            self.angle -= 2 * math.pi

def main(args=None):
    rclpy.init(args=args)
    node = CameraController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Camera rotator stopped by user")
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
