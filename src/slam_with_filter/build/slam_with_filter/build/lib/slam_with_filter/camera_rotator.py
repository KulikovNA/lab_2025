#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration

class CameraRotator(Node):
    def __init__(self):
        super().__init__('camera_rotator')
        self.publisher_ = self.create_publisher(JointTrajectory, '/camera_joint_trajectory', 10)
        self.timer = self.create_timer(0.1, self.timer_callback)  # Публикация каждые 0.1 с
        self.joint_name = 'camera_joint'
        self.angular_velocity = 1.0  # Угловая скорость, рад/с
        self.current_position = 0.0  # Текущий угол, начинается с 0

    def timer_callback(self):
        # Обновляем позицию на основе скорости и времени
        dt = 0.1  # Период таймера
        self.current_position += self.angular_velocity * dt

        # Создаем сообщение JointTrajectory
        msg = JointTrajectory()
        msg.joint_names = [self.joint_name]
        point = JointTrajectoryPoint()
        point.positions = [self.current_position]
        point.velocities = [self.angular_velocity]
        point.time_from_start = Duration(sec=0, nanosec=int(dt * 1e9))
        msg.points = [point]

        # Публикуем сообщение
        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing camera joint position: {self.current_position}')

def main(args=None):
    rclpy.init(args=args)
    node = CameraRotator()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
