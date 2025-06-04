import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from nav2_msgs.action import NavigateToPose
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
import numpy as np
import random
import math

class RandomGoalGenerator(Node):
    def __init__(self):
        super().__init__('random_goal_generator')
        self._action_client = ActionClient(self, NavigateToPose, '/navigate_to_pose')
        self.map_sub = self.create_subscription(
            OccupancyGrid, '/global_costmap/costmap', self.map_callback, 10)
        self.map_data = None
        self.map_width = 0
        self.map_height = 0
        self.map_resolution = 0.0
        self.map_origin = None
        self.free_cells = []
        self.goal_active = False  # Отслеживание статуса достижения цели
        self.get_logger().info('Waiting for map data...')

    def map_callback(self, msg):
        if self.map_data is not None:
            return  # Обработка карты единожды
        self.map_data = np.array(msg.data, dtype=np.int8).reshape(msg.info.height, msg.info.width)
        self.map_width = msg.info.width
        self.map_height = msg.info.height
        self.map_resolution = msg.info.resolution
        self.map_origin = msg.info.origin
        # Определение свободных клеток
        self.free_cells = [(y, x) for y in range(self.map_height) for x in range(self.map_width) if self.map_data[y, x] == 0]
        self.get_logger().info(f'Found {len(self.free_cells)} free cells in map')
        # Дать новую цель при условии выполнения предыдущей
        if not self.goal_active:
            self.send_random_goal()

    def grid_to_world(self, grid_x, grid_y):
        # Конвертирование ячеек карты в координаты
        world_x = self.map_origin.position.x + (grid_x + 0.5) * self.map_resolution
        world_y = self.map_origin.position.y + (grid_y + 0.5) * self.map_resolution
        return world_x, world_y

    def send_random_goal(self):
        if not self.free_cells or not self._action_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().warn('No free cells or action server unavailable')
            return

        if self.goal_active:
            self.get_logger().info('Waiting for previous goal to complete')
            return

        # Выбор случайной цели
        grid_y, grid_x = random.choice(self.free_cells)
        world_x, world_y = self.grid_to_world(grid_x, grid_y)
        # Случайный угол поворота
        yaw = random.uniform(-math.pi, math.pi)

        goal_msg = NavigateToPose.Goal()
        goal_pose = PoseStamped()
        goal_pose.header.frame_id = 'map'
        goal_pose.header.stamp = self.get_clock().now().to_msg()
        goal_pose.pose.position.x = world_x
        goal_pose.pose.position.y = world_y
        goal_pose.pose.position.z = 0.0
        q = self.quaternion_from_euler(0, 0, yaw)
        goal_pose.pose.orientation.x = q[0]
        goal_pose.pose.orientation.y = q[1]
        goal_pose.pose.orientation.z = q[2]
        goal_pose.pose.orientation.w = q[3]
        goal_msg.pose = goal_pose

        self.get_logger().info(f'Sending goal: x={world_x:.2f}, y={world_y:.2f}, yaw={yaw:.2f}')
        self.goal_active = True  # Пометка активной цели
        self._send_goal_future = self._action_client.send_goal_async(goal_msg)
        self._send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected')
            self.goal_active = False
            self.send_random_goal()  # Новая цель
            return
        self.get_logger().info('Goal accepted')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result()
        status = result.status
        if status == 4:  # Успех
            self.get_logger().info('Goal succeeded')
        elif status == 6:  # Неудача
            self.get_logger().info('Goal aborted')
        elif status == 5:  # Отказ от выполнения
            self.get_logger().info('Goal canceled')
        else:
            self.get_logger().info(f'Goal finished with status {status}')
        self.goal_active = False  # Сброс статуса текущей задачи
        self.send_random_goal()  # Задание новой цели

    def quaternion_from_euler(self, roll, pitch, yaw):
        qx = math.sin(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) - math.cos(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        qy = math.cos(roll/2) * math.sin(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.cos(pitch/2) * math.sin(yaw/2)
        qz = math.cos(roll/2) * math.cos(pitch/2) * math.sin(yaw/2) - math.sin(roll/2) * math.sin(pitch/2) * math.cos(yaw/2)
        qw = math.cos(roll/2) * math.cos(pitch/2) * math.cos(yaw/2) + math.sin(roll/2) * math.sin(pitch/2) * math.sin(yaw/2)
        return [qx, qy, qz, qw]

def main(args=None):
    rclpy.init(args=args)
    node = RandomGoalGenerator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()