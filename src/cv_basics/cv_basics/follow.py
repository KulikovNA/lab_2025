import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image, LaserScan
from std_msgs.msg import Int32MultiArray
import time
from sensor_msgs.msg import LaserScan

class MyNodes(Node):
    def __init__(self):
        super().__init__('follow_node')
        self.subscription = self.create_subscription(Image, '/camera/image_raw', self.image_callback, 10)
        self.subscription_center = self.create_subscription(Int32MultiArray, '/object_center', self.center_callback, 10)
        self.subscription_scan = self.create_subscription(LaserScan, '/scan', self.lidar_callback, 10)
        self.cmd_pub = self.create_publisher(Twist, 'cmd_vel', 10)

        self.image_width = None
        self.object_center_x = None
        self.last_object_time = None
        self.last_known_offset = 0
        self.latest_scan = None

        self.max_turn_speed = 0.4
        self.forward_speed = 0.2
        self.center_tolerance = 30
        self.timeout_sec = 1.0
        self.stop_distance = 0.6

        self.timer = self.create_timer(0.1, self.control_loop)

    def image_callback(self, msg):
        self.image_width = msg.width

    def center_callback(self, msg):
        if len(msg.data) >= 2:
            self.object_center_x = msg.data[0]
            self.last_object_time = time.time()
            if self.image_width:
                self.last_known_offset = self.object_center_x - self.image_width / 2
        else:
            self.object_center_x = None

    def lidar_callback(self, msg: LaserScan):
        self.latest_scan = msg

    def get_distance_ahead(self):
        """Возвращает расстояние по направлению камеры"""
        if not self.latest_scan:
            return float('inf')

        ranges = self.latest_scan.ranges
        mid_index = len(ranges) // 2 
        return ranges[mid_index] if not (ranges[mid_index] == float('inf') or ranges[mid_index] != ranges[mid_index]) else float('inf')

    def control_loop(self):
        cmd = Twist()
        current_time = time.time()

        object_visible = (
            self.object_center_x is not None and
            self.last_object_time is not None and
            (current_time - self.last_object_time) < self.timeout_sec
        )

        distance = self.get_distance_ahead() - 3
        self.get_logger().info(f"Расстояние до объекта: {distance:.2f} м")

        if not object_visible or self.image_width is None:
            if abs(self.last_known_offset) > self.center_tolerance:
                norm_offset = self.last_known_offset / (self.image_width / 2)
                cmd.angular.z = -self.max_turn_speed * norm_offset
                self.get_logger().info("Объект потерян — вращаюсь в сторону последней позиции")
            else:
                cmd.angular.z = 0.1
                self.get_logger().info("Объект потерян — вращаюсь для поиска")
        else:
            offset = self.object_center_x - self.image_width / 2
            norm_offset = offset / (self.image_width / 2)
            cmd.angular.z = -self.max_turn_speed * norm_offset

            if distance > self.stop_distance:
                if abs(offset) < self.center_tolerance:
                    cmd.linear.x = self.forward_speed
                    self.get_logger().info("Двигаюсь вперёд к объекту")
                else:
                    cmd.linear.x = max(0.05, self.forward_speed * (1.0 - abs(norm_offset)))
                    self.get_logger().info(f"Корректирую курс: offset = {offset:.1f}px")
            else:
                cmd.linear.x = 0.0
                self.get_logger().info("Остановился — объект близко")

        self.cmd_pub.publish(cmd)



def main(args=None):
    rclpy.init(args=args)
    node = MyNodes()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
