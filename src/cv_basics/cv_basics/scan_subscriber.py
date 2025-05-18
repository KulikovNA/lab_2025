import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32MultiArray
from visualization_msgs.msg import Marker, MarkerArray
import math

# Словарт для сопоставления индексов классов COCO с их названиями
COCO_NAMES = {
    0: 'person', 1: 'bicycle', 2: 'car', 3: 'motorcycle', 4: 'airplane', 5: 'bus', 6: 'train', 7: 'truck', 8: 'boat', 9: 'traffic light',
    10: 'fire hydrant', 11: 'stop sign', 12: 'parking meter', 13: 'bench', 14: 'bird', 15: 'cat', 16: 'dog', 17: 'horse', 18: 'sheep', 19: 'cow',
    20: 'elephant', 21: 'bear', 22: 'zebra', 23: 'giraffe', 24: 'backpack', 25: 'umbrella', 26: 'handbag', 27: 'tie', 28: 'suitcase', 29: 'frisbee',
    30: 'skis', 31: 'snowboard', 32: 'sports ball', 33: 'kite', 34: 'baseball bat', 35: 'baseball glove', 36: 'skateboard', 37: 'surfboard', 38: 'tennis racket', 39: 'bottle',
    40: 'wine glass', 41: 'cup', 42: 'fork', 43: 'knife', 44: 'spoon', 45: 'bowl', 46: 'banana', 47: 'apple', 48: 'sandwich', 49: 'orange',
    50: 'broccoli', 51: 'carrot', 52: 'hot dog', 53: 'pizza', 54: 'donut', 55: 'cake', 56: 'chair', 57: 'couch', 58: 'potted plant', 59: 'bed',
    60: 'dining table', 61: 'toilet', 62: 'tv', 63: 'laptop', 64: 'mouse', 65: 'remote', 66: 'keyboard', 67: 'cell phone', 68: 'microwave', 69: 'oven',
    70: 'toaster', 71: 'sink', 72: 'refrigerator', 73: 'book', 74: 'clock', 75: 'vase', 76: 'scissors', 77: 'teddy bear', 78: 'hair drier', 79: 'toothbrush'
}

class ScanSubscriber(Node):
    def __init__(self):
        super().__init__('scan_subscriber')
        # Подписка на топик с данными LiDAR
        self.create_subscription(LaserScan, 'scan', self.listener_callback, 10)
        # Подписка на топик с данными YOLO (центры боксов, класс, confidence)
        self.create_subscription(Float32MultiArray, 'yolo_boxes', self.yolo_callback, 10)
        # Паблишер для маркеров Rviz
        self.marker_publisher = self.create_publisher(MarkerArray, 'visualization_marker_array', 10)
        # Хранение центров боксов YOLO в списке (x, y, class_id, confidence)
        self.yolo_centers = []

    def yolo_callback(self, msg):
        # Парсим входящее сообщение: [x1, y1, class_id1, conf1, x2, y2, class_id2, conf2, ...]
        self.yolo_centers.clear()
        for i in range(0, len(msg.data), 4):
            x = msg.data[i]
            y = msg.data[i+1]
            class_id = int(msg.data[i+2])
            confidence = msg.data[i+3]
            self.yolo_centers.append((x, y, class_id, confidence))

    def listener_callback(self, msg):
        # --- Нстройки камеры ---
        camera_fov = 1.3962634  # ~80 градусов
        image_width = 640       # Ширина изображения, px
        # --- Фильрация lidar-лучей по сектору камеры и удаление nan/inf ---
        filtered_ranges = []
        filtered_angles = []
        for i, range_value in enumerate(msg.ranges):
            angle = msg.angle_min + i * msg.angle_increment
            if -camera_fov/2 <= angle <= camera_fov/2:
                if math.isinf(range_value) or math.isnan(range_value):
                    continue
                filtered_ranges.append(range_value)
                filtered_angles.append(angle)
        # --- Для каждого центра бокса YOLO находим ближайший лидар-луч ---
        yolo_to_lidar = []
        if self.yolo_centers and filtered_angles:
            for (x, y, class_id, confidence) in self.yolo_centers:
                # Переводим x-координату в угол LiDAR
                lidar_angle = - (x / image_width - 0.5) * camera_fov
                # Находим ближайший лидар-луч к углу LiDAR
                min_diff = float('inf')
                min_idx = 0
                for idx, a in enumerate(filtered_angles):
                    diff = abs(a - lidar_angle)
                    if diff < min_diff:
                        min_diff = diff
                        min_idx = idx
                if min_diff < 0.05:  # Если разница меньше 0.02 радиан, то добавляем точку
                    closest_angle = filtered_angles[min_idx]
                    closest_range = filtered_ranges[min_idx]
                    yolo_to_lidar.append((closest_angle, closest_range, class_id, confidence))
        # Удаляем старые маркеры
        if yolo_to_lidar:
            max_marker_count = 360
            delete_array = MarkerArray()
            for i in range(max_marker_count):
                marker = Marker()
                marker.header.frame_id = msg.header.frame_id
                marker.header.stamp = self.get_clock().now().to_msg()
                marker.ns = "processed_points"
                marker.id = i
                marker.action = Marker.DELETE
                delete_array.markers.append(marker)
                text_marker = Marker()
                text_marker.header.frame_id = msg.header.frame_id
                text_marker.header.stamp = self.get_clock().now().to_msg()
                text_marker.ns = "scan_markers"
                text_marker.id = i + 100
                text_marker.action = Marker.DELETE
                delete_array.markers.append(text_marker)
            self.marker_publisher.publish(delete_array)
        # Визуализация маркеров: сфера и текст
        marker_array = MarkerArray()
        if yolo_to_lidar:
            for idx, (lidar_angle, interp_range, class_id, confidence) in enumerate(yolo_to_lidar):
                # Сфера в точке lidаr-луча
                class_name = COCO_NAMES.get(class_id, str(class_id))
                sphere_marker = Marker()
                sphere_marker.header.frame_id = msg.header.frame_id
                sphere_marker.header.stamp = self.get_clock().now().to_msg()
                sphere_marker.ns = "processed_points"
                sphere_marker.id = idx
                sphere_marker.type = Marker.SPHERE
                sphere_marker.action = Marker.ADD
                sphere_marker.pose.position.x = interp_range * math.cos(lidar_angle)
                sphere_marker.pose.position.y = interp_range * math.sin(lidar_angle)
                sphere_marker.pose.position.z = 0.0
                sphere_marker.pose.orientation.w = 1.0
                sphere_marker.scale.x = 0.1
                sphere_marker.scale.y = 0.1
                sphere_marker.scale.z = 0.1
                sphere_marker.color.a = 1.0
                sphere_marker.color.r = 0.0
                sphere_marker.color.g = 1.0
                sphere_marker.color.b = 0.0
                marker_array.markers.append(sphere_marker)
                # Текст с расстоянием и классом
                text_marker = Marker()
                text_marker.header.frame_id = msg.header.frame_id
                text_marker.header.stamp = self.get_clock().now().to_msg()
                text_marker.ns = "scan_markers"
                text_marker.id = idx + 100
                text_marker.type = Marker.TEXT_VIEW_FACING
                text_marker.action = Marker.ADD
                text_marker.pose.position.x = interp_range * math.cos(lidar_angle)
                text_marker.pose.position.y = interp_range * math.sin(lidar_angle)
                text_marker.pose.position.z = 0.2
                text_marker.pose.orientation.w = 1.0
                text_marker.scale.z = 0.2
                text_marker.color.a = 1.0
                text_marker.color.r = 1.0
                text_marker.color.g = 1.0
                text_marker.color.b = 1.0
                text_marker.text = f"{interp_range:.2f}m class: {class_name}"
                marker_array.markers.append(text_marker)
        self.marker_publisher.publish(marker_array)

def main(args=None):
    rclpy.init(args=args)
    node = ScanSubscriber()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()