import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from cv_bridge import CvBridge
import cv2
import os
import numpy as np
from ultralytics import YOLO
from ament_index_python.packages import get_package_share_directory
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
import onnxruntime

class YOLOSegmentationNode(Node):
    def __init__(self):
        super().__init__('yolo_segmentation_node')
        
        # Инициализация CV Bridge
        self.br = CvBridge()
        self.current_image = None

        # Получение путей к файлам модели
        package_share_directory = get_package_share_directory('cv_basics')
        weights_path = os.path.join(package_share_directory, 'data', 'yolo11n-seg.pt')
        onnx_path = os.path.join(package_share_directory, 'data', 'yolo11n-seg.onnx')
        
        # Конвертация в ONNX при первом запуске
        if not os.path.exists(onnx_path):
            self.get_logger().info("Converting model to ONNX...")
            model = YOLO(weights_path)
            model.export(format="onnx", dynamic=True, simplify=True)
            os.rename(weights_path.replace('.pt', '.onnx'), onnx_path)
        
        # Инициализация ONNX Runtime
        self.ort_session = onnxruntime.InferenceSession(onnx_path)
        self.input_name = self.ort_session.get_inputs()[0].name
        
        # Сохранение оригинальной YOLO модели для визуализации
        self.model = YOLO(weights_path)
        self.model.model.to('cpu')
        
        # Публикаторы
        self.publisher_image = self.create_publisher(Image, 'video_with_predict', 10)
        self.publisher_markers = self.create_publisher(MarkerArray, 'segmentation_masks', 10)
        self.publisher_mask = self.create_publisher(Image, 'segmentation_mask', 10)
        
        # Подписка на изображения с камеры
        self.subscription = self.create_subscription(
            Image, '/camera/image_raw', self.image_callback, 10)
        
        # TF Broadcaster для трансформаций камеры
        self.tf_broadcaster = TransformBroadcaster(self)
        self.tf_frame = "camera_optical_frame"
        self.timer = self.create_timer(0.1, self.timer_callback)

    def image_callback(self, msg):
        try:
            self.current_image = self.br.imgmsg_to_cv2(msg, "bgr8")
            self.tf_frame = msg.header.frame_id
            self.publish_camera_tf()
        except Exception as e:
            self.get_logger().error(f'Ошибка обработки изображения: {str(e)}')

    def publish_camera_tf(self):
        """Публикация статической трансформации для камеры"""
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = "base_link"
        transform.child_frame_id = self.tf_frame
        transform.transform.translation.x = 0.0
        transform.transform.translation.y = 0.0
        transform.transform.translation.z = 0.0
        transform.transform.rotation.w = 1.0
        self.tf_broadcaster.sendTransform(transform)

    def create_mask_markers(self, masks):
        """Создание маркеров для визуализации масок в RViz"""
        marker_array = MarkerArray()
        
        if masks is None:
            return marker_array
            
        for i, mask in enumerate(masks):
            mask = (mask * 255).astype(np.uint8)
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            
            for j, contour in enumerate(contours):
                marker = Marker()
                marker.header.frame_id = self.tf_frame
                marker.header.stamp = self.get_clock().now().to_msg()
                marker.id = i * 100 + j
                marker.type = Marker.LINE_STRIP
                marker.action = Marker.ADD
                marker.scale.x = 0.02
                marker.color.r = float((i + 1) % 3 == 0)
                marker.color.g = float((i + 1) % 3 == 1)
                marker.color.b = float((i + 1) % 3 == 2)
                marker.color.a = 0.8
                
                for point in contour.squeeze():
                    p = Point()
                    p.x = float(point[0]) / mask.shape[1]
                    p.y = 1.0 - float(point[1]) / mask.shape[0]
                    p.z = 0.0
                    marker.points.append(p)
                
                if len(marker.points) > 0:
                    marker.points.append(marker.points[0])
                
                marker_array.markers.append(marker)
        
        return marker_array

    def create_mask_image(self, masks):
        """Создание бинарного изображения с масками сегментации"""
        if masks is None or len(masks) == 0:
            return None
            
        mask_img = np.zeros(self.current_image.shape[:2], dtype=np.uint8)
        
        for mask in masks:
            mask = (mask * 255).astype(np.uint8)
            mask_img = cv2.bitwise_or(mask_img, mask)
            
        return mask_img

    def process_frame_with_onnx(self, img):
        """Обработка кадра с использованием ONNX Runtime"""
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        img = cv2.resize(img, (640, 640))
        img = img.transpose(2, 0, 1)[None]
        img = np.ascontiguousarray(img) / 255.0
        img = img.astype(np.float32)

        outputs = self.ort_session.run(None, {self.input_name: img})
        
        return None, None

    def process_frame(self):
        if self.current_image is None:
            return None, None, None
        
        masks, mask_image = self.process_frame_with_onnx(self.current_image)
        
        results = self.model.predict(
            self.current_image,
            conf=0.5,
            verbose=False
        )
        
        plotted_image = results[0].plot()
        
        if masks is None and results[0].masks is not None:
            masks = results[0].masks.data.cpu().numpy()
            mask_image = self.create_mask_image(masks)
        
        return plotted_image, masks, mask_image

    def timer_callback(self):
        plotted_image, masks, mask_image = self.process_frame()
        
        if plotted_image is not None:
            self.publisher_image.publish(self.br.cv2_to_imgmsg(plotted_image, "bgr8"))
            
            if masks is not None:
                marker_array = self.create_mask_markers(masks)
                self.publisher_markers.publish(marker_array)
            
            if mask_image is not None:
                mask_msg = self.br.cv2_to_imgmsg(mask_image, "mono8")
                mask_msg.header.frame_id = self.tf_frame
                self.publisher_mask.publish(mask_msg)

def main(args=None):
    rclpy.init(args=args)
    node = YOLOSegmentationNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
