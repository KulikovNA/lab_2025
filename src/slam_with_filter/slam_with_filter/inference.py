import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import os
from ultralytics import YOLO
from ament_index_python.packages import get_package_share_directory

class SegmentationFilterNode(Node):
    def __init__(self):
        super().__init__('segmentation_filter')

        package_share_directory = get_package_share_directory('slam_with_filter')
        weights_path = os.path.join(package_share_directory, 'data', 'yolo11n-seg.pt')

        try:
            self.model = YOLO(weights_path)
            self.model.model.to('cpu')  # Используем CPU (можно изменить на GPU, если доступно)
            self.get_logger().info(f"Loaded YOLO11n-seg model from {weights_path}")
        except Exception as e:
            self.get_logger().error(f"Failed to load YOLO model: {str(e)}")
            return


        self.publisher_ = self.create_publisher(Image, 'filtered_image', 10)

        self.debug_publisher_ = self.create_publisher(Image, 'segmented_image', 10)

        self.subscription = self.create_subscription(
            Image, '/camera/image_raw', self.listener_callback, 10)


        self.timer = self.create_timer(0.1, self.timer_callback)


        self.br = CvBridge()

        self.cv_image = None

        self.moving_classes = [0, 2, 3, 5, 7]  # COCO классы (0: person, 2: car, 3: motorcycle, 5: bus, 7: truck)

    def listener_callback(self, msg: Image):
        try:

            self.cv_image = self.br.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().error(f'Failed to convert image message to OpenCV: {str(e)}')

    def filter_moving_objects(self, image):
        if image is None:
            return None, None

        current_image = image.copy()

        results = self.model.predict(
            current_image,
            conf=0.5,  # Порог уверенности
            classes=self.moving_classes,  # Фильтруем только движущиеся объекты
            save=False,
            verbose=False
        )

        # Создаем маску для исключения движущихся объектов
        mask = np.ones_like(current_image, dtype=np.uint8) * 255  # Белая маска (все пиксели включены)
        if results[0].masks is not None:
            for mask_data in results[0].masks.data:
                # Маска для одного объекта (бинарная, 0 или 1)
                obj_mask = mask_data.cpu().numpy().astype(np.uint8) * 255
                # Инвертируем маску: области объектов становятся черными (0)
                mask[obj_mask == 255] = [0, 0, 0]

        # Применяем маску к изображению: области движущихся объектов становятся черными
        filtered_image = cv2.bitwise_and(current_image, mask)
        debug_image = results[0].plot() if results else current_image.copy()

        return filtered_image, debug_image

    def timer_callback(self):
        if self.cv_image is None:
            return


        filtered_image, debug_image = self.filter_moving_objects(self.cv_image)

        if filtered_image is not None:
            try:

                filtered_msg = self.br.cv2_to_imgmsg(filtered_image, encoding='bgr8')
                self.publisher_.publish(filtered_msg)
                self.get_logger().info('Published filtered image')


                debug_msg = self.br.cv2_to_imgmsg(debug_image, encoding='bgr8')
                self.debug_publisher_.publish(debug_msg)
                self.get_logger().info('Published debug segmented image')
            except Exception as e:
                self.get_logger().error(f'Failed to publish images: {str(e)}')

def main(args=None):
    rclpy.init(args=args)
    node = SegmentationFilterNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
