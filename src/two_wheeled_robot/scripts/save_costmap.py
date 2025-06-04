import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
import numpy as np
import cv2
import yaml
import os

class CostmapSaver(Node):
    def __init__(self):
        super().__init__('costmap_saver')
        self.subscription = self.create_subscription(
            OccupancyGrid, '/global_costmap/costmap', self.costmap_callback, 10)
        self.costmap_received = False
        self.output_dir = '/workspace/lab_2025/src/two_wheeled_robot/maps/'
        self.get_logger().info('Waiting for costmap...')

    def costmap_callback(self, msg):
        if self.costmap_received:
            return
        self.costmap_received = True

        # Extract costmap data
        width = msg.info.width
        height = msg.info.height
        resolution = msg.info.resolution
        origin_x = msg.info.origin.position.x
        origin_y = msg.info.origin.position.y
        data = np.array(msg.data, dtype=np.int8).reshape(height, width)

        # Convert to PGM format (0-255 scale)
        # Costmap: -1 (unknown) -> 205, 0 (free) -> 255, 100 (occupied) -> 0
        pgm_data = np.zeros_like(data, dtype=np.uint8)
        pgm_data[data == -1] = 205  # Unknown (gray)
        pgm_data[data == 0] = 255   # Free (white)
        pgm_data[data > 0] = (100 - data[data > 0]) * 255 // 100  # Occupied/inflated (black to gray)

        # Save PGM
        pgm_file = os.path.join(self.output_dir, 'costmap.pgm')
        cv2.imwrite(pgm_file, pgm_data)
        self.get_logger().info(f'Saved costmap PGM to {pgm_file}')

        # Save YAML
        yaml_data = {
            'image': 'costmap.pgm',
            'mode': 'trinary',
            'resolution': resolution,
            'origin': [origin_x, origin_y, 0.0],
            'negate': 0,
            'occupied_thresh': 0.65,
            'free_thresh': 0.25
        }
        yaml_file = os.path.join(self.output_dir, 'costmap.yaml')
        with open(yaml_file, 'w') as f:
            yaml.dump(yaml_data, f)
        self.get_logger().info(f'Saved costmap YAML to {yaml_file}')

        # Shutdown after saving
        rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = CostmapSaver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()