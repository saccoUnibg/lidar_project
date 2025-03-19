import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import numpy as np
import struct

class PointCloudSubscriber(Node):
    def __init__(self):
        super().__init__('pointcloud_subscriber')
        self.subscription = self.create_subscription(
            PointCloud2,
            '/rslidar_points',  # Sostituisci con il nome corretto del topic
            self.pointcloud_callback,
            10)
        self.subscription  # Evita che venga garbage-collected

    def pointcloud_callback(self, msg):
        self.get_logger().info(f'Ricevuto PointCloud2 con {msg.width * msg.height} punti')
        points = self.convert_pointcloud2_to_array(msg)
        self.get_logger().info(f'Primo punto: {points[0]}' if len(points) > 0 else 'Nessun punto valido')

    def convert_pointcloud2_to_array(self, cloud_msg):
        """Converte PointCloud2 in un array di numpy"""
        points = []
        for i in range(0, len(cloud_msg.data), cloud_msg.point_step):
            x, y, z = struct.unpack_from('fff', cloud_msg.data, offset=i)
            points.append([x, y, z])
        return np.array(points)

def main(args=None):
    rclpy.init(args=args)
    node = PointCloudSubscriber()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
