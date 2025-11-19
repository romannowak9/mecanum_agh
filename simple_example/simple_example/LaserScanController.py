import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2

class LaserScanController(Node):
    def __init__(self):
        super().__init__('LaserScanController')

        self.subscription = self.create_subscription(
            PointCloud2,
            '/lidar/points',  
            self.laser_scan_callback,
            10
        )

    def laser_scan_callback(self, msg):
        pc_data = pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
        
        for point in pc_data:
            x, y, z = point
            self.get_logger().info(f"Point: x={x}, y={y}, z={z}")

def main(args=None):
    rclpy.init(args=args)

    node = LaserScanController()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
