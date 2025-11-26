import json
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
from std_msgs.msg import String

class LaserScanController(Node):
    def __init__(self):
        super().__init__('LaserScanController')

        self.subscription = self.create_subscription(
            PointCloud2,
            '/lidar/points',  
            self.laser_scan_callback,
            10
        )

        self.surround_publisher = self.create_publisher(
            String,
            '/model/vehicle_blue/surround_area',
            10
        )

        self.SURROUNDING_DISTANCE_THRESHOLD = 1.0  # meters
        self.surround_area = {'front': False, 'back': False, 'left': False, 'right': False}
        self.Z_THRESHOLD = 0.5  # meters
        self.VEH_LENGTH = 1.5  # meters

    def laser_scan_callback(self, msg):
        try:
            pc_data = pc2.read_points(msg, field_names=("x", "y", "z"), skip_nans=True)
            temp_area = {'front': False, 'back': False, 'left': False, 'right': False}
            for point in pc_data:
                x, y, z = point
                if abs(z) < self.Z_THRESHOLD:
                    distance = (x**2 + y**2)**0.5
                    if distance < self.SURROUNDING_DISTANCE_THRESHOLD:
                        if x > 0 and abs(y) < abs(x):
                            temp_area['front'] = True
                        elif x < 0:
                            temp_area['back'] = True
                        elif y > 0 and abs(x) < abs(y):
                            temp_area['left'] = True
                        elif y < 0 and abs(x) < abs(y):
                            temp_area['right'] = True
                    elif distance < self.SURROUNDING_DISTANCE_THRESHOLD + self.VEH_LENGTH and x < 0 and y < 1.0 and y > -1.0:
                        temp_area['back'] = True

            self.surround_area = temp_area
            msg_out = String()
            msg_out.data = json.dumps(self.surround_area)
            self.surround_publisher.publish(msg_out)
        except Exception as e:
            self.get_logger().error(f"Error processing point cloud data: {e}")  

def main(args=None):
    rclpy.init(args=args)

    node = LaserScanController()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
