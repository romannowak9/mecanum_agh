import datetime
import os
import time

import numpy as np
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
import cv2
from cv_bridge import CvBridge

LOGGING = True

class Logger(Node):
    def __init__(self, log_dir="ros2_ws/pos_logs"):
        super().__init__('Logger')
        self.get_logger().info('Logger Node started.')

        self.log_dir = os.path.expanduser(os.path.join('~', log_dir))
        os.makedirs(self.log_dir, exist_ok=True)
        now = datetime.datetime.now()
        timestamp_str = now.strftime("%Y%m%d_%H%M%S")
        self.filename = os.path.join(self.log_dir, f"pos_log_{timestamp_str}.txt")
        self.file_handle = None
        
        try:
            self.file_handle = open(self.filename, 'w')
            self.get_logger().info(f"--- Log file created: {self.filename} ---")
            self.file_handle.write("Timestamp(s), X, Y, Orientation_Z, Orientation_W, Linear_X\n")
            
        except IOError as e:
            self.get_logger().error(f"Error opening log file: {e}")
            self.file_handle = None
        
        self.subscription = self.create_subscription(
            Odometry,
            '/model/vehicle_blue/odometry',
            self.odometry_callback,
            10
        )
        self.get_logger().info(f"Subscribing to '/model/vehicle_blue/odometry'...")

        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.minimap_ = self.create_publisher(Image, "/minimap", qos_profile=qos_profile)
        self.bridge = CvBridge()

        try:
            waypoints = np.load('../../scripts/center_line.npy', allow_pickle=True)
            self.track = waypoints[:, 0:2] * 12
        except Exception as e:
            self.get_logger().warn(f"Could not load track file: {e}. Using empty track.")
            self.track = np.array([[0,0]])

        self.map_width = 500
        self.map_height = 500
        self.margin = 50

        min_x = np.min(self.track[:, 0] - 10)
        max_x = np.max(self.track[:, 0] + 10)
        min_y = np.min(self.track[:, 1] - 10)
        max_y = np.max(self.track[:, 1] + 10)

        self.scale_x = (self.map_width - 2 * self.margin) / (max_x - min_x) if max_x != min_x else 1.0
        self.scale_y = (self.map_height - 2 * self.margin) / (max_y - min_y) if max_y != min_y else 1.0
        self.scale = min(self.scale_x, self.scale_y)

        self.offset_x = -min_x * self.scale + (self.map_width - (max_x - min_x) * self.scale) / 2
        self.offset_y = -min_y * self.scale + (self.map_height - (max_y - min_y) * self.scale) / 2

        self.base_image = np.zeros((self.map_height, self.map_width, 3), dtype=np.uint8)
        self.base_image[:] = (255, 255, 255)

        if self.track.shape[0] > 1:
            pts = []
            for point in self.track:
                u = int(point[0] * self.scale + self.offset_x)
                v = int(self.map_height - (point[1] * self.scale + self.offset_y))
                pts.append([u, v])
            
            pts = np.array(pts, np.int32)
            pts = pts.reshape((-1, 1, 2))
            cv2.polylines(self.base_image, [pts], isClosed=False, color=(0, 0, 255), thickness=2)

    def odometry_callback(self, msg: Odometry):
        if self.file_handle:
            try:
                sec = msg.header.stamp.sec
                nsec = msg.header.stamp.nanosec
                log_time = float(sec) + float(nsec) / 1e9 

                x = msg.pose.pose.position.x
                y = msg.pose.pose.position.y
                ori_z = msg.pose.pose.orientation.z
                ori_w = msg.pose.pose.orientation.w
                lin_x = msg.twist.twist.linear.x

                log_line = f"{log_time:.6f},{x},{y},{ori_z},{ori_w},{lin_x}\n"
                self.file_handle.write(log_line)
                self.file_handle.flush()
                
            except Exception as e:
                self.get_logger().error(f"Error writing data to log file: {e}")

        try:
            current_x = msg.pose.pose.position.x
            current_y = msg.pose.pose.position.y

            u = int(current_x * self.scale + self.offset_x)
            v = int(self.map_height - (current_y * self.scale + self.offset_y))

            map_img = self.base_image.copy()
            
            cv2.circle(map_img, (u, v), 8, (0, 0, 255), -1) 

            ros_image = self.bridge.cv2_to_imgmsg(map_img, encoding="bgr8")
            self.minimap_.publish(ros_image)

        except Exception as e:
             self.get_logger().error(f"Error publishing minimap: {e}")

    def close_file(self):
        if self.file_handle:
            self.file_handle.close()
            self.get_logger().info(f"--- Log file safely closed: {self.filename} ---")
            self.file_handle = None

def main(args=None):
    if LOGGING:
        rclpy.init(args=args)
        logger_node = Logger()
        
        try:
            rclpy.spin(logger_node)
        except KeyboardInterrupt:
            logger_node.get_logger().info('Keyboard Interrupt detected. Shutting down.')
        finally:
            logger_node.close_file()
            logger_node.destroy_node()

if __name__ == '__main__':
    main()