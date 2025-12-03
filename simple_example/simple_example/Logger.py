import datetime
import os
import time

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry

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
        
        # 3. Create ROS Subscriber
        self.subscription = self.create_subscription(
            Odometry,
            '/model/vehicle_blue/odometry',
            self.odometry_callback,
            10
        )
        self.get_logger().info(f"Subscribing to '/model/vehicle_blue/odometry'...")


    def odometry_callback(self, msg: Odometry):
        if not self.file_handle:
            return

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