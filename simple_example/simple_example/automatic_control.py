import json
import rclpy
from rclpy.node import Node
import math
from geometry_msgs.msg import Point, Twist
from sensor_msgs.msg import Imu
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from std_msgs.msg import String

class PID:
    def __init__(self, kp, ki, kd):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.prev_error = 0
        self.prev_prev_error = 0

    def __call__(self, err):
        p = self.kp * (err - self.prev_error)
        i = self.ki * (err)
        d = self.kd * (err - 2 * self.prev_error + self.prev_prev_error)

        self.prev_prev_error = self.prev_error
        self.prev_error = err

        return p + i + d

class AutomaticController(Node):
    def __init__(self):
        super().__init__('automatic_controller')

        point_qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.point_sub = self.create_subscription(
            Point, 
            "/img_set_point", 
            self.point_callback, 
            qos_profile=point_qos_profile
        )

        self.lidar_sub = self.create_subscription(
            String, 
            "/model/vehicle_blue/surround_area", 
            self.lidar_callback, 
            qos_profile=point_qos_profile
        )

        self.imu_sub = self.create_subscription(
            Imu,
            "/imu",
            self.imu_callback,
            10
        )

        self.vehicle_length = 1.0
        self.surround_area = {'front': False, 'back': False, 'left': False, 'right': False}
        self.latest_auto_twist = Twist()

        self.pid = PID(kp=3, ki=1.0, kd=0.05)

        self.velocity = {'x': 0.0, 'y': 0.0, 'z': 0.0}
        self.prev_time = self.get_clock().now().seconds_nanoseconds()[0]
        self.velocity_destined = 6

    def point_callback(self, point: Point):
        msg = Twist()
        vel_delta = self.velocity['x'] - self.velocity_destined
        msg.linear.x = 0.0 + self.velocity_destined
        msg.linear.y = 0.0
        msg.linear.z = 0.0
        dist = point.y
        e = -point.x

        calc_angle = self.pid(e)

        # calc_angle = pure_pursuit(self.vehicle_length, e, msg.linear.x, gain=0.1)

        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = float(calc_angle)

        self.latest_auto_twist = msg

    def lidar_callback(self, msg: String):
        surround_area = json.loads(msg.data)
        self.surround_area = surround_area
    
    def imu_callback(self, msg: Imu):
        current_time = self.get_clock().now().seconds_nanoseconds()[0] + self.get_clock().now().seconds_nanoseconds()[1] / 1e9
        dt = (current_time - self.prev_time)
        acc_x = msg.linear_acceleration.x
        acc_y = msg.linear_acceleration.y
        acc_z = msg.linear_acceleration.z

        # self.velocity['x'] += acc_x * dt
        # self.velocity['y'] += acc_y * dt
        # self.velocity['z'] += acc_z * dt

        self.prev_time = current_time

        # self.get_logger().info('x ' + str(self.velocity['x']))
        # self.get_logger().info('y ' + str(self.velocity['y']))
        # self.get_logger().info('z ' + str(self.velocity['z']))

# vehcile_length = L
# error_angle = alpha
# distance_from_point = ld
# forward_velcity = vf
def pure_pursuit(vehicle_length, error_angle, forward_velocity, gain=1):
    heading_angle = math.atan((2.0 * vehicle_length * math.sin(error_angle)) / (gain * forward_velocity))
    return heading_angle