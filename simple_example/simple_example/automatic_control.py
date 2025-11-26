import json
import rclpy
from rclpy.node import Node
import math
from geometry_msgs.msg import Point, Twist
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from std_msgs.msg import String

def stanley_control(control_angle, track_error, vehicle_velocity, vehicle_gain, margin=0.05):
    crosstrack = math.atan2(vehicle_gain * track_error) / (vehicle_velocity + margin)
    steer_angle = control_angle + crosstrack
    return steer_angle 

def pure_pursuit(vehicle_length, error_angle, forward_velocity, gain=1):
    heading_angle = math.atan((2.0 * vehicle_length * math.sin(error_angle)) / (gain * forward_velocity))
    return heading_angle

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

        self.const_velocity = (2.0, 0.0, 0.0)
        self.vehicle_length = 1.0
        self.surround_area = {'front': False, 'back': False, 'left': False, 'right': False}
        self.emergency = False
        self.latest_auto_twist = Twist()

    def point_callback(self, point: Point):
        msg = Twist()
        msg.linear.x = self.const_velocity[0]
        msg.linear.y = self.const_velocity[1]
        msg.linear.z = self.const_velocity[2]

        dist = point.y
        e = -point.x

        err_angle = math.atan2(e, dist + self.vehicle_length)

        calc_angle = pure_pursuit(
            vehicle_length=self.vehicle_length,
            error_angle=err_angle,
            forward_velocity=self.const_velocity[0]
        )
        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = float(calc_angle)

        if self.surround_area.get('front', False):
            msg.linear.x = 0.0
            if self.surround_area.get('left', False) and not self.surround_area.get('right', False):
                msg.angular.z = -abs(msg.angular.z) if msg.angular.z != 0.0 else -0.5
            elif self.surround_area.get('right', False) and not self.surround_area.get('left', False):
                msg.angular.z = abs(msg.angular.z) if msg.angular.z != 0.0 else 0.5
            else:
                msg.angular.z = msg.angular.z * 1.0
        else:
            if self.surround_area.get('left', False) and not self.surround_area.get('right', False):
                msg.angular.z = min(msg.angular.z, -0.2)
            elif self.surround_area.get('right', False) and not self.surround_area.get('left', False):
                msg.angular.z = max(msg.angular.z, 0.2)


        self.emergency = any(self.surround_area.values())
        self.latest_auto_twist = msg

    def lidar_callback(self, msg: String):
        surround_area = json.loads(msg.data)
        self.surround_area = surround_area