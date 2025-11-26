import rclpy
from rclpy.node import Node
import math
from geometry_msgs.msg import Point, Twist
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy

def stanley_control(control_angle, track_error, vehicle_velocity, vehicle_gain, margin=0.05):
    crosstrack = math.atan2(vehicle_gain * track_error) / (vehicle_velocity + margin)
    steer_angle = control_angle + crosstrack
    return steer_angle 

def pure_pursuit(vehicle_length, error_angle, forward_velocity, gain=1):
    heading_angle = math.atan((2.0 * vehicle_length * math.sin(error_angle)) / (gain * forward_velocity))
    return heading_angle


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

        self.const_velocity = (2.0, 0.0, 0.0)
        self.vehicle_length = 1.0
        
        self.latest_auto_twist = Twist()

        self.pid = PID(kp=2, ki=0.5, kd=0.02)

    def point_callback(self, point: Point):
        msg = Twist()
        msg.linear.x = self.const_velocity[0]
        msg.linear.y = self.const_velocity[1]
        msg.linear.z = self.const_velocity[2]

        dist = point.y
        e = -point.x

        ## PURE PURSUIT
        # err_angle = math.atan2(e, dist + self.vehicle_length)

        # calc_angle = pure_pursuit(
        #     vehicle_length=self.vehicle_length,
        #     error_angle=err_angle,
        #     forward_velocity=self.const_velocity[0]
        # )

        ## PID
        calc_angle  = self.pid(e)


        msg.angular.x = 0.0
        msg.angular.y = 0.0
        msg.angular.z = float(calc_angle)

        self.latest_auto_twist = msg
