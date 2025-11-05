import rclpy
from rclpy.node import Node
import math

from geometry_msgs.msg import Point
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist

from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy


class VelocityPublisher(Node):

    def __init__(self):
        super().__init__('velocity_publisher')

        point_qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )

        cmd_qos_profile = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.publisher_ = self.create_publisher(Twist, '/model/vehicle_blue/cmd_vel', qos_profile=cmd_qos_profile)
        self.point_sub = self.create_subscription(Point, "/img_set_point", self.point_callback, qos_profile=point_qos_profile)

        # timer_period = 0.5  # seconds
        # self.timer = self.create_timer(timer_period, self.timer_callback)
        self.const_velocity = (2.0, 0.0, 0.0)
        self.vehicle_length = 1.0

    def point_callback(self, point: Point):
        msg = Twist()

        msg.linear.x = self.const_velocity[0]
        msg.linear.y = self.const_velocity[1]
        msg.linear.z = self.const_velocity[2]

        # Calculate error angle from input point
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

        self.publisher_.publish(msg)

# control_angle = psi(t)
# steer_angle = sigma(t)
# track_error = e
# vehicle_velocity = vf
# vehicle_gain = k
# margin = ks
def stanley_control(control_angle, track_error, vehicle_velocity, vehicle_gain, margin=0.05):
    crosstrack = math.atan2(vehicle_gain * track_error) / (vehicle_velocity + margin)
    steer_angle = control_angle + crosstrack
    # TODO: add limist [min_steer_angle, max_steer_angle]
    return steer_angle 

# vehcile_length = L
# error_angle = alpha
# distance_from_point = ld
# forward_velcity = vf
def pure_pursuit(vehicle_length, error_angle, forward_velocity, gain=1):
    heading_angle = math.atan((2.0 * vehicle_length * math.sin(error_angle)) / (gain * forward_velocity))
    return heading_angle


def main(args=None):
    rclpy.init(args=args)

    vel_publisher = VelocityPublisher()

    rclpy.spin(vel_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    vel_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()