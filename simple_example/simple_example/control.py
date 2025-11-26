import rclpy
from rclpy.node import Node
import math

from geometry_msgs.msg import Point
from std_msgs.msg import Float32
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu

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
        self.imu_sub = self.create_subscription(Imu, "/imu", self.imu_callback, qos_profile=point_qos_profile)

        # timer_period = 0.5  # seconds
        # self.timer = self.create_timer(timer_period, self.timer_callback)
        self.const_velocity = (2.0, 0.0, 0.0)
        self.vehicle_length = 1.0

        self.curr_vel = 0.0

        self.last_imu_time = None
        self.curr_vel = 0.0
        self.alpha = 0.5      # filtr dolnoprzepustowy
        self.filtered_acc = 0.0

    def imu_callback(self, imu_msg: Imu):
        current_time = imu_msg.header.stamp.sec + imu_msg.header.stamp.nanosec * 1e-9  # s

        if self.last_imu_time is None:
            self.last_imu_time = current_time
            return

        dt = current_time - self.last_imu_time
        self.last_imu_time = current_time

        # if dt <= 0.0 or dt > 1.0:   # zabezpieczenie przed błędnym dt
        #     return

        acc_x = -imu_msg.linear_acceleration.x

        # Filtr dolnoprepustowy IIR
        self.filtered_acc = self.alpha * self.filtered_acc + (1 - self.alpha) * acc_x

        self.curr_vel += self.filtered_acc * dt

        # Ograniczenie prędkości
        # self.curr_vel = max(min(self.curr_vel, 10.0), -2.0)

        self.get_logger().info(f"IMU dt: {dt} s")
        self.get_logger().info(f"Vel from IMU: {self.curr_vel:.3f} m/s")
        self.get_logger().info(f"acceleration: {acc_x} m/s^2")
        

        

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
            forward_velocity=self.const_velocity[0]  # Żeby brało prędkość z IMU podmienić na self.curr_vel
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