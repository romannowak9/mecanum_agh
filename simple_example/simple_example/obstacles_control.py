import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from rclpy.qos import QoSProfile, QoSHistoryPolicy
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
import math
from enum import Enum


OBSTACLE_AVOIDING_LINEAR_VEL = 0.9
OBSTACLE_AVOIDING_ANGULAR_VEL = 0.6
MAX_ANGULAR = 1.5
SAFE_DISTANCE = 2.2
TARGET_WALL_DIST = 1.4
WALL_SEEN_DIST = 2.6                  # odległość, przy której uznajemy, że "widzimy" ścianę po boku
KP_WALL_FOLLOW = 1.5                  # współczynnik P dla utrzymania odległości

# angles
FRONT_MIN = -math.radians(35)
FRONT_MAX =  math.radians(35)

LEFT_MIN  =  math.radians(35)
LEFT_MAX  =  math.radians(110)

RIGHT_MIN = -math.radians(110)
RIGHT_MAX = -math.radians(35)


class ControlState(Enum):
    DRIVE = 1
    OBSTACLE_DETECTED = 2
    TURNING = 3
    WALL_FOLLOWING = 4
    OBSTACLE_END = 5


class LaserScanController(Node):
    def __init__(self):
        super().__init__('LaserScanController')

        self.laser_sub_ = self.create_subscription(
            LaserScan,
            '/lidar',  
            self.lidar_cb,
            10
        )

        self.avoid_flag_publisher_ = self.create_publisher(
            Bool,
            '/avoiding_obstacle',
            10
        )

        cmd_qos_profile = QoSProfile(history=QoSHistoryPolicy.KEEP_LAST, depth=10)
        self.vel_publisher_ = self.create_publisher(
            Twist,
            '/avoiding_twist',
            qos_profile=cmd_qos_profile
        )

        self.see_setpoint_sub_ = self.create_subscription(
            Bool,
            '/see_setpoint',  
            self.see_setpoint_cb,
            10
        )

        self.follow_side : str = None   # "left" albo "right"

        self.safe_dist = SAFE_DISTANCE        # kiedy uznajemy że coś blokuje
        self.target_wall_dist = TARGET_WALL_DIST
        self.linear_vel = OBSTACLE_AVOIDING_LINEAR_VEL
        self.angular_vel = OBSTACLE_AVOIDING_ANGULAR_VEL
        self.kp = KP_WALL_FOLLOW
        self.max_angular_vel = MAX_ANGULAR

        self.state : ControlState = ControlState.DRIVE

        self.see_setpoint = False

    def see_setpoint_cb(self, msg: Bool):
        self.see_setpoint = msg.data

    def get_min_dist_in_angle_range(self, scan, angle_min, angle_max):
        min_dist = float('inf')

        start_angle = scan.angle_min
        inc = scan.angle_increment

        for i, r in enumerate(scan.ranges):
            angle = start_angle + i * inc

            # normalizacja kąta do [-pi,pi]
            a = math.atan2(math.sin(angle), math.cos(angle))

            # normalizacja granic
            amin = math.atan2(math.sin(angle_min), math.cos(angle_min))
            amax = math.atan2(math.sin(angle_max), math.cos(angle_max))

            # obsługa przypadku, gdy przedział przekracza granicę -pi/pi
            in_range = False
            if amin <= amax:
                in_range = (amin <= a <= amax)
            else:
                in_range = (a >= amin or a <= amax)

            if in_range:
                if r is not None and not math.isinf(r) and not math.isnan(r):
                    if r >= 0.0:
                        min_dist = min(min_dist, r)

        return min_dist

    def lidar_cb(self, scan: LaserScan):
        left_min  = self.get_min_dist_in_angle_range(scan, LEFT_MIN,  LEFT_MAX)
        right_min = self.get_min_dist_in_angle_range(scan, RIGHT_MIN, RIGHT_MAX)
        front_min = self.get_min_dist_in_angle_range(scan, FRONT_MIN, FRONT_MAX)

        obstacle_front = (front_min < self.safe_dist)

        match self.state:
            case ControlState.DRIVE:
                # Najbliżesze odległości do przeszkód - z przodu, z lewej i z prawej
                
                # self.get_logger().info(f"Obstacle in front: {obstacle_front}")

                if obstacle_front:
                    self.get_logger().info("Obstacle detected!")
                    self.state = ControlState.OBSTACLE_DETECTED

            case ControlState.OBSTACLE_DETECTED:
                # Publish avoiding flag to disable automatic control in control node
                avoid_flag_msg = Bool()
                avoid_flag_msg.data = True
                self.avoid_flag_publisher_.publish(avoid_flag_msg)

                # Wybór strony
                if left_min < right_min:
                    self.follow_side = "left"    # z lewej szybciej - skręt w lewo
                else:
                    self.follow_side = "right"   # z prawej szybciej - skręt w prawo

                self.get_logger().info(f"Start avoiding - side: {self.follow_side}")

                self.state = ControlState.TURNING

            case ControlState.TURNING:
                # Initial turning phase - to rotate from obstacle
                cmd = Twist()
                cmd.linear.x = 0.0  # rotate in place to find wall on chosen side
                if self.follow_side == "right":
                    cmd.angular.z = -self.angular_vel  # negative = right
                    # check if we now "see" the wall on the left
                    if left_min < WALL_SEEN_DIST:
                        # start wall-following
                        self.state = ControlState.WALL_FOLLOWING
                        self.get_logger().info("Wall seen on left -> start following")
                else:  # left
                    cmd.angular.z = self.angular_vel  # positive = left
                    if right_min < WALL_SEEN_DIST:
                        self.state = ControlState.WALL_FOLLOWING
                        self.get_logger().info("Wall seen on right -> start following")

                self.vel_publisher_.publish(cmd)

            case ControlState.WALL_FOLLOWING:

                if self.follow_side == "right":
                    wall_dist = left_min if left_min != float('inf') else 10.0
                    error = self.target_wall_dist - wall_dist
                    ang = - self.kp * error   # negative -> turn right if too close to left wall
                else:  # left
                    wall_dist = right_min if right_min != float('inf') else 10.0
                    error = self.target_wall_dist - wall_dist
                    ang = + self.kp * error   # positive -> turn left if too close to right wall

                # limit angular and scale linear velocity down if big turning
                ang = min(self.max_angular_vel, ang)
                ang = max(-self.max_angular_vel, ang)

                # reduce forward speed when turning sharply
                forward = self.linear_vel * max(0.3, 1.0 - abs(ang) / self.max_angular_vel)
                
                cmd = Twist()
                cmd.linear.x = forward
                cmd.angular.z = ang

                self.vel_publisher_.publish(cmd)

                if self.see_setpoint and not obstacle_front:
                    self.state = ControlState.OBSTACLE_END

            case ControlState.OBSTACLE_END:
                # Publish avoiding flag to enable automatic control in control node
                avoid_flag_msg = Bool()
                avoid_flag_msg.data = False
                self.avoid_flag_publisher_.publish(avoid_flag_msg)

                self.get_logger().info("Obstacle passed!")
                self.state = ControlState.DRIVE

        return 
    

def main(args=None):
    rclpy.init(args=args)

    node = LaserScanController()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
