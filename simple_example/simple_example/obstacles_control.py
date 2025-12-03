import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from rclpy.qos import QoSProfile, QoSHistoryPolicy
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
import math


OBSTACLE_AVOIDING_LINEAR_VEL = 0.5
OBSTACLE_AVOIDING_ANGULAR_VEL = 0.01
SAFE_DISTANCE = 1.0
TARGET_WALL_DIST = 0.6


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

        self.avoiding = False
        self.follow_side : str = None   # "left" albo "right"

        self.safe_dist = SAFE_DISTANCE        # kiedy uznajemy że coś blokuje
        self.target_wall_dist = TARGET_WALL_DIST
        self.linear_vel = OBSTACLE_AVOIDING_LINEAR_VEL
        self.angular_vel = OBSTACLE_AVOIDING_ANGULAR_VEL


    def get_min_dist_in_angle_range(self, scan, angle_min, angle_max):
        min_dist = float('inf')

        for i, r in enumerate(scan.ranges):
            angle = scan.angle_min + i * scan.angle_increment

            if angle_min <= angle <= angle_max:
                if not math.isinf(r) and not math.isnan(r):
                    min_dist = min(min_dist, r)

        return min_dist

    def lidar_cb(self, scan: LaserScan):
        # angles
        FRONT_MIN = -math.radians(15)
        FRONT_MAX =  math.radians(15)

        LEFT_MIN  =  math.radians(30)
        LEFT_MAX  =  math.radians(90)

        RIGHT_MIN = -math.radians(90)
        RIGHT_MAX = -math.radians(30)

        # Najbliżesze odległości do przeszkód - z przodu, z lewej i z prawej
        front_min = self.get_min_dist_in_angle_range(scan, FRONT_MIN, FRONT_MAX)
        left_min  = self.get_min_dist_in_angle_range(scan, LEFT_MIN,  LEFT_MAX)
        right_min = self.get_min_dist_in_angle_range(scan, RIGHT_MIN, RIGHT_MAX)

        obstacle_front = front_min < self.safe_dist
        self.get_logger().info(f"Obstacle in front: {obstacle_front}")

        # Wybór strony
        if obstacle_front and not self.avoiding:

            self.avoiding = True
            avoid_flag_msg = Bool()
            avoid_flag_msg.data = True
            self.avoid_flag_publisher_.publish(avoid_flag_msg)

            if left_min > right_min:
                self.follow_side = "left"    # z lewej szybciej - skręt w lewo
            else:
                self.follow_side = "right"   # z prawej szybciej - skręt w prawo

            self.get_logger().info(f"Obstacle detected! - start avoiding - side: {self.follow_side}")

        # Tryb omijania
        if self.avoiding:

            vel_msg = Twist()

            if self.follow_side == "left":
                wall_dist = left_min
                error = self.target_wall_dist - wall_dist
                vel_msg.linear.x = self.linear_vel
                vel_msg.angular.z = -self.angular_vel * error   # ujemne → korekta w prawo

            elif self.follow_side == "right":
                wall_dist = right_min
                error = self.target_wall_dist - wall_dist
                vel_msg.linear.x = self.linear_vel
                vel_msg.angular.z = +self.angular_vel * error   # dodatnie → korekta w lewo
            
            # Warunek zakończenia
            # if front_min > self.angular_vel and wall_dist > 1.2:
            #     self.avoiding = False
            #     self.follow_side = None
            #     self.get_logger().info("END AVOIDING")
                # avoid_flag_msg = Bool()
                # avoid_flag_msg.data = False
                # self.avoid_flag_publisher.publish(avoid_flag_msg)

            self.vel_publisher_.publish(vel_msg)

            return


def main(args=None):
    rclpy.init(args=args)

    node = LaserScanController()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
