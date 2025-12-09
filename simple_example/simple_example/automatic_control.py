import json
import rclpy
from rclpy.node import Node
import math
from geometry_msgs.msg import Point, Twist
from sensor_msgs.msg import Imu
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from std_msgs.msg import String
import casadi as ca
import numpy as np

def euler_from_quaternion(quaternion):
    """
    Converts quaternion (w in last place) to euler roll, pitch, yaw
    quaternion = [x, y, z, w]
    Bellow should be replaced when porting for ROS 2 Python tf_conversions is done.
    """
    x = quaternion.x
    y = quaternion.y
    z = quaternion.z
    w = quaternion.w

    sinr_cosp = 2 * (w * x + y * z)
    cosr_cosp = 1 - 2 * (x * x + y * y)
    roll = np.arctan2(sinr_cosp, cosr_cosp)

    sinp = 2 * (w * y - z * x)
    pitch = np.arcsin(sinp)

    siny_cosp = 2 * (w * z + x * y)
    cosy_cosp = 1 - 2 * (y * y + z * z)
    yaw = np.arctan2(siny_cosp, cosy_cosp)

    return roll, pitch, yaw
    
class MPCController:
    def __init__(self, N=500, dt=0.1, L=1.0, lr=0.5):
        self.N = N
        self.dt = dt
        self.L = L
        self.lr = lr
        x = ca.SX.sym('x')
        y = ca.SX.sym('y')
        theta = ca.SX.sym('theta')
        delta = ca.SX.sym('delta')
        v = ca.SX.sym('v')

        state = ca.vertcat(x, y, theta, delta, v)
        phi = ca.SX.sym('phi')
        a = ca.SX.sym('a')

        control = ca.vertcat(phi, a)
        beta = ca.atan((self.lr / self.L) * ca.tan(delta))
        dx = v * ca.cos(theta + beta)
        dy = v * ca.sin(theta + beta)
        dtheta = v * ca.sin(beta) / self.lr
        ddelta = phi
        dv = a

        rhs = ca.vertcat(dx, dy, dtheta, ddelta, dv)

        self.f = ca.Function("f", [state, control], [rhs])


    def compute_control(self, state, reference, dt, heading):
        x0, y0, th0, d0, v0 = state
        ref_x, ref_y, ref_vel = reference

        N = self.N
        X = ca.SX.sym('X', 5, N + 1)
        U = ca.SX.sym('U', 2, N)
        cost = 0
        for k in range(N):
            cost += 5000_000 * (X[0, k] - ref_x) ** 2      
            cost += 150 * (X[1, k] - ref_y) ** 2      
            cost += 100  * (X[4, k] - ref_vel) ** 2
            cost += 100 * (X[2, k] - heading) ** 2
            cost += 50 * (X[3, k] ** 2)
            cost += 1000 * (U[0, k] ** 2)             
            cost += 1000 * (U[1, k] ** 2)             

        g = []
        g.append(X[:, 0] - ca.vertcat(x0, y0, th0, d0, v0))

        for k in range(N):
            x_next = X[:, k] + dt * self.f(X[:, k], U[:, k])
            g.append(X[:, k + 1] - x_next)

        g = ca.vertcat(*g)

        vars = ca.vertcat(X.reshape((-1, 1)), U.reshape((-1, 1)))

        solver = ca.nlpsol("solver", "ipopt",
        {
            "f": cost,
            "g": g,
            "x": vars
        },
        {
            "ipopt.print_level": 0,
            "print_time": 0
        })

        sol = solver(lbg=0, ubg=0)
        opt = sol["x"].full().flatten()

        U_opt = opt[-2 * N:]
        phi = U_opt[0]
        accel = U_opt[1]

        return phi, accel


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

        self.const_velocity = (2.0, 0.0, 0.0)
        self.vehicle_length = 1.0
        self.surround_area = {'front': False, 'back': False, 'left': False, 'right': False}
        self.emergency = False
        self.latest_auto_twist = Twist()
        self.mpc = MPCController(L=self.vehicle_length)
        self.steering_angle = 0.0
        self.heading = 0.0
        self.pid = PID(kp=2, ki=0.5, kd=0.02)

        self.velocity = {'x': 0.0, 'y': 0.0, 'z': 0.0}
        self.prev_time = self.get_clock().now().seconds_nanoseconds()[0]
        self.prev_time_mpc = self.get_clock().now().seconds_nanoseconds()[0] + self.get_clock().now().seconds_nanoseconds()[1] / 1e9
        self.velocity_destined = 2.0

    def point_callback(self, point: Point):
        msg = Twist()
        vel_delta = self.velocity['x'] - self.velocity_destined
        current_time = self.get_clock().now().seconds_nanoseconds()[0] + self.get_clock().now().seconds_nanoseconds()[1] / 1e9
        dt = current_time - self.prev_time_mpc
        self.prev_time_mpc = current_time
        msg.linear.x = 0.0 + self.velocity_destined
        msg.linear.y = 0.0
        msg.linear.z = 0.0
        dist = point.y
        e = -point.x

        # calc_angle = self.pid(e)

        # msg.angular.x = 0.0
        # msg.angular.y = 0.0
        # msg.angular.z = float(calc_angle)

        state = [
        -e,                
        dist,               
        self.heading,
        self.steering_angle,                
        self.velocity['x']  
        ]
        reference = [0.0, 0.0, self.velocity_destined]
        steer, accel = self.mpc.compute_control(state, reference, dt, self.heading)
        self.steering_angle += steer * dt
        msg.angular.x = float(accel + self.velocity['x'])
        msg.angular.y = 0.0
        msg.angular.z = float(steer)

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
    
    def imu_callback(self, msg: Imu):
        current_time = self.get_clock().now().seconds_nanoseconds()[0] + self.get_clock().now().seconds_nanoseconds()[1] / 1e9
        dt = (current_time - self.prev_time)
        acc_x = msg.linear_acceleration.x
        acc_y = msg.linear_acceleration.y
        acc_z = msg.linear_acceleration.z

        self.velocity['x'] += acc_x * dt
        try:
            self.velocity['y'] += acc_y * dt
            self.velocity['z'] += acc_z * dt

            self.prev_time = current_time
            q = msg.orientation
            roll, pitch, yaw = euler_from_quaternion(q)
            self.heading = yaw
        except Exception as e:
            self.get_logger().error(f"Error in IMU callback: {e}")
