import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial
import time

ARDUINO_PORT = "/dev/ttyACM0"
BAUDRATE = 9600

IF_COMPLEX = 250
DISTANCE = 0

MAX_PWM = 255
MAX_VEL_LINEAR = 1.0      # m/s (do kalibracji)

# geometria robota (POŁOWY wymiarów)
HALF_LENGTH = 1.511 / 2        # L
HALF_WIDTH = 1.25 / 2         # W


def clamp(value, min_val, max_val):
    return max(min(value, max_val), min_val)


def normalize(values):
    max_val = max(abs(v) for v in values)
    if max_val > MAX_VEL_LINEAR:
        scale = MAX_VEL_LINEAR / max_val
        return [v * scale for v in values]
    return values


def velocity_to_pwm(v):
    v = clamp(v, -MAX_VEL_LINEAR, MAX_VEL_LINEAR)
    direction = 1 if v >= 0 else 0
    pwm = int(abs(v) / MAX_VEL_LINEAR * MAX_PWM)
    return pwm, direction


class ArduinoMsgsNode(Node):

    def __init__(self):
        super().__init__('arduino_msgs_node')

        self.arduino = serial.Serial(
            ARDUINO_PORT,
            BAUDRATE,
            timeout=0.01
        )
        time.sleep(2)

        self.subscription = self.create_subscription(
            Twist,
            '/model/vehicle_blue/cmd_vel',
            self.cmd_callback,
            10
        )

        self.get_logger().info("ArduinoMsgsNode uruchomiony")

    def cmd_callback(self, cmd_vel_msg: Twist):
        """
        Przelicza cmd_vel -> kinematyka mecanum -> ramka Arduino
        """

        vx = cmd_vel_msg.linear.x
        vy = cmd_vel_msg.linear.y
        omega = cmd_vel_msg.angular.z

        k = HALF_LENGTH + HALF_WIDTH

        # kinematyka mecanum
        v_lf = vx - vy - omega * k
        v_rf = vx + vy + omega * k
        v_lb = vx + vy - omega * k
        v_rb = vx - vy + omega * k

        # normalizacja
        v_lf, v_rf, v_lb, v_rb = normalize(
            [v_lf, v_rf, v_lb, v_rb]
        )

        # PWM + DIR
        lf_pwm, lf_dir = velocity_to_pwm(v_lf)
        rf_pwm, rf_dir = velocity_to_pwm(v_rf)
        lb_pwm, lb_dir = velocity_to_pwm(v_lb)
        rb_pwm, rb_dir = velocity_to_pwm(v_rb)

        # Ramka
        frame = bytes([
            IF_COMPLEX,
            DISTANCE,
            lf_pwm,
            rf_pwm,
            lb_pwm,
            rb_pwm,
            lf_dir,
            lb_dir,
            rf_dir,
            rb_dir
        ])

        self.arduino.write(frame)

        self.get_logger().debug(
            f"vx={vx:.2f}, vy={vy:.2f}, ω={omega:.2f} | "
            f"PWM: LF={lf_pwm}, RF={rf_pwm}, LB={lb_pwm}, RB={rb_pwm}"
        )


def main(args=None):
    rclpy.init(args=args)

    node = ArduinoMsgsNode()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
