import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial
import time

SERIAL_PORT = '/dev/ttyACM0'
BAUD_RATE = 9600
PWM_SCALE = 200.0
ROTATION_SCALE = 1.0

class MotorDriver(Node):
    def __init__(self):
        super().__init__('motor_driver')
        self.serial_connected = False
        self.arduino = None
        self.last_cmd_time = self.get_clock().now()
        
        try:
            self.arduino = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.1)
            time.sleep(2)
            self.serial_connected = True
            self.get_logger().info(f"Connected to Arduino on {SERIAL_PORT}")
        except Exception as e:
            self.get_logger().warn(f"Serial connection failed: {e}. Running in headless mode.")
            self.serial_connected = False

        self.subscription = self.create_subscription(
            Twist,
            '/model/vehicle_blue/cmd_vel',
            self.cmd_vel_callback,
            10
        )
        
        self.timer = self.create_timer(0.1, self.watchdog)

    def cmd_vel_callback(self, msg):
        if not self.serial_connected:
            return

        self.last_cmd_time = self.get_clock().now()
        
        linear_x = msg.linear.x
        angular_z = msg.angular.z
        
        left_motor_val = (linear_x - (angular_z * ROTATION_SCALE)) * PWM_SCALE
        right_motor_val = (linear_x + (angular_z * ROTATION_SCALE)) * PWM_SCALE
        
        self.send_packet(left_motor_val, right_motor_val)

    def send_packet(self, left_val, right_val):
        if not self.serial_connected:
            return

        speed_l = int(abs(left_val))
        speed_r = int(abs(right_val))
        
        speed_l = max(0, min(255, speed_l))
        speed_r = max(0, min(255, speed_r))
        
        if left_val >= 0:
            d_left = 1
        else:
            d_left = 0
            
        if right_val >= 0:
            d_right = 0
        else:
            d_right = 1
            
        sTL, sBL = speed_l, speed_l
        sTR, sBR = speed_r, speed_r
        
        dTL, dBL = d_left, d_left
        dTR, dBR = d_right, d_right
        
        mode = 240
        distance = 0
        
        packet = [mode, distance, sTL, sTR, sBL, sBR, dTL, dTR, dBL, dBR]
        
        try:
            self.arduino.write(bytes(packet))
        except Exception:
            pass

    def watchdog(self):
        if not self.serial_connected:
            return

        time_diff = (self.get_clock().now() - self.last_cmd_time).nanoseconds / 1e9
        if time_diff > 0.5:
            self.send_packet(0, 0)
    
    def cleanup(self):
        if self.arduino and self.arduino.is_open:
            self.arduino.close()