import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial
import time

# Configuration
SERIAL_PORT = '/dev/ttyACM0'
BAUD_RATE = 9600
PWM_SCALE = 200.0
ROTATION_SCALE = 1.0

class ArduinoDriver(Node):
    def __init__(self):
        super().__init__('motor_driver')
        
        # State variables
        self.serial_connected = False
        self.arduino = None
        self.last_cmd_time = self.get_clock().now()
        
        # Attempt to connect to Serial
        try:
            self.arduino = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.1)
            time.sleep(2) # Wait for Arduino reset
            self.serial_connected = True
            self.get_logger().info(f"Connected to Arduino on {SERIAL_PORT}")
        except Exception as e:
            self.get_logger().warn(f"Serial connection failed: {e}. Running in headless mode.")
            self.serial_connected = False

        # Subscribers
        self.subscription = self.create_subscription(
            Twist,
            '/model/vehicle_blue/cmd_vel',
            self.cmd_vel_callback,
            10
        )
        
        # Safety Watchdog
        self.timer = self.create_timer(0.1, self.watchdog)

    def cmd_vel_callback(self, msg):
        """Convert Twist message to Differential Drive motor commands"""
        if not self.serial_connected:
            return

        self.last_cmd_time = self.get_clock().now()
        
        linear_x = msg.linear.x
        angular_z = msg.angular.z
        
        # Calculate motor values
        left_motor_val = (linear_x - (angular_z * ROTATION_SCALE)) * PWM_SCALE
        right_motor_val = (linear_x + (angular_z * ROTATION_SCALE)) * PWM_SCALE
        
        self.send_packet(left_motor_val, right_motor_val)

    def send_packet(self, left_val, right_val):
        """Constructs and sends the data packet to Arduino"""
        if not self.serial_connected:
            return

        # Clamp values between 0 and 255
        speed_l = int(abs(left_val))
        speed_r = int(abs(right_val))
        
        speed_l = max(0, min(255, speed_l))
        speed_r = max(0, min(255, speed_r))
        
        # Determine direction (1 for forward, 0 for backward usually, logic preserved from original)
        if left_val >= 0:
            d_left = 1
        else:
            d_left = 0
            
        if right_val >= 0:
            d_right = 0
        else:
            d_right = 1
            
        # Map to specific packet bytes
        sTL, sBL = speed_l, speed_l
        sTR, sBR = speed_r, speed_r
        
        dTL, dBL = d_left, d_left
        dTR, dBR = d_right, d_right
        
        mode = 240 # Start byte / mode
        distance = 0
        
        packet = [mode, distance, sTL, sTR, sBL, sBR, dTL, dTR, dBL, dBR]
        
        try:
            self.arduino.write(bytes(packet))
        except Exception as e:
            self.get_logger().warn(f"Failed to write to serial: {e}")

    def watchdog(self):
        """Stops motors if no command received for 0.5 seconds"""
        if not self.serial_connected:
            return

        time_diff = (self.get_clock().now() - self.last_cmd_time).nanoseconds / 1e9
        if time_diff > 0.5:
            self.send_packet(0, 0)
    
    def cleanup(self):
        """Close serial port on shutdown"""
        if self.arduino and self.arduino.is_open:
            self.arduino.close()
            self.get_logger().info("Serial port closed.")

def main(args=None):
    rclpy.init(args=args)
    
    node = MotorDriver()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cleanup()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()