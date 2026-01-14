import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool
from rclpy.qos import QoSProfile, QoSHistoryPolicy
import pygame
import sys

# Imports from your package
from .automatic_control import AutomaticController
from .arduino_driver import ArduinoDriver

class MasterController(Node):
    def __init__(self, auto_node_ref):
        super().__init__('master_controller')
        
        self.auto_node = auto_node_ref

        cmd_qos_profile = QoSProfile(history=QoSHistoryPolicy.KEEP_LAST, depth=10)
        self.publisher_ = self.create_publisher(Twist, '/model/vehicle_blue/cmd_vel', qos_profile=cmd_qos_profile)

        self.avoid_flag_sub_ = self.create_subscription(
            Bool,
            '/avoiding_obstacle',  
            self.avoid_flag_cb,
            10
        )

        self.avoid_vel_sub_ = self.create_subscription(
            Twist,
            '/avoiding_twist',  
            self.avoid_vel_cb,
            10
        )

        self.manual_linear_speed = 6.0
        self.manual_angular_speed = 3.0
        self.manual_twist = Twist()

        self.joystick_connected = False
        self.mode = 'AUTO'
        self.button_previous_state = False 

        self.avoiding_obstacle = False
        self.avoiding_twist = Twist()

        pygame.init()
        if pygame.joystick.get_count() > 0:
            self.joystick = pygame.joystick.Joystick(0)
            self.joystick.init()
            self.joystick_connected = True
            self.mode = 'MANUAL'
            print(f"Gamepad detected: {self.joystick.get_name()}")
        else:
            self.joystick_connected = False
            self.mode = 'AUTO'
            print("No gamepad detected. Running in automatic mode only.")

        self.print_instructions()
        
        self.timer = self.create_timer(0.05, self.control_loop)
    
    def avoid_flag_cb(self, msg: Bool):
        self.avoiding_obstacle = msg.data

    def avoid_vel_cb(self, msg: Twist):
        self.avoiding_twist = msg

    def print_instructions(self):
        print("\n=== Master Control Node ===")
        print(f"Current Mode: {self.mode}")
        if self.joystick_connected:
            print("Controls:")
            print("  Left Joystick - Move (Manual Mode)")
            print("  Button 0 (A/X) - Stop")
            print("  Button 1 (B/O) - Switch Modes (Manual <-> Auto)")
        else:
            print("Running in headless Automatic mode.")
        print("===========================\n")

    def control_loop(self):
        pygame.event.pump()
        
        if self.joystick_connected:
            mode_button = self.joystick.get_button(1)
            
            if mode_button and not self.button_previous_state:
                if self.mode == 'MANUAL':
                    self.mode = 'AUTO'
                    self.stop()
                else:
                    self.mode = 'MANUAL'
                    self.stop()
                print(f"Switched mode to: {self.mode}")
            
            self.button_previous_state = mode_button

        if self.mode == 'AUTO':
            if not self.avoiding_obstacle:
                cmd = self.auto_node.latest_auto_twist
            else:
                cmd = self.avoiding_twist

            self.publisher_.publish(cmd)
            
        elif self.mode == 'MANUAL' and self.joystick_connected:
            self.process_manual_input()
            self.publisher_.publish(self.manual_twist)

    def process_manual_input(self):
        forward_backward = self.joystick.get_axis(1)
        left_right = self.joystick.get_axis(0)
        stop_button = self.joystick.get_button(0)

        if stop_button:
            self.manual_twist.linear.x = 0.0
            self.manual_twist.angular.z = 0.0
        else:
            if abs(forward_backward) > 0.1:
                self.manual_twist.linear.x = self.manual_linear_speed * -forward_backward
            else:
                self.manual_twist.linear.x = 0.0
            
            if abs(left_right) > 0.1:
                self.manual_twist.angular.z = self.manual_angular_speed * (-left_right)
            else:
                self.manual_twist.angular.z = 0.0

    def stop(self):
        stop_twist = Twist()
        self.publisher_.publish(stop_twist)

    def cleanup(self):
        if self.joystick_connected:
            pygame.quit()

def main(args=None):
    rclpy.init(args=args)

    auto_node = AutomaticController()
    master_node = MasterController(auto_node_ref=auto_node)
    motor_node = ArduinoDriver()

    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(auto_node)
    executor.add_node(master_node)
    executor.add_node(motor_node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        print("\nInterrupted by user")
    except Exception as e:
        print(f"An error occurred: {e}")
    finally:
        master_node.cleanup()
        motor_node.cleanup()
        master_node.destroy_node()
        auto_node.destroy_node()
        motor_node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()