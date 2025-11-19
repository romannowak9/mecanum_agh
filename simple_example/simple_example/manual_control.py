import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import pygame
import sys

class GamepadCarController(Node):

    def __init__(self):
        super().__init__('gamepad_car_controller')
        self.publisher_ = self.create_publisher(Twist, '/model/vehicle_blue/cmd_vel', 10)
        
        self.linear_speed = 6.0
        self.angular_speed = 3.0
    
        self.current_twist = Twist()

        pygame.init()
        self.joystick_count = pygame.joystick.get_count()
        
        if self.joystick_count == 0:
            print("No gamepad connected. Exiting...")
            sys.exit()
        
        self.joystick = pygame.joystick.Joystick(0)
        self.joystick.init()
        
        # Instructions
        self.print_instructions()

    def print_instructions(self):
        print("\n=== Car Gamepad Controller ===")
        print("Controls:")
        print("  Left Joystick - Move (Up/Down for forward/backward, Left/Right for turning)")
        print("  Button X - Stop")
        print("==============================\n")

    def process_gamepad_input(self):
        """Process gamepad input and update velocity commands"""
        pygame.event.pump()
        
        forward_backward = self.joystick.get_axis(1)
        left_right = self.joystick.get_axis(0)
        
        if abs(forward_backward) > 0.1:
            self.current_twist.linear.x = self.linear_speed * -forward_backward
        else:
            self.current_twist.linear.x = 0.0
        
        if abs(left_right) > 0.1:
            self.current_twist.angular.z = self.angular_speed * (-left_right)
        else:
            self.current_twist.angular.z = 0.0
        
        stop_button = self.joystick.get_button(0)

        if stop_button:
            self.current_twist.linear.x = 0.0
            self.current_twist.angular.z = 0.0
        
        
        self.publisher_.publish(self.current_twist)
        return True

    def stop(self):
        """Stop the vehicle"""
        stop_twist = Twist()
        self.publisher_.publish(stop_twist)
        print("Vehicle stopped")

    def cleanup(self):
        """Cleanup pygame"""
        pygame.quit()

def main(args=None):
    rclpy.init(args=args)

    controller = GamepadCarController()

    try:
        print("Controller started. Use the gamepad to control the vehicle...")
        
        while rclpy.ok():
            rclpy.spin_once(controller, timeout_sec=0.01)
            
            if not controller.process_gamepad_input():
                break
                
    except KeyboardInterrupt:
        print("\nInterrupted by user")
    except Exception as e:
        print(f"An error occurred: {e}")
        controller.stop()
        controller.cleanup()
        controller.destroy_node()
        rclpy.shutdown()
        print("Controller shutdown complete")

if __name__ == '__main__':
    main()
