import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import pygame
import sys

class GamepadCarController(Node):

    def __init__(self):
        super().__init__('gamepad_car_controller')
        self.publisher_ = self.create_publisher(Twist, '/model/vehicle_blue/cmd_vel', 10)
        
        # Control parameters
        self.linear_speed = 1.0
        self.angular_speed = 1.0
        self.current_twist = Twist()

        # Initialize pygame and the joystick
        pygame.init()
        self.joystick_count = pygame.joystick.get_count()
        
        if self.joystick_count == 0:
            print("No gamepad connected. Exiting...")
            sys.exit()
        
        # Assuming the first joystick is the one to be used
        self.joystick = pygame.joystick.Joystick(0)
        self.joystick.init()
        
        # Instructions
        self.print_instructions()

    def print_instructions(self):
        print("\n=== Car Gamepad Controller ===")
        print("Controls:")
        print("  Left Joystick - Move (Up/Down for forward/backward, Left/Right for turning)")
        print("  Right Trigger - Move forward")
        print("  Left Trigger - Move backward")
        print("  Buttons (O, X) for emergency stop and quit")
        print("==============================\n")

    def process_gamepad_input(self):
        """Process gamepad input and update velocity commands"""
        pygame.event.pump()  # Process any events
        
        # Get joystick axis values (range is typically -1 to 1)
        forward_backward = self.joystick.get_axis(1)  # Axis 1 controls forward/backward
        left_right = self.joystick.get_axis(0)  # Axis 0 controls left/right turn
        
        # Adjust the values from the joystick input range to control speeds
        if abs(forward_backward) > 0.1:
            self.current_twist.linear.x = self.linear_speed * -forward_backward  # Invert to match expected direction
        else:
            self.current_twist.linear.x = 0
        
        if abs(left_right) > 0.1:
            self.current_twist.angular.z = self.angular_speed * left_right
        
        # Check if trigger buttons are pressed (for emergency stop and quit)
        button_stop = self.joystick.get_button(0)  # Assuming Button 0 is emergency stop
        button_quit = self.joystick.get_button(1)  # Assuming Button 1 is quit
        
        if button_stop:
            self.current_twist = Twist()
            print("EMERGENCY STOP!")
        
        if button_quit:
            self.current_twist = Twist()
            print("Stopping and quitting...")
            return False
        
        # Publish the velocity command
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
        
        # Main loop
        while rclpy.ok():
            # Spin once to handle any callbacks
            rclpy.spin_once(controller, timeout_sec=0.01)
            
            # Process gamepad input
            if not controller.process_gamepad_input():
                break
                
    except KeyboardInterrupt:
        print("\nInterrupted by user")
    except Exception as e:
        print(f"An error occurred: {e}")
    finally:
        # Cleanup
        controller.stop()
        controller.cleanup()
        controller.destroy_node()
        rclpy.shutdown()
        print("Controller shutdown complete")

if __name__ == '__main__':
    main()
