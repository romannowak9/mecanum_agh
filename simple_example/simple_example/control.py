import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import termios
import tty
import sys
import select


class KeyboardCarController(Node):

    def __init__(self):
        super().__init__('keyboard_car_controller')
        self.publisher_ = self.create_publisher(Twist, '/model/vehicle_blue/cmd_vel', 10)
        
        # Control parameters
        self.linear_speed = 1.0
        self.angular_speed = 1.0
        self.current_twist = Twist()
        
        # Instructions
        self.print_instructions()
        
        # Set up terminal for keyboard input
        self.old_settings = termios.tcgetattr(sys.stdin)
        
    def print_instructions(self):
        print("\n=== Car Keyboard Controller ===")
        print("Controls:")
        print("  W - Move forward")
        print("  S - Move backward")
        print("  A - Turn left")
        print("  D - Turn right")
        print("  Q - Stop and quit")
        print("  X - Emergency stop")
        print("==============================\n")

    def get_key(self):
        """Get a single key press without requiring Enter"""
        try:
            tty.setraw(sys.stdin.fileno())
            if select.select([sys.stdin], [], [], 0.1)[0]:
                key = sys.stdin.read(1)
                return key
            return None
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_settings)

    def process_keyboard_input(self):
        """Process keyboard input and update velocity commands"""
        key = self.get_key()
        
        if key is None:
            return True  # Continue running
        
        key = key.lower()
        
        # Reset twist
        self.current_twist = Twist()
        
        if key == 'w':
            # Move forward
            self.current_twist.linear.x = self.linear_speed
            print("Moving forward")
            
        elif key == 's':
            # Move backward
            self.current_twist.linear.x = -self.linear_speed
            print("Moving backward")
            
        elif key == 'a':
            # Turn left (while moving forward)
            self.current_twist.linear.x = self.linear_speed * 0.5
            self.current_twist.angular.z = self.angular_speed
            print("Turning left")
            
        elif key == 'd':
            # Turn right (while moving forward)
            self.current_twist.linear.x = self.linear_speed * 0.5
            self.current_twist.angular.z = -self.angular_speed
            print("Turning right")
            
        elif key == 'x':
            # Emergency stop
            self.current_twist = Twist()
            print("EMERGENCY STOP!")
            
        elif key == 'q':
            # Quit
            self.current_twist = Twist()
            print("Stopping and quitting...")
            return False
            
        else:
            # Stop if any other key is pressed
            self.current_twist = Twist()
            print("Stopped")
        
        # Publish the velocity command
        self.publisher_.publish(self.current_twist)
        return True

    def stop(self):
        """Stop the vehicle"""
        stop_twist = Twist()
        self.publisher_.publish(stop_twist)
        print("Vehicle stopped")

    def cleanup(self):
        """Restore terminal settings"""
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.old_settings)


def main(args=None):
    rclpy.init(args=args)

    controller = KeyboardCarController()

    try:
        print("Controller started. Press keys to control the vehicle...")
        
        # Main loop
        while rclpy.ok():
            # Spin once to handle any callbacks
            rclpy.spin_once(controller, timeout_sec=0.01)
            
            # Process keyboard input
            if not controller.process_keyboard_input():
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