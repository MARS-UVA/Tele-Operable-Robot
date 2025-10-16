import rclpy
from rclpy.node import Node
# The standard message for robot velocity commands
from geometry_msgs.msg import Twist 
# Use Float64 for single motor velocity commands, or define a custom message
from std_msgs.msg import Float64 
# Assuming your signal processing classes are in a file named 'signal_processing.py'
from .signal_processing import Deadband, Clamp 

# Define a class for your robot's driver node
class MessageSubscriber(Node):

    def __init__(self):
        super().__init__('simple_subscriber')

        # --- 1. Signal Processing Setup ---
        # Initialize your signal transforms (e.g., a deadband for stability)
        self.linear_deadband = Deadband(min_magnitude=0.05) # Ignore small joystick noise
        self.angular_deadband = Deadband(min_magnitude=0.05)
        self.velocity_clamp = Clamp(min_value=-0.5, max_value=0.5) # Limit max motor velocity to 0.5 m/s

        # --- 2. Subscriber Setup ---
        # Subscribe to the standard velocity command topic, typically 'cmd_vel'
        self.subscription = self.create_subscription(
            Twist,              # Message type is Twist
            'cmd_vel',          # Standard topic for velocity commands
            self.cmd_vel_callback, # The function to call when a message is received
            10)
        self.subscription  

        # --- 3. Publisher Setup (Motor Commands) ---
        # You need publishers for the actual motor control. 
        # A simple robot might have a left and a right wheel.
        self.left_motor_pub = self.create_publisher(Float64, 'left_motor_velocity', 10)
        self.right_motor_pub = self.create_publisher(Float64, 'right_motor_velocity', 10)

        self.get_logger().info('Robot Driver node started. Waiting for cmd_vel...')


    def cmd_vel_callback(self, msg: Twist):
        """Processes the incoming Twist message and publishes motor commands."""
        
        # Extract the relevant components: linear.x (forward/backward) and angular.z (turning)
        linear_x = msg.linear.x
        angular_z = msg.angular.z

        self.get_logger().debug(f'Received Twist: Linear={linear_x:.2f}, Angular={angular_z:.2f}')

        # --- 4. Apply Signal Processing ---
        
        # Apply deadband to ignore noise near zero
        processed_linear = self.linear_deadband(linear_x)
        processed_angular = self.angular_deadband(angular_z)

        # --- 5. Differential Drive Kinematics (Conversion to Motor Speed) ---
        # This is the core logic to convert a desired robot velocity (Twist) 
        # into individual motor velocities for a two-wheel (differential drive) robot.
        
        # Assume 'base_width' is the distance between your wheels (you'll need to measure this)
        # and 'W' is a placeholder for this value. The simplest kinematic model is:
        
        # left_vel = linear_vel - (angular_vel * W / 2)
        # right_vel = linear_vel + (angular_vel * W / 2)
        
        # For simplicity, let's omit the width (W) and use a direct sum/difference model:
        left_vel_raw = processed_linear - processed_angular
        right_vel_raw = processed_linear + processed_angular
        
        # --- 6. Clamp Motor Velocities ---
        
        # Apply the overall max/min velocity limit to the motor commands
        left_vel = self.velocity_clamp(left_vel_raw)
        right_vel = self.velocity_clamp(right_vel_raw)
        
        # --- 7. Publish Motor Commands ---
        
        # Create and populate the message objects
        left_msg = Float64()
        left_msg.data = left_vel
        
        right_msg = Float64()
        right_msg.data = right_vel

        # Publish the final commands
        self.left_motor_pub.publish(left_msg)
        self.right_motor_pub.publish(right_msg)

        self.get_logger().info(f'Motor Commands: Left={left_vel:.2f}, Right={right_vel:.2f}')


def main(args=None):
    rclpy.init(args=args)
    robot_driver = RobotDriver()
    rclpy.spin(robot_driver)
    robot_driver.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()