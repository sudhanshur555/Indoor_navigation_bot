import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time
import math

class CalibrationNode(Node):
    def __init__(self):
        super().__init__('calibration_node')
        
        # Declare parameters for movement type, distance, speed, etc.
        self.declare_parameter('move_type', 'forward') # 'forward' or 'rotate'
        self.declare_parameter('distance_m', 2.0)
        self.declare_parameter('speed_mps', 0.2)
        self.declare_parameter('rotations', 1.0)
        self.declare_parameter('rot_speed_rps', 0.5) # Radians per second
        
        # Get parameters
        self.move_type = self.get_parameter('move_type').get_parameter_value().string_value
        self.distance = self.get_parameter('distance_m').get_parameter_value().double_value
        self.speed = self.get_parameter('speed_mps').get_parameter_value().double_value
        self.rotations = self.get_parameter('rotations').get_parameter_value().double_value
        self.rot_speed = self.get_parameter('rot_speed_rps').get_parameter_value().double_value
        
        # Create a publisher to the cmd_vel topic
        self.publisher_ = self.create_publisher(Twist, '/mybot_controller/cmd_vel_unstamped', 10)
        
        # Wait a moment for the publisher to establish a connection
        time.sleep(1.0)

    def move_forward(self):
        self.get_logger().info(f"Starting FORWARD calibration: moving {self.distance} meters at {self.speed} m/s.")
        if self.speed <= 0:
            self.get_logger().error("Speed must be a positive value.")
            return
            
        duration = self.distance / self.speed
        self.get_logger().info(f"Movement will take {duration:.2f} seconds.")
        twist_msg = Twist()
        twist_msg.linear.x = self.speed
        
        start_time = time.time()
        while rclpy.ok() and time.time() - start_time < duration:
            self.publisher_.publish(twist_msg)
            time.sleep(0.1)

        # Send a final stop message
        self.publisher_.publish(Twist())
        self.get_logger().info("Forward movement finished. Robot stopped.")

    def rotate(self):
        self.get_logger().info(f"Starting ROTATION calibration: performing {self.rotations} rotation(s) at {self.rot_speed} rad/s.")
        if self.rot_speed <= 0:
            self.get_logger().error("Rotation speed must be a positive value.")
            return

        # One full rotation is 2*PI radians
        total_angle = self.rotations * 2.0 * math.pi
        duration = total_angle / self.rot_speed
        self.get_logger().info(f"Rotation will take {duration:.2f} seconds.")
        
        twist_msg = Twist()
        twist_msg.angular.z = self.rot_speed
        
        start_time = time.time()
        while rclpy.ok() and time.time() - start_time < duration:
            self.publisher_.publish(twist_msg)
            time.sleep(0.1)

        # Send a final stop message
        self.publisher_.publish(Twist())
        self.get_logger().info("Rotation finished. Robot stopped.")

def main(args=None):
    rclpy.init(args=args)
    
    node = CalibrationNode()
    
    # Run the chosen logic based on the parameter
    if node.move_type == 'forward':
        node.move_forward()
    elif node.move_type == 'rotate':
        node.rotate()
    else:
        node.get_logger().error(f"Invalid move_type: {node.move_type}. Choose 'forward' or 'rotate'.")

    # Cleanly destroy the node before exiting
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()