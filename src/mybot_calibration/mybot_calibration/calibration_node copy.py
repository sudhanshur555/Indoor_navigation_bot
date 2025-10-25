import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import time

class CalibrationNode(Node):
    def __init__(self):
        super().__init__('calibration_node')
        
        # Declare parameters for distance and speed
        self.declare_parameter('distance_m', 2.0)
        self.declare_parameter('speed_mps', 0.2)
        
        # Get parameters
        self.distance = self.get_parameter('distance_m').get_parameter_value().double_value
        self.speed = self.get_parameter('speed_mps').get_parameter_value().double_value
        
        # Create a publisher to the cmd_vel topic
        self.publisher_ = self.create_publisher(Twist, '/mybot_controller/cmd_vel_unstamped', 10)
        
        self.get_logger().info(f"Starting calibration: moving forward {self.distance} meters at {self.speed} m/s.")
        
        # Wait a moment for the publisher to establish a connection
        time.sleep(1.0)

    def move_forward(self):
        # Calculate the duration needed to cover the distance
        if self.speed <= 0:
            self.get_logger().error("Speed must be a positive value.")
            return
            
        duration = self.distance / self.speed
        self.get_logger().info(f"Movement will take {duration:.2f} seconds.")

        # Create the Twist message
        twist_msg = Twist()
        twist_msg.linear.x = self.speed
        
        # Publish the message at a steady rate
        start_time = time.time()
        while rclpy.ok() and time.time() - start_time < duration:
            self.publisher_.publish(twist_msg)
            time.sleep(0.1) # Publish at 10 Hz

        # Send a final stop message
        stop_msg = Twist()
        self.publisher_.publish(stop_msg)
        self.get_logger().info("Calibration movement finished. Robot stopped.")
        

def main(args=None):
    rclpy.init(args=args)
    
    # Create the node
    node = CalibrationNode()
    
    # Run the movement logic
    node.move_forward()
    
    # Cleanly destroy the node before exiting
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()