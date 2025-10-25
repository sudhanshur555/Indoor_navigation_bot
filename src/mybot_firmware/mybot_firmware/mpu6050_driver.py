#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu
import time

# Use the correct library for software I2C to prevent hardware crashes
from adafruit_extended_bus import ExtendedI2C
import adafruit_bno08x
from adafruit_bno08x.i2c import BNO08X_I2C

# I2C address of BNO085
DEVICE_ADDRESS = 0x4a
# Software I2C bus number
I2C_BUS = 3

class BNO085_Driver(Node):

    def __init__(self):
        super().__init__("bno085_driver")
        
        # I2C and sensor state
        self.sensor = None
        self.is_connected_ = False
        self.last_try = 0.0

        # ROS 2 IMU publisher
        self.imu_pub_ = self.create_publisher(Imu, "/imu/data", qos_profile=qos_profile_sensor_data)
        self.imu_msg_ = Imu()
        # Use a standard frame_id that should match your URDF
        self.imu_msg_.header.frame_id = "imu_link" 

        # Try to initialize the sensor at startup
        self.init_i2c()
        
        # Timer frequency (50 Hz is a good rate for this sensor)
        self.frequency_ = 0.01
        self.timer_ = self.create_timer(self.frequency_, self.timerCallback)

        self.get_logger().info("BNO085 Driver Node Initialized")

    def init_i2c(self):
        try:
            self.get_logger().info(f"Attempting to initialize BNO085 on software I2C bus {I2C_BUS}...")
            # Use ExtendedI2C for the software I2C bus
            i2c = ExtendedI2C(I2C_BUS)
            self.sensor = BNO08X_I2C(i2c, address=DEVICE_ADDRESS)

            # Enable all necessary reports for sensor fusion
            self.sensor.enable_feature(adafruit_bno08x.BNO_REPORT_ACCELEROMETER)
            self.sensor.enable_feature(adafruit_bno08x.BNO_REPORT_GYROSCOPE)
            self.sensor.enable_feature(adafruit_bno08x.BNO_REPORT_ROTATION_VECTOR)

            self.is_connected_ = True
            self.get_logger().info("BNO085 sensor connected and reports enabled.")

        except Exception as e:
            self.sensor = None
            self.is_connected_ = False
            self.get_logger().error(f"Failed to initialize BNO085: {str(e)}")

    def timerCallback(self):
        # Retry initialization only once per second if disconnected
        if not self.is_connected_:
            now = time.time()
            if now - self.last_try < 1.0:
                return
            self.last_try = now
            self.init_i2c()
            return

        try:
            # Read all sensor data
            accel_x, accel_y, accel_z = self.sensor.acceleration
            gyro_x, gyro_y, gyro_z = self.sensor.gyro
            quat_x, quat_y, quat_z, quat_w = self.sensor.quaternion

            # Fill the complete IMU message
            self.imu_msg_.linear_acceleration.x = accel_x
            self.imu_msg_.linear_acceleration.y = accel_y
            self.imu_msg_.linear_acceleration.z = accel_z

            self.imu_msg_.angular_velocity.x = gyro_x
            self.imu_msg_.angular_velocity.y = gyro_y
            self.imu_msg_.angular_velocity.z = gyro_z

            self.imu_msg_.orientation.x = quat_x
            self.imu_msg_.orientation.y = quat_y
            self.imu_msg_.orientation.z = quat_z
            self.imu_msg_.orientation.w = quat_w

            # Add covariance data (important for EKF)
            # These are example values and may need tuning
            self.imu_msg_.orientation_covariance[0] = 0.01
            self.imu_msg_.orientation_covariance[4] = 0.01
            self.imu_msg_.orientation_covariance[8] = 0.01
            self.imu_msg_.angular_velocity_covariance[0] = 0.05
            self.imu_msg_.angular_velocity_covariance[4] = 0.05
            self.imu_msg_.angular_velocity_covariance[8] = 0.05
            self.imu_msg_.linear_acceleration_covariance[0] = 0.1
            self.imu_msg_.linear_acceleration_covariance[4] = 0.1
            self.imu_msg_.linear_acceleration_covariance[8] = 0.1
            
            # Timestamp and publish
            self.imu_msg_.header.stamp = self.get_clock().now().to_msg()
            self.imu_pub_.publish(self.imu_msg_)

        except Exception as e:
            self.get_logger().warn(f"I2C read failed: {str(e)}")
            self.is_connected_ = False
            self.sensor = None

def main():
    rclpy.init()
    driver = BNO085_Driver()
    rclpy.spin(driver)
    # Cleanly destroy the node and shut down rclpy
    driver.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()