#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu
import time
import board
import busio
import adafruit_bno08x
from adafruit_bno08x.i2c import BNO08X_I2C

# I2C address of BNO085
DEVICE_ADDRESS = 0x4a

class BNO085_Driver(Node):

    def __init__(self):
        super().__init__("bno085_driver")
        
        # I2C and sensor state
        self.sensor = None
        self.is_connected_ = False
        self.last_try = 0.0

        # ROS 2 IMU publisher
        self.imu_pub_ = self.create_publisher(Imu, "/imu/out", qos_profile=qos_profile_sensor_data)
        self.imu_msg_ = Imu()
        self.imu_msg_.header.frame_id = "base_footprint"

        # Timer frequency (20 Hz recommended for Pi)
        self.frequency_ = 0.05
        self.timer_ = self.create_timer(self.frequency_, self.timerCallback)

        self.get_logger().info("BNO085 Driver Node Initialized")

    def init_i2c(self):
        try:
            self.get_logger().info("Attempting to initialize BNO085 sensor...")
            i2c = busio.I2C(board.SCL, board.SDA)
            self.sensor = BNO08X_I2C(i2c, address=DEVICE_ADDRESS)

            # Enable only accelerometer and gyro (orientation skipped)
            self.sensor.enable_feature(adafruit_bno08x.BNO_REPORT_ACCELEROMETER)
            self.sensor.enable_feature(adafruit_bno08x.BNO_REPORT_GYROSCOPE)

            self.is_connected_ = True
            self.get_logger().info("BNO085 sensor connected successfully.")

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
            # Read accelerometer (m/s²) and gyro (rad/s)
            accel_x, accel_y, accel_z = self.sensor.acceleration
            gyro_x, gyro_y, gyro_z = self.sensor.gyro

            # Fill IMU message
            self.imu_msg_.linear_acceleration.x = accel_x
            self.imu_msg_.linear_acceleration.y = accel_y
            self.imu_msg_.linear_acceleration.z = accel_z

            self.imu_msg_.angular_velocity.x = gyro_x
            self.imu_msg_.angular_velocity.y = gyro_y
            self.imu_msg_.angular_velocity.z = gyro_z

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
    driver.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
