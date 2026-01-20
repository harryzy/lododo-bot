#!/usr/bin/env python
# encoding: utf-8

# public lib
import sys
import math
import random
import threading
import yaml
import os
from ament_index_python.packages import get_package_share_directory

from YbImuLib import YbImuSerial

# ros lib
import rclpy
from rclpy.node import Node
from rclpy.clock import Clock
from sensor_msgs.msg import Imu, MagneticField
from std_msgs.msg import Float32MultiArray


class ybimu_driver(Node):
    def __init__(self, name):
        super().__init__(name)
        self.robot = None


    def init_topic(self):
        # 从hardware_config.yaml读取IMU串口配置 / Read IMU port config from hardware_config.yaml
        try:
            config_path = os.path.join(
                get_package_share_directory('bot_hardware'),
                'config', 'hardware_config.yaml'
            )
            with open(config_path, 'r') as f:
                config = yaml.safe_load(f)
            
            imu_port = config['serial']['imu_port']
            imu_baudrate = config['serial']['imu_baudrate']
            self.get_logger().info(f"Loading config: port={imu_port}, baudrate={imu_baudrate}")
            
            port_list = [imu_port, "/dev/ttyUSB0", "/dev/ttyUSB1", "/dev/ttyUSB2"]
        except Exception as e:
            self.get_logger().warn(f"Failed to load config: {e}, using default ports")
            port_list = ["/dev/myimu", "/dev/ttyUSB0", "/dev/ttyUSB1", "/dev/ttyUSB2"]
        
        for port in port_list:
            try:
                self.robot = YbImuSerial(port)
                self.get_logger().info("Open Ybimu Port OK:%s" % port)
                break
            except Exception as e:
                self.get_logger().debug(f"Failed to open {port}: {e}")
                pass
        if self.robot is None:
            self.get_logger().error("---------Fail To Open Ybimu Serial------------")
            self.get_logger().error(f"Tried ports: {port_list}")
            return
        self.robot.create_receive_threading()

        # create publisher
        self.imuPublisher = self.create_publisher(Imu, "imu/data_raw", 100)
        self.magPublisher = self.create_publisher(MagneticField, "imu/mag", 100)
        self.baroPublisher = self.create_publisher(Float32MultiArray, "baro", 100)
        self.eulerPublisher = self.create_publisher(Float32MultiArray, "euler", 100)

        # create timer
        self.timer = self.create_timer(0.1, self.pub_data)


    # pub data
    def pub_data(self):
        if self.robot is None:
            return
        time_stamp = Clock().now()
        imu = Imu()
        mag = MagneticField()
        baro = Float32MultiArray()
        euler = Float32MultiArray()

        [ax, ay, az] = self.robot.get_accelerometer_data()
        # self.get_logger().info("ax = {}, ay = {}, az = {} ".format(ax,ay,az))
        [gx, gy, gz] = self.robot.get_gyroscope_data()
        # self.get_logger().info("gx = {}, gy = {}, gz = {} ".format(gx,gy,gz))
        [mx, my, mz] = self.robot.get_magnetometer_data()
        # self.get_logger().info("mx = {}, my = {}, mz = {} ".format(mx,my,mz))
        [q0, q1, q2, q3] = self.robot.get_imu_quaternion_data()
        [height, temperature, pressure, pressure_contrast] = self.robot.get_baro_data()
        [roll, pitch, yaw] = self.robot.get_imu_attitude_data(True)

        # 发布陀螺仪的数据
        # Publish gyroscope data
        imu.header.stamp = time_stamp.to_msg()
        imu.header.frame_id = "imu_link"
        imu.linear_acceleration.x = ax * 1.0
        imu.linear_acceleration.y = ay * 1.0
        imu.linear_acceleration.z = az * 1.0
        imu.angular_velocity.x = gx * 1.0
        imu.angular_velocity.y = gy * 1.0
        imu.angular_velocity.z = gz * 1.0
        imu.orientation.w = q0
        imu.orientation.x = q1
        imu.orientation.y = q2
        imu.orientation.z = q3

        mag.header.stamp = time_stamp.to_msg()
        mag.header.frame_id = "imu_link"
        mag.magnetic_field.x = mx * 1.0
        mag.magnetic_field.y = -my * 1.0
        mag.magnetic_field.z = mz * 1.0

        baro.data = [height, temperature, pressure, pressure_contrast]
        euler.data = [roll, pitch, yaw]

        self.imuPublisher.publish(imu)
        self.magPublisher.publish(mag)
        self.baroPublisher.publish(baro)
        self.eulerPublisher.publish(euler)

    def ready(self):
        if self.robot is None:
            return False
        else:
            return True



def main(args=None):
    rclpy.init(args=args)
    node = ybimu_driver("ybimu_node")
    node.init_topic()
    if not node.ready():
        return
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()


if __name__ == "__main__":
    main()
