#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
IMU滤波和坐标转换节点 / IMU Filter and Coordinate Transformation Node

功能 / Functions:
1. 订阅/imu/data_raw，发布/imu/data
2. NED→ENU坐标系转换 
3. 应用mounting_rotation（IMU安装角度补偿）
4. 滑动平均滤波
5. 保留原始时间戳（<5ms延迟）

参考 / Reference:
- 设计文档 §3.5.2.2 imu_filter_node设计
- P3计划 P3.2任务
"""

import numpy as np
from collections import deque
import yaml
import os
from ament_index_python.packages import get_package_share_directory

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import Imu
from scipy.spatial.transform import Rotation as R


class MovingAverageFilter:
    """滑动平均滤波器 / Moving Average Filter"""
    
    def __init__(self, window_size=5):
        self.window = deque(maxlen=window_size)
    
    def update(self, value):
        """更新滤波器并返回滤波后的值 / Update filter and return filtered value"""
        self.window.append(np.array(value))
        if len(self.window) == 0:
            return np.array(value)
        return np.mean(self.window, axis=0)
    
    def reset(self):
        """重置滤波器 / Reset filter"""
        self.window.clear()


class IMUFilterNode(Node):
    """IMU滤波节点 / IMU Filter Node"""
    
    def __init__(self):
        super().__init__('imu_filter_node')
        
        # 加载配置 / Load configuration
        self.load_config()
        
        # 初始化坐标转换 / Initialize coordinate transformation
        self.init_coordinate_transform()
        
        # 初始化滤波器 / Initialize filters
        self.accel_filter = MovingAverageFilter(self.filter_window_size)
        self.gyro_filter = MovingAverageFilter(self.filter_window_size)
        
        # 创建QoS配置（BEST_EFFORT以匹配robot_localization默认QoS）
        # Create QoS profile (BEST_EFFORT to match robot_localization default)
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,  # 从RELIABLE改为BEST_EFFORT
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # 创建订阅和发布 / Create subscription and publisher
        self.subscription = self.create_subscription(
            Imu,
            '/imu/data_raw',
            self.imu_callback,
            10
        )
        
        self.publisher = self.create_publisher(
            Imu,
            '/imu/data',
            qos_profile  # 使用BEST_EFFORT QoS
        )
        
        self.get_logger().info('IMU Filter Node initialized')
        self.get_logger().info(f'  - Filter window size: {self.filter_window_size}')
        self.get_logger().info(f'  - Mounting rotation: roll={self.mounting_roll:.2f}, '
                              f'pitch={self.mounting_pitch:.2f}, yaw={self.mounting_yaw:.2f} rad')
        self.get_logger().info(f'  - Frame ID: {self.frame_id}')
    
    def load_config(self):
        """从hardware_config.yaml加载配置 / Load config from hardware_config.yaml"""
        try:
            config_path = os.path.join(
                get_package_share_directory('bot_hardware'),
                'config', 'hardware_config.yaml'
            )
            
            with open(config_path, 'r') as f:
                config = yaml.safe_load(f)
            
            # 读取IMU配置 / Read IMU configuration
            imu_config = config.get('imu', {})
            self.frame_id = imu_config.get('frame_id', 'imu_link')
            
            # 读取mounting_rotation / Read mounting rotation
            mounting_rotation = imu_config.get('mounting_rotation', {})
            self.mounting_roll = mounting_rotation.get('roll', 0.0)
            self.mounting_pitch = mounting_rotation.get('pitch', 0.0)
            self.mounting_yaw = mounting_rotation.get('yaw', 0.0)
            
            # 读取滤波参数 / Read filter parameters
            filter_config = imu_config.get('filter', {})
            self.filter_window_size = filter_config.get('median_window_size', 5)
            
            self.get_logger().info('Configuration loaded successfully')
            
        except Exception as e:
            self.get_logger().error(f'Failed to load config: {e}')
            # 使用默认值 / Use default values
            self.frame_id = 'imu_link'
            self.mounting_roll = 0.0
            self.mounting_pitch = 0.0
            self.mounting_yaw = 0.0
            self.filter_window_size = 5
            self.get_logger().warn('Using default configuration')
    
    def init_coordinate_transform(self):
        """初始化坐标转换矩阵 / Initialize coordinate transformation matrix"""
        # NED → ENU转换矩阵 / NED to ENU transformation matrix
        # X_enu = Y_ned, Y_enu = X_ned, Z_enu = -Z_ned
        self.R_ned_to_enu = np.array([
            [0,  1,  0],
            [1,  0,  0],
            [0,  0, -1]
        ])
        
        # Mounting rotation（IMU安装角度补偿）/ Mounting rotation compensation
        self.R_mount = R.from_euler('xyz', 
                                     [self.mounting_roll, self.mounting_pitch, self.mounting_yaw],
                                     degrees=False).as_matrix()
        
        # 组合变换矩阵 / Combined transformation matrix
        self.R_transform = self.R_mount @ self.R_ned_to_enu
    
    def transform_vector(self, vec):
        """转换向量（NED→ENU + mounting rotation）/ Transform vector (NED→ENU + mounting)"""
        return self.R_transform @ np.array(vec)
    
    def imu_callback(self, msg):
        """
        IMU数据回调函数 / IMU data callback
        
        关键设计：保留原始时间戳（方案B）
        处理延迟<5ms，符合robot_localization要求
        """
        try:
            # 创建输出消息 / Create output message
            filtered_msg = Imu()
            
            # ⭐ 保留原始时间戳 / Preserve original timestamp
            filtered_msg.header.stamp = msg.header.stamp
            filtered_msg.header.frame_id = 'base_link'  # 转换后使用base_link坐标系
            
            # 1. 坐标转换：线性加速度 / Coordinate transform: linear acceleration
            accel_raw = np.array([
                msg.linear_acceleration.x,
                msg.linear_acceleration.y,
                msg.linear_acceleration.z
            ])
            accel_transformed = self.transform_vector(accel_raw)
            
            # 2. 滤波：线性加速度 / Filter: linear acceleration
            accel_filtered = self.accel_filter.update(accel_transformed)
            
            filtered_msg.linear_acceleration.x = accel_filtered[0]
            filtered_msg.linear_acceleration.y = accel_filtered[1]
            filtered_msg.linear_acceleration.z = accel_filtered[2]
            
            # 3. 坐标转换：角速度 / Coordinate transform: angular velocity
            gyro_raw = np.array([
                msg.angular_velocity.x,
                msg.angular_velocity.y,
                msg.angular_velocity.z
            ])
            gyro_transformed = self.transform_vector(gyro_raw)
            
            # 4. 滤波：角速度 / Filter: angular velocity
            gyro_filtered = self.gyro_filter.update(gyro_transformed)
            
            filtered_msg.angular_velocity.x = gyro_filtered[0]
            filtered_msg.angular_velocity.y = gyro_filtered[1]
            filtered_msg.angular_velocity.z = gyro_filtered[2]
            
            # 5. 四元数：直接传递（已在ybimu_driver中计算）/ Quaternion: pass through
            filtered_msg.orientation = msg.orientation
            
            # 6. 协方差矩阵：从hardware_config.yaml读取 / Covariance: from config
            # TODO: 后续从配置文件读取 / Read from config later
            # 暂时使用保守估计值 / Conservative estimates for now
            filtered_msg.orientation_covariance = [0.01, 0.0, 0.0, 0.0, 0.01, 0.0, 0.0, 0.0, 0.01]
            filtered_msg.angular_velocity_covariance = [0.02, 0.0, 0.0, 0.0, 0.02, 0.0, 0.0, 0.0, 0.02]
            filtered_msg.linear_acceleration_covariance = [0.05, 0.0, 0.0, 0.0, 0.05, 0.0, 0.0, 0.0, 0.05]
            
            # 发布滤波后的数据 / Publish filtered data
            self.publisher.publish(filtered_msg)
            
        except Exception as e:
            self.get_logger().error(f'Error in IMU callback: {e}')


def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = IMUFilterNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'Error: {e}')
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
