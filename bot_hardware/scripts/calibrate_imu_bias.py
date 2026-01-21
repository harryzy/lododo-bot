#!/usr/bin/env python3
"""
IMU零偏标定工具 / IMU Bias Calibration Tool

使用方法 / Usage:
1. 将机器人放在平稳表面上，保持静止 / Place robot on stable surface, keep still
2. 运行此脚本: python3 calibrate_imu_bias.py
3. 等待60秒采样完成 / Wait 60 seconds for sampling
4. 脚本会打印标定值，手动更新到hardware_config.yaml / Script prints calibration values

需要机器人启动并发布/imu/data_raw话题 / Requires robot running and publishing /imu/data_raw topic
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import numpy as np
import sys


class IMUBiasCalibrator(Node):
    def __init__(self):
        super().__init__('imu_bias_calibrator')
        
        # 采样参数 / Sampling parameters
        self.sample_duration = 60.0  # 采样时长（秒）/ Sampling duration (seconds)
        self.samples = {
            'gyro_x': [],
            'gyro_y': [],
            'gyro_z': [],
            'accel_x': [],
            'accel_y': [],
            'accel_z': []
        }
        
        self.start_time = self.get_clock().now()
        self.subscription = self.create_subscription(
            Imu,
            '/imu/data_raw',
            self.imu_callback,
            10
        )
        
        self.get_logger().info('='*70)
        self.get_logger().info('IMU零偏标定工具启动 / IMU Bias Calibration Tool Started')
        self.get_logger().info('='*70)
        self.get_logger().info('请确保机器人静止不动，放置在平稳表面上')
        self.get_logger().info('Please ensure robot is stationary on stable surface')
        self.get_logger().info(f'将采集{self.sample_duration}秒数据... / Collecting {self.sample_duration}s data...')
        self.get_logger().info('')
        
    def imu_callback(self, msg):
        elapsed = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
        
        if elapsed > self.sample_duration:
            if len(self.samples['gyro_x']) == 0:
                self.get_logger().error('未采集到数据！ / No data collected!')
                rclpy.shutdown()
                return
            
            self.calculate_and_display_bias()
            rclpy.shutdown()
            return
        
        # 采集数据 / Collect data
        self.samples['gyro_x'].append(msg.angular_velocity.x)
        self.samples['gyro_y'].append(msg.angular_velocity.y)
        self.samples['gyro_z'].append(msg.angular_velocity.z)
        self.samples['accel_x'].append(msg.linear_acceleration.x)
        self.samples['accel_y'].append(msg.linear_acceleration.y)
        self.samples['accel_z'].append(msg.linear_acceleration.z)
        
        # 每10秒显示进度 / Show progress every 10s
        if int(elapsed) % 10 == 0 and int(elapsed) > 0 and len(self.samples['gyro_x']) % 500 == 0:
            self.get_logger().info(f'采样进度: {int(elapsed)}/{int(self.sample_duration)}s, '
                                   f'样本数: {len(self.samples["gyro_x"])}')
    
    def calculate_and_display_bias(self):
        """计算并显示零偏值 / Calculate and display bias values"""
        
        # 计算均值和标准差 / Calculate mean and std
        gyro_x_mean = np.mean(self.samples['gyro_x'])
        gyro_y_mean = np.mean(self.samples['gyro_y'])
        gyro_z_mean = np.mean(self.samples['gyro_z'])
        
        gyro_x_std = np.std(self.samples['gyro_x'])
        gyro_y_std = np.std(self.samples['gyro_y'])
        gyro_z_std = np.std(self.samples['gyro_z'])
        
        accel_x_mean = np.mean(self.samples['accel_x'])
        accel_y_mean = np.mean(self.samples['accel_y'])
        accel_z_mean = np.mean(self.samples['accel_z'])
        
        accel_x_std = np.std(self.samples['accel_x'])
        accel_y_std = np.std(self.samples['accel_y'])
        accel_z_std = np.std(self.samples['accel_z'])
        
        # 计算重力加速度偏移（z轴应该接近9.81m/s²） / Calculate gravity offset
        gravity = 9.81
        accel_z_bias = accel_z_mean - gravity
        
        self.get_logger().info('')
        self.get_logger().info('='*70)
        self.get_logger().info('标定完成！ / Calibration Complete!')
        self.get_logger().info('='*70)
        self.get_logger().info(f'采样数量 / Sample count: {len(self.samples["gyro_x"])}')
        self.get_logger().info('')
        
        self.get_logger().info('陀螺仪零偏 / Gyroscope Bias:')
        self.get_logger().info(f'  gyro_bias_x: {gyro_x_mean:.6f} rad/s (±{gyro_x_std:.6f}) = {np.rad2deg(gyro_x_mean):.3f}°/s')
        self.get_logger().info(f'  gyro_bias_y: {gyro_y_mean:.6f} rad/s (±{gyro_y_std:.6f}) = {np.rad2deg(gyro_y_mean):.3f}°/s')
        self.get_logger().info(f'  gyro_bias_z: {gyro_z_mean:.6f} rad/s (±{gyro_z_std:.6f}) = {np.rad2deg(gyro_z_mean):.3f}°/s  ← 主要偏差')
        self.get_logger().info('')
        
        self.get_logger().info('加速度计零偏 / Accelerometer Bias:')
        self.get_logger().info(f'  accel_bias_x: {accel_x_mean:.6f} m/s² (±{accel_x_std:.6f})')
        self.get_logger().info(f'  accel_bias_y: {accel_y_mean:.6f} m/s² (±{accel_y_std:.6f})')
        self.get_logger().info(f'  accel_bias_z: {accel_z_bias:.6f} m/s² (±{accel_z_std:.6f}) [重力补偿后 / After gravity]')
        self.get_logger().info(f'  实测重力值: {accel_z_mean:.4f} m/s² (理论9.81)')
        self.get_logger().info('')
        
        self.get_logger().info('='*70)
        self.get_logger().info('请将以下值更新到 hardware_config.yaml:')
        self.get_logger().info('Please update hardware_config.yaml with these values:')
        self.get_logger().info('='*70)
        print(f"""
imu:
  calibration:
    gyro_bias_x: {gyro_x_mean:.8f}
    gyro_bias_y: {gyro_y_mean:.8f}
    gyro_bias_z: {gyro_z_mean:.8f}
    accel_bias_x: {accel_x_mean:.8f}
    accel_bias_y: {accel_y_mean:.8f}
    accel_bias_z: {accel_z_bias:.8f}
""")
        
        # 警告检查 / Warning checks
        if abs(gyro_z_mean) > 0.05:  # 超过2.86°/s
            self.get_logger().warn(f'⚠️ Z轴陀螺仪零偏较大 ({np.rad2deg(gyro_z_mean):.2f}°/s)，建议检查IMU安装!')
        
        if abs(accel_z_mean - gravity) > 0.5:
            self.get_logger().warn(f'⚠️ Z轴加速度与重力偏差较大 ({accel_z_mean:.2f} vs 9.81)，检查IMU水平度!')


def main():
    rclpy.init()
    
    try:
        calibrator = IMUBiasCalibrator()
        rclpy.spin(calibrator)
    except KeyboardInterrupt:
        print('\n标定被中断 / Calibration interrupted')
    except Exception as e:
        print(f'错误 / Error: {e}')
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
