#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
IMU坐标系验证工具 / IMU Coordinate System Validation Tool

功能 / Features:
1. 采集静止状态下的IMU数据（100个样本） / Collect IMU data when robot is stationary (100 samples)
2. 计算重力向量的均值和标准差 / Calculate mean and std deviation of gravity vector
3. 验证坐标系转换是否正确 / Verify coordinate transformation is correct
4. 判定标准 / Criteria:
   - ✅ GOOD: 标准差 < 0.2 m/s²
   - ⚠️  WARN: 标准差 < 0.5 m/s²
   - ❌ FAIL: 标准差 ≥ 0.5 m/s²

使用方法 / Usage:
    将机器人放置在水平面上，保持静止 / Place robot on level surface, keep stationary
    ros2 run bot_hardware test_imu_coordinate
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import numpy as np
import sys
from collections import deque


class IMUCoordinateTest(Node):
    """IMU坐标系验证节点 / IMU coordinate verification node"""
    
    def __init__(self):
        super().__init__('test_imu_coordinate')
        
        # 配置参数 / Configuration
        self.sample_count = 100  # 采样数量
        self.gravity_expected = 9.81  # 预期重力加速度 m/s²
        
        # 数据存储 / Data storage
        self.samples = deque(maxlen=self.sample_count)
        self.collecting = True
        
        # 订阅滤波后的IMU数据 / Subscribe to filtered IMU data
        self.subscription = self.create_subscription(
            Imu,
            '/imu/data',
            self.imu_callback,
            10
        )
        
        self.get_logger().info('═══════════════════════════════════════')
        self.get_logger().info('IMU坐标系验证工具 / IMU Coordinate Test Tool')
        self.get_logger().info('═══════════════════════════════════════')
        self.get_logger().info('📋 请确保机器人放置在水平面上并保持静止')
        self.get_logger().info('   Please ensure robot is on level surface and stationary')
        self.get_logger().info(f'📊 将采集 {self.sample_count} 个样本...')
        self.get_logger().info(f'   Collecting {self.sample_count} samples...')
        self.get_logger().info('═══════════════════════════════════════')
        
    def imu_callback(self, msg: Imu):
        """IMU数据回调 / IMU data callback"""
        if not self.collecting:
            return
            
        # 提取线性加速度 / Extract linear acceleration
        accel = np.array([
            msg.linear_acceleration.x,
            msg.linear_acceleration.y,
            msg.linear_acceleration.z
        ])
        
        self.samples.append(accel)
        
        # 显示采集进度 / Show progress
        if len(self.samples) % 10 == 0:
            self.get_logger().info(f'📊 已采集 {len(self.samples)}/{self.sample_count} 样本')
        
        # 采集完成，分析数据 / Collection complete, analyze data
        if len(self.samples) >= self.sample_count:
            self.collecting = False
            self.analyze_data()
            
    def analyze_data(self):
        """分析采集的IMU数据 / Analyze collected IMU data"""
        self.get_logger().info('═══════════════════════════════════════')
        self.get_logger().info('📈 数据分析 / Data Analysis')
        self.get_logger().info('═══════════════════════════════════════')
        
        # 转换为numpy数组 / Convert to numpy array
        samples_array = np.array(self.samples)
        
        # 计算统计量 / Calculate statistics
        mean_accel = np.mean(samples_array, axis=0)
        std_accel = np.std(samples_array, axis=0)
        
        # 计算重力向量模长 / Calculate gravity vector magnitude
        gravity_magnitude = np.linalg.norm(mean_accel)
        gravity_error = abs(gravity_magnitude - self.gravity_expected)
        
        # 计算总体标准差 / Calculate overall std deviation
        overall_std = np.linalg.norm(std_accel)
        
        # 打印详细结果 / Print detailed results
        self.get_logger().info('')
        self.get_logger().info('📊 统计结果 / Statistics:')
        self.get_logger().info(f'   X轴 / X-axis: {mean_accel[0]:+7.4f} ± {std_accel[0]:.4f} m/s²')
        self.get_logger().info(f'   Y轴 / Y-axis: {mean_accel[1]:+7.4f} ± {std_accel[1]:.4f} m/s²')
        self.get_logger().info(f'   Z轴 / Z-axis: {mean_accel[2]:+7.4f} ± {std_accel[2]:.4f} m/s²')
        self.get_logger().info('')
        self.get_logger().info(f'📏 重力向量模长 / Gravity magnitude: {gravity_magnitude:.4f} m/s²')
        self.get_logger().info(f'   预期值 / Expected: {self.gravity_expected:.2f} m/s²')
        self.get_logger().info(f'   误差 / Error: {gravity_error:.4f} m/s² ({gravity_error/self.gravity_expected*100:.2f}%)')
        self.get_logger().info('')
        
        # 坐标系方向判定 / Coordinate system orientation check
        self.get_logger().info('🧭 坐标系方向验证 / Coordinate System Verification:')
        dominant_axis = np.argmax(np.abs(mean_accel))
        dominant_value = mean_accel[dominant_axis]
        axis_names = ['X', 'Y', 'Z']
        
        if dominant_axis == 2:  # Z轴
            if dominant_value < 0:  # 负Z方向
                self.get_logger().info('   ✅ 重力主要在-Z方向 (ENU坐标系正确)')
                self.get_logger().info('      Gravity primarily in -Z direction (ENU correct)')
                coord_system_ok = True
            else:
                self.get_logger().warn('   ⚠️  重力主要在+Z方向 (可能坐标系错误)')
                self.get_logger().warn('      Gravity primarily in +Z direction (possible coord error)')
                coord_system_ok = False
        else:
            self.get_logger().error(f'   ❌ 重力主要在{axis_names[dominant_axis]}方向 (坐标系错误!)')
            self.get_logger().error(f'      Gravity primarily in {axis_names[dominant_axis]} direction (coord system error!)')
            coord_system_ok = False
            
        self.get_logger().info('')
        
        # 噪声水平判定 / Noise level assessment
        self.get_logger().info('📉 噪声水平评估 / Noise Level Assessment:')
        self.get_logger().info(f'   总体标准差 / Overall std: {overall_std:.4f} m/s²')
        
        if overall_std < 0.2:
            noise_level = 'GOOD'
            noise_emoji = '✅'
            noise_color = 'info'
            noise_msg = '噪声水平优秀 / Excellent noise level'
        elif overall_std < 0.5:
            noise_level = 'WARN'
            noise_emoji = '⚠️'
            noise_color = 'warn'
            noise_msg = '噪声水平可接受 / Acceptable noise level'
        else:
            noise_level = 'FAIL'
            noise_emoji = '❌'
            noise_color = 'error'
            noise_msg = '噪声水平过高! / Noise level too high!'
            
        log_func = getattr(self.get_logger(), noise_color)
        log_func(f'   {noise_emoji} {noise_level}: {noise_msg}')
        self.get_logger().info('')
        
        # 最终判定 / Final verdict
        self.get_logger().info('═══════════════════════════════════════')
        self.get_logger().info('🎯 最终判定 / Final Verdict:')
        self.get_logger().info('═══════════════════════════════════════')
        
        if coord_system_ok and noise_level == 'GOOD':
            self.get_logger().info('✅ 测试通过 / TEST PASSED')
            self.get_logger().info('   坐标系转换正确，噪声水平优秀')
            self.get_logger().info('   Coordinate transform correct, noise level excellent')
            exit_code = 0
        elif coord_system_ok and noise_level == 'WARN':
            self.get_logger().warn('⚠️  测试警告 / TEST WARNING')
            self.get_logger().warn('   坐标系正确，但噪声略高，建议检查滤波参数')
            self.get_logger().warn('   Coordinate correct, but noise high, check filter params')
            exit_code = 1
        else:
            self.get_logger().error('❌ 测试失败 / TEST FAILED')
            if not coord_system_ok:
                self.get_logger().error('   坐标系转换错误！请检查NED→ENU转换矩阵')
                self.get_logger().error('   Coordinate transform error! Check NED→ENU matrix')
            if noise_level == 'FAIL':
                self.get_logger().error('   噪声过高！请检查滤波器配置或硬件连接')
                self.get_logger().error('   Noise too high! Check filter config or hardware')
            exit_code = 2
            
        self.get_logger().info('═══════════════════════════════════════')
        
        # 建议 / Recommendations
        if exit_code > 0:
            self.get_logger().info('')
            self.get_logger().info('💡 建议 / Recommendations:')
            if noise_level in ['WARN', 'FAIL']:
                self.get_logger().info('   1. 增大滤波窗口大小 (hardware_config.yaml: filter.median_window_size)')
                self.get_logger().info('      Increase filter window size')
                self.get_logger().info('   2. 检查IMU安装是否牢固')
                self.get_logger().info('      Check IMU mounting is secure')
                self.get_logger().info('   3. 执行IMU标定 (P5.1阶段)')
                self.get_logger().info('      Perform IMU calibration (P5.1 phase)')
            if not coord_system_ok:
                self.get_logger().info('   1. 检查imu_filter_node.py中的R_ned_to_enu矩阵')
                self.get_logger().info('      Check R_ned_to_enu matrix in imu_filter_node.py')
                self.get_logger().info('   2. 验证mounting_rotation配置 (hardware_config.yaml)')
                self.get_logger().info('      Verify mounting_rotation config')
        
        # 关闭节点 / Shutdown node
        rclpy.shutdown()
        sys.exit(exit_code)


def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = IMUCoordinateTest()
        rclpy.spin(node)
    except KeyboardInterrupt:
        print('\n⚠️  测试被用户中断 / Test interrupted by user')
    except Exception as e:
        print(f'\n❌ 测试异常 / Test error: {e}')
        sys.exit(3)
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
