#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
OmniKinematics - 全向轮运动学工具类
Omnidirectional Wheel Kinematics Utility

功能特性 / Features:
- 逆向运动学：机器人速度 → 轮子角速度 / Inverse kinematics: robot velocity → wheel velocities
- 正向运动学：轮子角速度 → 机器人速度 / Forward kinematics: wheel velocities → robot velocity
- 从配置读取雅可比矩阵参数 / Load Jacobian parameters from configuration

设计参考 / Design Reference:
- 主设计: HARDWARE_DEPLOYMENT_DESIGN.md §3.2.3 (行2272-2314)
- 参考实现: bot_control/omni_controller_node.py (行56-92)
- 测试标准: HARDWARE_DEPLOYMENT_DESIGN.md §6.1.2 kinematics_test.yaml (行5438-5487)

雅可比矩阵推导 / Jacobian Matrix Derivation:
- 三轮全向轮系统（120°均匀分布）/ Three-wheel omni system (120° evenly distributed)
- 轮子滚动方向角度：wheel1=90°, wheel2=30°, wheel3=150°
- 公式：w_i = (1/R) * [cos(θ_i)*vx + sin(θ_i)*vy + L_i*ω_z]
- J = [[cos(θ1)/R, sin(θ1)/R, L1/R],
       [cos(θ2)/R, sin(θ2)/R, L2/R],
       [cos(θ3)/R, sin(θ3)/R, L3/R]]
"""

import numpy as np
from typing import Tuple, List

class OmniKinematics:
    """全向轮运动学工具类 / Omnidirectional wheel kinematics utility"""
    
    def __init__(self, config: dict):
        """初始化运动学工具 / Initialize kinematics utility
        
        Args:
            config: 完整的hardware_config.yaml配置字典 / Complete hardware config dict
        
        从配置读取参数 / Read parameters from config:
        - kinematics.wheel_radius: 轮子半径 (m)
        - kinematics.wheel_base.L1/L2/L3: 轮子到机器人中心的距离 (m)
        - kinematics.jacobian: 预计算的雅可比矩阵（可选，用于验证）
        """
        kinematics_cfg = config['kinematics']
        
        # 读取物理参数 / Read physical parameters
        self.wheel_radius = kinematics_cfg['wheel_radius']  # R = 0.05 m
        
        wheel_base_cfg = kinematics_cfg['wheel_base_distances']
        self.L1 = wheel_base_cfg['L1']  # 后轮到中心距离 / Rear wheel to center
        self.L2 = wheel_base_cfg['L2']  # 右前轮到中心距离 / Right front wheel to center
        self.L3 = wheel_base_cfg['L3']  # 左前轮到中心距离 / Left front wheel to center
        
        # 轮子滚动方向角度（弧度）/ Wheel rolling direction angles (radians)
        # 参考 omni_controller_node.py:70-74
        self.theta1 = np.pi / 2      # wheel1: 90.0° (rear) - rolls along +Y
        self.theta2 = np.pi / 6      # wheel2: 30.0° (right front) - rolls 30° from +X
        self.theta3 = 5 * np.pi / 6  # wheel3: 150.0° (left front) - rolls 150° from +X
        
        # 计算雅可比矩阵 / Compute Jacobian matrix
        # J * [vx, vy, omega_z]^T = [w1, w2, w3]^T
        R = self.wheel_radius
        self.J = np.array([
            [np.cos(self.theta1) / R, np.sin(self.theta1) / R, self.L1 / R],
            [np.cos(self.theta2) / R, np.sin(self.theta2) / R, self.L2 / R],
            [np.cos(self.theta3) / R, np.sin(self.theta3) / R, self.L3 / R]
        ])
        
        # 计算雅可比伪逆矩阵用于正向运动学 / Compute Jacobian pseudo-inverse for forward kinematics
        # J_pinv * [w1, w2, w3]^T = [vx, vy, omega_z]^T
        self.J_pinv = np.linalg.pinv(self.J)
        
        # 验证配置中的雅可比矩阵（如果提供）/ Verify Jacobian in config (if provided)
        if 'jacobian' in kinematics_cfg:
            config_jacobian = np.array(kinematics_cfg['jacobian'])
            max_diff = np.max(np.abs(self.J - config_jacobian))
            if max_diff > 0.01:
                raise ValueError(
                    f'Computed Jacobian differs from config by {max_diff:.6f}!\n'
                    f'Computed:\n{self.J}\n'
                    f'Config:\n{config_jacobian}'
                )
    
    def inverse_kinematics(
        self, 
        vx: float, 
        vy: float, 
        omega_z: float
    ) -> Tuple[float, float, float]:
        """逆向运动学：机器人速度 → 轮子角速度 / Inverse kinematics: robot velocity → wheel velocities
        
        Args:
            vx: X方向线速度 (m/s) / Linear velocity in X (m/s)
            vy: Y方向线速度 (m/s) / Linear velocity in Y (m/s)
            omega_z: 绕Z轴角速度 (rad/s) / Angular velocity around Z (rad/s)
        
        Returns:
            (w1, w2, w3): 三个轮子的角速度 (rad/s) / Three wheel angular velocities (rad/s)
            - w1: 后轮 / Rear wheel
            - w2: 右前轮 / Right front wheel
            - w3: 左前轮 / Left front wheel
        
        算法 / Algorithm:
        - 使用雅可比矩阵：[w1, w2, w3]^T = J * [vx, vy, omega_z]^T
        - J已在__init__中预计算 / J is pre-computed in __init__
        """
        velocity_vector = np.array([vx, vy, omega_z])
        wheel_speeds = self.J @ velocity_vector
        
        return wheel_speeds[0], wheel_speeds[1], wheel_speeds[2]
    
    def forward_kinematics(
        self, 
        w1: float, 
        w2: float, 
        w3: float
    ) -> Tuple[float, float, float]:
        """正向运动学：轮子角速度 → 机器人速度 / Forward kinematics: wheel velocities → robot velocity
        
        Args:
            w1: 后轮角速度 (rad/s) / Rear wheel angular velocity (rad/s)
            w2: 右前轮角速度 (rad/s) / Right front wheel angular velocity (rad/s)
            w3: 左前轮角速度 (rad/s) / Left front wheel angular velocity (rad/s)
        
        Returns:
            (vx, vy, omega_z): 机器人速度 / Robot velocity
            - vx: X方向线速度 (m/s) / Linear velocity in X (m/s)
            - vy: Y方向线速度 (m/s) / Linear velocity in Y (m/s)
            - omega_z: 绕Z轴角速度 (rad/s) / Angular velocity around Z (rad/s)
        
        算法 / Algorithm:
        - 使用雅可比伪逆矩阵：[vx, vy, omega_z]^T = J_pinv * [w1, w2, w3]^T
        - J_pinv已在__init__中预计算 / J_pinv is pre-computed in __init__
        """
        wheel_speeds = np.array([w1, w2, w3])
        velocity = self.J_pinv @ wheel_speeds
        
        return velocity[0], velocity[1], velocity[2]
    
    def get_jacobian(self) -> np.ndarray:
        """获取雅可比矩阵 / Get Jacobian matrix
        
        Returns:
            3x3 NumPy数组 / 3x3 NumPy array
        """
        return self.J.copy()
    
    def get_jacobian_pinv(self) -> np.ndarray:
        """获取雅可比伪逆矩阵 / Get Jacobian pseudo-inverse matrix
        
        Returns:
            3x3 NumPy数组 / 3x3 NumPy array
        """
        return self.J_pinv.copy()
    
    def get_parameters(self) -> dict:
        """获取运动学参数 / Get kinematics parameters
        
        Returns:
            参数字典，包含轮子半径、轮基距离等 / Parameter dict with wheel radius, wheel base, etc.
        """
        return {
            'wheel_radius': self.wheel_radius,
            'L1': self.L1,
            'L2': self.L2,
            'L3': self.L3,
            'theta1_deg': np.degrees(self.theta1),
            'theta2_deg': np.degrees(self.theta2),
            'theta3_deg': np.degrees(self.theta3)
        }


def main():
    """测试函数 / Test function"""
    print("OmniKinematics - 全向轮运动学工具测试")
    print("=" * 50)
    
    # 模拟配置 / Mock configuration
    config = {
        'kinematics': {
            'wheel_radius': 0.05,
            'wheel_base': {
                'L1': 0.126377,  # 后轮 / Rear wheel
                'L2': 0.125897,  # 右前轮 / Right front wheel
                'L3': 0.125897   # 左前轮 / Left front wheel
            },
            # 可选：提供预计算的雅可比矩阵用于验证 / Optional: pre-computed Jacobian for verification
            'jacobian': [
                [0.0,       20.0,      2.52754],
                [17.32051,  10.0,      2.51794],
                [-17.32051, 10.0,      2.51794]
            ]
        }
    }
    
    # 创建运动学工具 / Create kinematics utility
    kinematics = OmniKinematics(config)
    
    print(f"\n参数 / Parameters:")
    params = kinematics.get_parameters()
    print(f"  轮子半径 / Wheel radius: {params['wheel_radius']} m")
    print(f"  轮基距离 / Wheel base: L1={params['L1']:.6f}, L2={params['L2']:.6f}, L3={params['L3']:.6f} m")
    print(f"  轮子角度 / Wheel angles: θ1={params['theta1_deg']:.1f}°, θ2={params['theta2_deg']:.1f}°, θ3={params['theta3_deg']:.1f}°")
    
    print(f"\n雅可比矩阵 / Jacobian Matrix:")
    print(kinematics.get_jacobian())
    
    print(f"\n雅可比伪逆矩阵 / Jacobian Pseudo-Inverse:")
    print(kinematics.get_jacobian_pinv())
    
    # 测试1: 逆向运动学 - 前进 / Test 1: Inverse kinematics - forward
    print(f"\n测试1: 逆向运动学 - 前进0.2m/s")
    print("-" * 50)
    vx, vy, omega_z = 0.2, 0.0, 0.0
    w1, w2, w3 = kinematics.inverse_kinematics(vx, vy, omega_z)
    print(f"  输入 / Input: vx={vx:.2f}, vy={vy:.2f}, ω={omega_z:.2f}")
    print(f"  输出 / Output: w1={w1:.2f}, w2={w2:.2f}, w3={w3:.2f} rad/s")
    
    # 测试2: 逆向运动学 - 侧向移动 / Test 2: Inverse kinematics - sideways
    print(f"\n测试2: 逆向运动学 - 侧向移动0.2m/s")
    print("-" * 50)
    vx, vy, omega_z = 0.0, 0.2, 0.0
    w1, w2, w3 = kinematics.inverse_kinematics(vx, vy, omega_z)
    print(f"  输入 / Input: vx={vx:.2f}, vy={vy:.2f}, ω={omega_z:.2f}")
    print(f"  输出 / Output: w1={w1:.2f}, w2={w2:.2f}, w3={w3:.2f} rad/s")
    
    # 测试3: 逆向运动学 - 原地旋转 / Test 3: Inverse kinematics - rotation
    print(f"\n测试3: 逆向运动学 - 原地旋转0.5rad/s")
    print("-" * 50)
    vx, vy, omega_z = 0.0, 0.0, 0.5
    w1, w2, w3 = kinematics.inverse_kinematics(vx, vy, omega_z)
    print(f"  输入 / Input: vx={vx:.2f}, vy={vy:.2f}, ω={omega_z:.2f}")
    print(f"  输出 / Output: w1={w1:.2f}, w2={w2:.2f}, w3={w3:.2f} rad/s")
    
    # 测试4: 正向运动学验证 / Test 4: Forward kinematics verification
    print(f"\n测试4: 正向运动学验证（往返）")
    print("-" * 50)
    vx_input, vy_input, omega_input = 0.2, 0.1, 0.3
    
    # 逆向 / Forward
    w1, w2, w3 = kinematics.inverse_kinematics(vx_input, vy_input, omega_input)
    
    # 正向 / Reverse
    vx_output, vy_output, omega_output = kinematics.forward_kinematics(w1, w2, w3)
    
    print(f"  输入速度 / Input velocity: vx={vx_input:.3f}, vy={vy_input:.3f}, ω={omega_input:.3f}")
    print(f"  轮子速度 / Wheel speeds: w1={w1:.3f}, w2={w2:.3f}, w3={w3:.3f}")
    print(f"  恢复速度 / Recovered velocity: vx={vx_output:.3f}, vy={vy_output:.3f}, ω={omega_output:.3f}")
    
    # 验证误差 / Verify error
    error_vx = abs(vx_output - vx_input)
    error_vy = abs(vy_output - vy_input)
    error_omega = abs(omega_output - omega_input)
    
    print(f"  误差 / Error: Δvx={error_vx:.6f}, Δvy={error_vy:.6f}, Δω={error_omega:.6f}")
    
    if error_vx < 1e-10 and error_vy < 1e-10 and error_omega < 1e-10:
        print(f"  ✅ 往返验证通过 / Round-trip verification PASSED")
    else:
        print(f"  ❌ 往返验证失败 / Round-trip verification FAILED")
    
    print(f"\n✅ 测试完成 / Test completed")


if __name__ == '__main__':
    main()
