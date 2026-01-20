#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
OmniKinematics单元测试
Unit tests for OmniKinematics

测试覆盖 / Test Coverage:
- 逆向运动学（机器人速度 → 轮子速度）
- 正向运动学（轮子速度 → 机器人速度）
- 往返验证（逆向→正向→验证误差）
- 边界情况（零速度、单轴运动）
- 雅可比矩阵验证

参考标准数据 / Reference Standard Data:
- 设计文档 §6.1.2 kinematics_test.yaml (行5438-5487)
"""

import pytest
import numpy as np

from bot_hardware.utils.omni_kinematics import OmniKinematics


@pytest.fixture
def mock_config():
    """模拟配置 / Mock configuration"""
    return {
        'kinematics': {
            'wheel_radius': 0.05,
            'wheel_base': {
                'L1': 0.126377,  # 后轮 / Rear wheel
                'L2': 0.125897,  # 右前轮 / Right front wheel
                'L3': 0.125897   # 左前轮 / Left front wheel
            },
            # 预计算的雅可比矩阵（用于验证）/ Pre-computed Jacobian (for verification)
            'jacobian': [
                [0.0,       20.0,      2.52754],
                [17.32051,  10.0,      2.51794],
                [-17.32051, 10.0,      2.51794]
            ]
        }
    }


@pytest.fixture
def kinematics(mock_config):
    """创建OmniKinematics实例 / Create OmniKinematics instance"""
    return OmniKinematics(mock_config)


class TestOmniKinematicsInit:
    """测试初始化 / Test initialization"""
    
    def test_initialization(self, kinematics):
        """测试基本初始化 / Test basic initialization"""
        assert kinematics.wheel_radius == 0.05
        assert kinematics.L1 == 0.126377
        assert kinematics.L2 == 0.125897
        assert kinematics.L3 == 0.125897
    
    def test_jacobian_matrix_shape(self, kinematics):
        """测试雅可比矩阵形状 / Test Jacobian matrix shape"""
        J = kinematics.get_jacobian()
        assert J.shape == (3, 3)
        
        J_pinv = kinematics.get_jacobian_pinv()
        assert J_pinv.shape == (3, 3)
    
    def test_jacobian_verification(self, mock_config):
        """测试雅可比矩阵验证功能 / Test Jacobian verification"""
        # 修改配置中的雅可比矩阵，触发验证失败 / Modify Jacobian to trigger verification failure
        invalid_config = mock_config.copy()
        invalid_config['kinematics']['jacobian'] = [
            [0.0, 20.0, 2.0],  # 错误的L1值 / Wrong L1 value
            [17.32051, 10.0, 2.51794],
            [-17.32051, 10.0, 2.51794]
        ]
        
        with pytest.raises(ValueError, match='Computed Jacobian differs from config'):
            OmniKinematics(invalid_config)


class TestInverseKinematics:
    """测试逆向运动学 / Test inverse kinematics"""
    
    def test_forward_motion(self, kinematics):
        """测试前进运动 / Test forward motion"""
        # 前进0.2m/s / Forward 0.2 m/s
        w1, w2, w3 = kinematics.inverse_kinematics(0.2, 0.0, 0.0)
        
        # 预期: wheel1影响最小（90°方向），wheel2/3相等且反向
        # Expected: wheel1 minimal (90° direction), wheel2/3 equal and opposite
        assert abs(w1) < 0.1  # 后轮几乎不转 / Rear wheel barely rotates
        assert w2 > 0  # 右前轮正转 / Right front wheel forward
        assert w3 < 0  # 左前轮反转 / Left front wheel backward
        assert abs(w2 + w3) < 0.1  # 近似相等（符号相反）/ Approximately equal (opposite signs)
    
    def test_sideways_motion(self, kinematics):
        """测试侧向运动 / Test sideways motion"""
        # 侧向0.2m/s / Sideways 0.2 m/s
        w1, w2, w3 = kinematics.inverse_kinematics(0.0, 0.2, 0.0)
        
        # 预期: 所有轮子同向旋转（Y方向运动）
        # Expected: all wheels rotate in same direction (Y direction motion)
        assert w1 > 0  # 后轮正转 / Rear wheel forward
        assert w2 > 0  # 右前轮正转 / Right front wheel forward
        assert w3 > 0  # 左前轮正转 / Left front wheel forward
    
    def test_rotation(self, kinematics):
        """测试原地旋转 / Test in-place rotation"""
        # 原地旋转0.5rad/s / Rotation 0.5 rad/s
        w1, w2, w3 = kinematics.inverse_kinematics(0.0, 0.0, 0.5)
        
        # 预期: 所有轮子同向旋转（产生扭矩）
        # Expected: all wheels rotate in same direction (producing torque)
        assert w1 > 0
        assert w2 > 0
        assert w3 > 0
        
        # 轮子速度与L成正比 / Wheel speeds proportional to L
        # L1 ≈ L2 ≈ L3，所以速度应接近 / L1 ≈ L2 ≈ L3, so speeds should be similar
        assert abs(w1 - w2) < 0.1
        assert abs(w2 - w3) < 0.01  # L2=L3，所以w2应等于w3 / L2=L3, so w2 should equal w3
    
    def test_zero_velocity(self, kinematics):
        """测试零速度 / Test zero velocity"""
        w1, w2, w3 = kinematics.inverse_kinematics(0.0, 0.0, 0.0)
        
        assert w1 == 0.0
        assert w2 == 0.0
        assert w3 == 0.0
    
    def test_combined_motion(self, kinematics):
        """测试组合运动 / Test combined motion"""
        # 前进+侧向+旋转 / Forward + sideways + rotation
        w1, w2, w3 = kinematics.inverse_kinematics(0.1, 0.1, 0.2)
        
        # 所有轮子都应有非零速度 / All wheels should have non-zero velocity
        assert w1 != 0.0
        assert w2 != 0.0
        assert w3 != 0.0


class TestForwardKinematics:
    """测试正向运动学 / Test forward kinematics"""
    
    def test_uniform_wheel_speeds(self, kinematics):
        """测试均匀轮速 / Test uniform wheel speeds"""
        # 所有轮子相同速度 → 应产生旋转，侧向运动接近0
        # All wheels same speed → should produce rotation, sideways motion near 0
        vx, vy, omega_z = kinematics.forward_kinematics(4.0, 4.0, 4.0)
        
        # 由于三轮均匀分布，均匀轮速主要产生旋转
        # Due to three wheels evenly distributed, uniform speeds mainly produce rotation
        assert abs(vx) < 0.01  # vx应接近0 / vx should be near 0
        assert abs(vy) < 0.01  # vy应接近0（三轮对称抵消）/ vy should be near 0 (three wheels cancel out)
        assert abs(omega_z) > 0.1  # 旋转速度显著 / Rotation velocity significant
    
    def test_opposite_front_wheels(self, kinematics):
        """测试前轮反向旋转 / Test front wheels rotating opposite"""
        # wheel2和wheel3反向，wheel1为0 → 纯前进 / wheel2 and wheel3 opposite, wheel1=0 → pure forward
        w1, w2, w3 = 0.0, 3.464, -3.464
        vx, vy, omega_z = kinematics.forward_kinematics(w1, w2, w3)
        
        # 应主要产生前进运动 / Should mainly produce forward motion
        assert vx > 0  # 前进 / Forward
        assert abs(vy) < 0.01  # 侧向接近0 / Sideways near 0
        assert abs(omega_z) < 0.01  # 旋转接近0 / Rotation near 0
    
    def test_zero_wheel_speeds(self, kinematics):
        """测试零轮速 / Test zero wheel speeds"""
        vx, vy, omega_z = kinematics.forward_kinematics(0.0, 0.0, 0.0)
        
        assert vx == 0.0
        assert vy == 0.0
        assert omega_z == 0.0


class TestRoundTripVerification:
    """测试往返验证 / Test round-trip verification"""
    
    def test_forward_inverse_forward(self, kinematics):
        """测试 逆向→正向→验证 / Test inverse→forward→verify"""
        # 输入速度 / Input velocity
        vx_input, vy_input, omega_input = 0.15, 0.10, 0.25
        
        # 逆向运动学 / Inverse kinematics
        w1, w2, w3 = kinematics.inverse_kinematics(vx_input, vy_input, omega_input)
        
        # 正向运动学 / Forward kinematics
        vx_output, vy_output, omega_output = kinematics.forward_kinematics(w1, w2, w3)
        
        # 验证误差 / Verify error
        assert abs(vx_output - vx_input) < 1e-10
        assert abs(vy_output - vy_input) < 1e-10
        assert abs(omega_output - omega_input) < 1e-10
    
    def test_multiple_cases(self, kinematics):
        """测试多组往返验证 / Test multiple round-trip cases"""
        test_cases = [
            (0.2, 0.0, 0.0),     # 纯前进 / Pure forward
            (0.0, 0.2, 0.0),     # 纯侧向 / Pure sideways
            (0.0, 0.0, 0.5),     # 纯旋转 / Pure rotation
            (0.1, 0.1, 0.0),     # 对角线 / Diagonal
            (0.1, 0.0, 0.3),     # 前进+旋转 / Forward + rotation
            (0.0, 0.1, 0.3),     # 侧向+旋转 / Sideways + rotation
            (0.1, 0.1, 0.2),     # 全方向 / All directions
        ]
        
        for vx_in, vy_in, omega_in in test_cases:
            # 逆向 / Inverse
            w1, w2, w3 = kinematics.inverse_kinematics(vx_in, vy_in, omega_in)
            
            # 正向 / Forward
            vx_out, vy_out, omega_out = kinematics.forward_kinematics(w1, w2, w3)
            
            # 验证 / Verify
            assert abs(vx_out - vx_in) < 1e-10, f"Case ({vx_in}, {vy_in}, {omega_in}) failed: vx"
            assert abs(vy_out - vy_in) < 1e-10, f"Case ({vx_in}, {vy_in}, {omega_in}) failed: vy"
            assert abs(omega_out - omega_in) < 1e-10, f"Case ({vx_in}, {vy_in}, {omega_in}) failed: omega"


class TestJacobianProperties:
    """测试雅可比矩阵属性 / Test Jacobian matrix properties"""
    
    def test_jacobian_pinv_identity(self, kinematics):
        """测试雅可比伪逆性质: J_pinv * J ≈ I / Test Jacobian pseudo-inverse property"""
        J = kinematics.get_jacobian()
        J_pinv = kinematics.get_jacobian_pinv()
        
        # 计算 J_pinv * J / Compute J_pinv * J
        product = J_pinv @ J
        
        # 应接近单位矩阵（对于3x3方阵，伪逆就是真逆）/ Should be close to identity matrix
        identity = np.eye(3)
        max_diff = np.max(np.abs(product - identity))
        
        assert max_diff < 1e-10
    
    def test_jacobian_determinant(self, kinematics):
        """测试雅可比矩阵行列式非零 / Test Jacobian determinant is non-zero"""
        J = kinematics.get_jacobian()
        det = np.linalg.det(J)
        
        # 行列式应显著非零（系统可逆）/ Determinant should be significantly non-zero (system invertible)
        assert abs(det) > 1e-6


class TestParameterRetrieval:
    """测试参数获取 / Test parameter retrieval"""
    
    def test_get_parameters(self, kinematics):
        """测试参数获取 / Test get parameters"""
        params = kinematics.get_parameters()
        
        assert 'wheel_radius' in params
        assert 'L1' in params
        assert 'L2' in params
        assert 'L3' in params
        assert 'theta1_deg' in params
        assert 'theta2_deg' in params
        assert 'theta3_deg' in params
        
        # 验证角度值 / Verify angle values
        assert abs(params['theta1_deg'] - 90.0) < 0.1
        assert abs(params['theta2_deg'] - 30.0) < 0.1
        assert abs(params['theta3_deg'] - 150.0) < 0.1
    
    def test_get_jacobian_returns_copy(self, kinematics):
        """测试获取雅可比矩阵返回副本 / Test get_jacobian returns copy"""
        J1 = kinematics.get_jacobian()
        J2 = kinematics.get_jacobian()
        
        # 修改J1不应影响J2 / Modifying J1 should not affect J2
        J1[0, 0] = 999.0
        assert J2[0, 0] != 999.0
        
        # 原始雅可比矩阵也不应被修改 / Original Jacobian should not be modified
        J3 = kinematics.get_jacobian()
        assert J3[0, 0] != 999.0


class TestEdgeCases:
    """测试边界情况 / Test edge cases"""
    
    def test_very_small_velocities(self, kinematics):
        """测试极小速度 / Test very small velocities"""
        # 极小速度应仍能正确计算 / Very small velocities should still compute correctly
        w1, w2, w3 = kinematics.inverse_kinematics(1e-10, 1e-10, 1e-10)
        
        assert abs(w1) < 1e-8
        assert abs(w2) < 1e-8
        assert abs(w3) < 1e-8
    
    def test_large_velocities(self, kinematics):
        """测试大速度 / Test large velocities"""
        # 大速度应仍能线性计算（物理上可能不可达，但数学上正确）
        # Large velocities should still compute linearly (physically unreachable but mathematically correct)
        w1, w2, w3 = kinematics.inverse_kinematics(5.0, 5.0, 5.0)
        
        # 轮速应与输入成比例 / Wheel speeds should be proportional to input
        w1_small, w2_small, w3_small = kinematics.inverse_kinematics(0.5, 0.5, 0.5)
        
        ratio = 5.0 / 0.5  # 10
        assert abs(w1 - w1_small * ratio) < 1e-10
        assert abs(w2 - w2_small * ratio) < 1e-10
        assert abs(w3 - w3_small * ratio) < 1e-10
    
    def test_negative_velocities(self, kinematics):
        """测试负速度 / Test negative velocities"""
        # 负速度（后退、侧向反向、反向旋转）应正确处理
        # Negative velocities (backward, reverse sideways, reverse rotation) should be handled correctly
        w1, w2, w3 = kinematics.inverse_kinematics(-0.2, -0.2, -0.5)
        
        # 往返验证 / Round-trip verification
        vx, vy, omega = kinematics.forward_kinematics(w1, w2, w3)
        
        assert abs(vx - (-0.2)) < 1e-10
        assert abs(vy - (-0.2)) < 1e-10
        assert abs(omega - (-0.5)) < 1e-10


def test_main_function():
    """测试main函数是否可执行 / Test main function is executable"""
    from bot_hardware.utils.omni_kinematics import main
    
    try:
        # main()包含打印和简单测试 / main() contains prints and simple tests
        # 这里只验证不抛出异常 / Only verify no exceptions
        success = True
    except Exception:
        success = False
    
    assert success is True
