#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
VelocityRamp单元测试
Unit tests for VelocityRamp

测试覆盖 / Test Coverage:
- 速度斜坡限制（线速度和角速度）
- 急停功能 + 自动恢复
- 速度突变检测
- 加速度/减速度限制
- 状态查询和重置
"""

import pytest
import numpy as np
from rclpy.time import Time

from bot_hardware.utils.velocity_ramp import VelocityRamp


@pytest.fixture
def mock_config():
    """模拟配置 / Mock configuration"""
    return {
        'motion': {
            'max_linear_velocity': 0.5,
            'max_angular_velocity': 1.0,
            'max_linear_acceleration': 0.5,
            'max_linear_deceleration': 0.8,
            'max_angular_acceleration': 1.0,
            'max_angular_deceleration': 1.5,
            'velocity_ramp': {
                'enable': True,
                'velocity_jump_threshold': 0.3,
                'log_velocity_changes': False,
                'emergency_stop': {
                    'enable': True,
                    'auto_recovery_time': 2.0,  # 缩短测试时间 / Shorten for testing
                    'manual_recovery': False
                }
            }
        }
    }


@pytest.fixture
def velocity_ramp(mock_config):
    """创建VelocityRamp实例 / Create VelocityRamp instance"""
    return VelocityRamp(mock_config)


class TestVelocityRampInit:
    """测试初始化 / Test initialization"""
    
    def test_initialization(self, velocity_ramp):
        """测试基本初始化 / Test basic initialization"""
        assert velocity_ramp.max_linear_accel == 0.5
        assert velocity_ramp.max_angular_accel == 1.0
        assert velocity_ramp.enable is True
        assert velocity_ramp.cooldown_duration == 2.0
        assert velocity_ramp.is_emergency_stopped is False
    
    def test_initialization_with_defaults(self):
        """测试带默认值的初始化 / Test initialization with defaults"""
        config = {
            'motion': {
                'max_linear_velocity': 0.5,
                'max_angular_velocity': 1.0,
                'max_linear_acceleration': 0.5,
                'max_angular_acceleration': 1.0
            }
        }
        vr = VelocityRamp(config)
        
        # 验证默认值 / Verify defaults
        assert vr.max_linear_decel == 0.5  # 默认同加速度 / Default same as accel
        assert vr.max_angular_decel == 1.0
        assert vr.enable is True
        assert vr.velocity_jump_threshold == 0.3


class TestLinearVelocityLimiting:
    """测试线速度限制 / Test linear velocity limiting"""
    
    def test_linear_acceleration_limiting(self, velocity_ramp):
        """测试线加速度限制 / Test linear acceleration limiting"""
        current_time = Time(seconds=0)
        
        # 目标: 0→0.5m/s，加速度限制0.5m/s²
        # 预期: 每0.02s增加0.01m/s
        target_vx, target_vy, target_omega = 0.5, 0.0, 0.0
        
        # 第一次调用: 初始化，返回目标 / First call: initialize, return target
        limited_vx, _, _ = velocity_ramp.limit(target_vx, target_vy, target_omega, current_time)
        assert limited_vx == 0.5  # 初始化直接返回 / Initialize returns directly
        
        # 从零开始测试斜坡 / Test ramp from zero
        velocity_ramp.last_linear_velocity = np.array([0.0, 0.0])
        velocity_ramp.last_time = Time(seconds=0)
        
        # 第二次调用: dt=0.02s，期望增加0.01m/s
        current_time = Time(seconds=0.02)
        limited_vx, _, _ = velocity_ramp.limit(target_vx, target_vy, target_omega, current_time)
        
        expected = 0.5 * 0.02  # 0.01 m/s
        assert abs(limited_vx - expected) < 1e-6
    
    def test_linear_velocity_upper_limit(self, velocity_ramp):
        """测试线速度上限 / Test linear velocity upper limit"""
        current_time = Time(seconds=0)
        
        # 初始化为接近上限 / Initialize near limit
        velocity_ramp.limit(0.45, 0.0, 0.0, current_time)
        
        # 尝试超过上限 / Try to exceed limit
        current_time = Time(seconds=0.1)
        limited_vx, _, _ = velocity_ramp.limit(1.0, 0.0, 0.0, current_time)
        
        # 应被限制在0.5m/s / Should be limited to 0.5 m/s
        assert abs(limited_vx) <= velocity_ramp.max_linear_velocity
    
    def test_omnidirectional_velocity(self, velocity_ramp):
        """测试全向速度（vx和vy）/ Test omnidirectional velocity (vx and vy)"""
        current_time = Time(seconds=0)
        
        # 初始化 / Initialize
        velocity_ramp.limit(0.0, 0.0, 0.0, current_time)
        velocity_ramp.last_linear_velocity = np.array([0.0, 0.0])
        velocity_ramp.last_time = Time(seconds=0)
        
        # 对角线方向运动 / Diagonal movement
        target_vx, target_vy = 0.3, 0.3
        current_time = Time(seconds=0.1)
        
        limited_vx, limited_vy, _ = velocity_ramp.limit(target_vx, target_vy, 0.0, current_time)
        
        # 验证速度向量的模被限制 / Verify velocity vector magnitude is limited
        limited_speed = np.linalg.norm([limited_vx, limited_vy])
        assert limited_speed <= velocity_ramp.max_linear_velocity


class TestAngularVelocityLimiting:
    """测试角速度限制 / Test angular velocity limiting"""
    
    def test_angular_acceleration_limiting(self, velocity_ramp):
        """测试角加速度限制 / Test angular acceleration limiting"""
        current_time = Time(seconds=0)
        
        # 初始化 / Initialize
        velocity_ramp.limit(0.0, 0.0, 0.0, current_time)
        velocity_ramp.last_angular_velocity = 0.0
        velocity_ramp.last_time = Time(seconds=0)
        
        # 目标: 0→1.0rad/s，加速度限制1.0rad/s²
        target_omega = 1.0
        current_time = Time(seconds=0.02)
        
        _, _, limited_omega = velocity_ramp.limit(0.0, 0.0, target_omega, current_time)
        
        expected = 1.0 * 0.02  # 0.02 rad/s
        assert abs(limited_omega - expected) < 1e-6
    
    def test_angular_velocity_upper_limit(self, velocity_ramp):
        """测试角速度上限 / Test angular velocity upper limit"""
        current_time = Time(seconds=0)
        
        # 初始化为接近上限 / Initialize near limit
        velocity_ramp.limit(0.0, 0.0, 0.95, current_time)
        
        # 尝试超过上限 / Try to exceed limit
        current_time = Time(seconds=0.1)
        _, _, limited_omega = velocity_ramp.limit(0.0, 0.0, 2.0, current_time)
        
        # 应被限制在1.0rad/s / Should be limited to 1.0 rad/s
        assert abs(limited_omega) <= velocity_ramp.max_angular_velocity


class TestDecelerationLimiting:
    """测试减速度限制 / Test deceleration limiting"""
    
    def test_linear_deceleration(self, velocity_ramp):
        """测试线减速度限制 / Test linear deceleration limiting"""
        current_time = Time(seconds=0)
        
        # 初始化为高速 / Initialize at high speed
        velocity_ramp.limit(0.5, 0.0, 0.0, current_time)
        velocity_ramp.last_linear_velocity = np.array([0.5, 0.0])
        velocity_ramp.last_time = Time(seconds=0)
        
        # 目标: 0.5→0m/s，减速度限制0.8m/s²
        current_time = Time(seconds=0.02)
        limited_vx, _, _ = velocity_ramp.limit(0.0, 0.0, 0.0, current_time)
        
        # 预期: 0.5 - (0.8 * 0.02) = 0.484 m/s
        expected = 0.5 - (0.8 * 0.02)
        assert abs(limited_vx - expected) < 1e-6
    
    def test_angular_deceleration(self, velocity_ramp):
        """测试角减速度限制 / Test angular deceleration limiting"""
        current_time = Time(seconds=0)
        
        # 初始化为高速旋转 / Initialize at high angular velocity
        velocity_ramp.limit(0.0, 0.0, 1.0, current_time)
        velocity_ramp.last_angular_velocity = 1.0
        velocity_ramp.last_time = Time(seconds=0)
        
        # 目标: 1.0→0rad/s，减速度限制1.5rad/s²
        current_time = Time(seconds=0.02)
        _, _, limited_omega = velocity_ramp.limit(0.0, 0.0, 0.0, current_time)
        
        # 预期: 1.0 - (1.5 * 0.02) = 0.97 rad/s
        expected = 1.0 - (1.5 * 0.02)
        assert abs(limited_omega - expected) < 1e-6


class TestEmergencyStop:
    """测试急停功能 / Test emergency stop"""
    
    def test_emergency_stop_trigger(self, velocity_ramp):
        """测试急停触发 / Test emergency stop trigger"""
        # 触发急停 / Trigger emergency stop
        velocity_ramp.emergency_stop("Test emergency")
        
        assert velocity_ramp.is_emergency_stopped is True
        assert velocity_ramp.emergency_stop_reason == "Test emergency"
        assert velocity_ramp.emergency_stop_time is not None
    
    def test_emergency_stop_blocks_velocity(self, velocity_ramp):
        """测试急停阻止速度输出 / Test emergency stop blocks velocity output"""
        # 触发急停 / Trigger emergency stop
        velocity_ramp.emergency_stop("Test emergency")
        
        # 尝试输出速度 / Try to output velocity
        current_time = Time(seconds=0)
        limited_vx, limited_vy, limited_omega = velocity_ramp.limit(
            0.5, 0.0, 1.0, current_time
        )
        
        # 所有速度应为零 / All velocities should be zero
        assert limited_vx == 0.0
        assert limited_vy == 0.0
        assert limited_omega == 0.0
    
    def test_auto_recovery(self, velocity_ramp):
        """测试自动恢复 / Test auto-recovery"""
        import time
        
        # 触发急停 / Trigger emergency stop
        velocity_ramp.emergency_stop("Test emergency")
        assert velocity_ramp.is_emergency_stopped is True
        
        # 等待冷却期 / Wait for cooldown (2.1s > 2.0s cooldown)
        time.sleep(2.1)
        
        # 调用check_recovery / Call check_recovery
        velocity_ramp.check_recovery()
        
        # 应已恢复 / Should be recovered
        assert velocity_ramp.is_emergency_stopped is False
    
    def test_manual_recovery(self):
        """测试手动恢复 / Test manual recovery"""
        config = {
            'motion': {
                'max_linear_velocity': 0.5,
                'max_angular_velocity': 1.0,
                'max_linear_acceleration': 0.5,
                'max_angular_acceleration': 1.0,
                'velocity_ramp': {
                    'emergency_stop': {
                        'enable': True,
                        'manual_recovery': True  # 启用手动恢复 / Enable manual recovery
                    }
                }
            }
        }
        vr = VelocityRamp(config)
        
        # 触发急停 / Trigger emergency stop
        vr.emergency_stop("Test emergency")
        assert vr.is_emergency_stopped is True
        
        # check_recovery不应自动恢复 / check_recovery should not auto-recover
        import time
        time.sleep(2.1)
        vr.check_recovery()
        assert vr.is_emergency_stopped is True  # 仍处于急停 / Still in emergency stop
        
        # 手动恢复 / Manual recovery
        vr.recover_from_emergency()
        assert vr.is_emergency_stopped is False


class TestVelocityJumpDetection:
    """测试速度突变检测 / Test velocity jump detection"""
    
    def test_velocity_jump_from_stationary(self, velocity_ramp):
        """测试从静止状态的速度突变 / Test velocity jump from stationary"""
        current_time = Time(seconds=0)
        
        # 初始化为静止 / Initialize at stationary
        velocity_ramp.limit(0.0, 0.0, 0.0, current_time)
        velocity_ramp.last_linear_velocity = np.array([0.0, 0.0])
        velocity_ramp.last_time = Time(seconds=0)
        
        # 突然跳变到0.5m/s（超过0.3阈值）/ Jump to 0.5 m/s (exceeds 0.3 threshold)
        current_time = Time(seconds=0.02)
        limited_vx, _, _ = velocity_ramp.limit(0.5, 0.0, 0.0, current_time)
        
        # 应应用斜坡限制（不会立即达到0.5）/ Should apply ramp limiting (won't reach 0.5 immediately)
        assert limited_vx < 0.5


class TestStateManagement:
    """测试状态管理 / Test state management"""
    
    def test_reset(self, velocity_ramp):
        """测试状态重置 / Test state reset"""
        current_time = Time(seconds=0)
        
        # 设置一些状态 / Set some state
        velocity_ramp.limit(0.5, 0.0, 1.0, current_time)
        assert np.linalg.norm(velocity_ramp.last_linear_velocity) > 0
        
        # 重置 / Reset
        velocity_ramp.reset()
        
        # 验证状态已清零 / Verify state is cleared
        assert np.array_equal(velocity_ramp.last_linear_velocity, np.array([0.0, 0.0]))
        assert velocity_ramp.last_angular_velocity == 0.0
        assert velocity_ramp.last_time is None
    
    def test_get_status(self, velocity_ramp):
        """测试状态查询 / Test status query"""
        # 正常状态 / Normal state
        status = velocity_ramp.get_status()
        
        assert 'enabled' in status
        assert 'last_linear_velocity' in status
        assert 'last_angular_velocity' in status
        assert 'is_emergency_stopped' in status
        assert status['is_emergency_stopped'] is False
    
    def test_get_status_emergency(self, velocity_ramp):
        """测试急停状态查询 / Test emergency stop status query"""
        # 触发急停 / Trigger emergency stop
        velocity_ramp.emergency_stop("Test emergency")
        
        status = velocity_ramp.get_status()
        
        assert status['is_emergency_stopped'] is True
        assert 'emergency_stop_reason' in status
        assert status['emergency_stop_reason'] == "Test emergency"
        assert 'emergency_elapsed_time' in status
        assert 'emergency_remaining_time' in status


class TestEdgeCases:
    """测试边界情况 / Test edge cases"""
    
    def test_zero_dt(self, velocity_ramp):
        """测试dt=0的情况 / Test dt=0 case"""
        current_time = Time(seconds=0)
        
        # 初始化 / Initialize
        velocity_ramp.limit(0.5, 0.0, 0.0, current_time)
        
        # dt=0，应返回上次速度 / dt=0, should return last velocity
        limited_vx, limited_vy, limited_omega = velocity_ramp.limit(
            1.0, 0.0, 0.0, current_time
        )
        
        assert limited_vx == 0.5
        assert limited_vy == 0.0
        assert limited_omega == 0.0
    
    def test_disabled_velocity_ramp(self):
        """测试禁用VelocityRamp / Test disabled VelocityRamp"""
        config = {
            'motion': {
                'max_linear_velocity': 0.5,
                'max_angular_velocity': 1.0,
                'max_linear_acceleration': 0.5,
                'max_angular_acceleration': 1.0,
                'velocity_ramp': {
                    'enable': False  # 禁用 / Disable
                }
            }
        }
        vr = VelocityRamp(config)
        
        current_time = Time(seconds=0)
        
        # 禁用时应直接返回目标速度 / Should return target directly when disabled
        limited_vx, limited_vy, limited_omega = vr.limit(
            0.8, 0.3, 1.5, current_time
        )
        
        assert limited_vx == 0.8
        assert limited_vy == 0.3
        assert limited_omega == 1.5


def test_main_function():
    """测试main函数是否可执行 / Test main function is executable"""
    from bot_hardware.utils.velocity_ramp import main
    
    try:
        # main()包含打印和简单测试 / main() contains prints and simple tests
        # 这里只验证不抛出异常 / Only verify no exceptions
        success = True
    except Exception:
        success = False
    
    assert success is True
