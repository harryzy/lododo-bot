#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
OmniHardwareInterface单元测试
Unit tests for OmniHardwareInterface

测试覆盖 / Test Coverage:
- 生命周期方法测试 (on_init, on_configure, on_activate, on_deactivate)
- read()方法测试 (编码器→里程计)
- write()方法测试 (cmd_vel→轮速)
- P1组件集成测试
- 错误处理测试
"""

import pytest
import numpy as np
from unittest.mock import Mock, patch, MagicMock
import sys
import os

# 添加模块路径 / Add module path
sys.path.insert(0, os.path.dirname(os.path.dirname(__file__)))

from bot_hardware.hardware_interface.omni_hardware_interface import OmniHardwareInterface


@pytest.fixture
def hardware_interface():
    """创建OmniHardwareInterface实例 / Create OmniHardwareInterface instance"""
    interface = OmniHardwareInterface()
    # Mock logger
    interface.logger = Mock()
    interface.logger.info = Mock()
    interface.logger.warning = Mock()
    interface.logger.error = Mock()
    return interface


@pytest.fixture
def mock_hardware_info():
    """模拟hardware_info / Mock hardware_info"""
    return {}


class TestOmniHardwareInterfaceInit:
    """测试初始化 / Test initialization"""
    
    def test_constructor(self):
        """测试构造函数 / Test constructor"""
        interface = OmniHardwareInterface()
        
        # 验证初始状态 / Verify initial state
        assert interface.config is None
        assert interface.driver is None
        assert interface.encoder_handler is None
        assert interface.velocity_ramp is None
        assert interface.kinematics is None
        assert interface.servo_ids == []
        assert np.array_equal(interface.pose, np.array([0.0, 0.0, 0.0]))
        assert interface.last_time is None
        assert interface.read_count == 0
        assert interface.write_count == 0
    
    @patch('bot_hardware.hardware_interface.omni_hardware_interface.PathManager')
    @patch('builtins.open')
    @patch('yaml.safe_load')
    def test_on_init_success(self, mock_yaml_load, mock_open, mock_path_manager, 
                             hardware_interface, mock_hardware_info):
        """测试on_init成功 / Test on_init success"""
        # 设置mock / Setup mocks
        mock_config = {
            'servo': {
                'wheel_1_id': 1,
                'wheel_2_id': 2,
                'wheel_3_id': 3
            },
            'ros2_control': {
                'update_rate': 50
            }
        }
        mock_yaml_load.return_value = mock_config
        
        mock_path_mgr_instance = Mock()
        mock_path_mgr_instance.resolve_path.return_value = '/path/to/config.yaml'
        mock_path_manager.return_value = mock_path_mgr_instance
        
        # 执行 / Execute
        from hardware_interface import return_type
        result = hardware_interface.on_init(mock_hardware_info)
        
        # 验证 / Verify
        assert result == return_type.OK
        assert hardware_interface.config == mock_config
        assert hardware_interface.servo_ids == [1, 2, 3]
        hardware_interface.logger.info.assert_called()
    
    @patch('bot_hardware.hardware_interface.omni_hardware_interface.PathManager')
    def test_on_init_config_load_failure(self, mock_path_manager, 
                                         hardware_interface, mock_hardware_info):
        """测试on_init配置加载失败 / Test on_init config load failure"""
        # 设置mock抛出异常 / Setup mock to raise exception
        mock_path_manager.side_effect = Exception("Config file not found")
        
        # 执行 / Execute
        from hardware_interface import return_type
        result = hardware_interface.on_init(mock_hardware_info)
        
        # 验证 / Verify
        assert result == return_type.ERROR
        hardware_interface.logger.error.assert_called()


class TestLifecycleMethods:
    """测试生命周期方法 / Test lifecycle methods"""
    
    def test_on_configure_success(self, hardware_interface):
        """测试on_configure成功 / Test on_configure success"""
        # 准备配置 / Prepare config
        hardware_interface.config = {
            'serial': {
                'servo_port': '/dev/ttyUSB0',
                'servo_baudrate': 1000000,
                'servo_timeout': 0.1
            },
            'servo': {
                'encoder_resolution': 4096
            },
            'kinematics': {
                'wheel_radius': 0.05,
                'wheel_base': {
                    'L1': 0.126377,
                    'L2': 0.125897,
                    'L3': 0.125897
                },
                'jacobian': [
                    [0.0, 20.0, 2.52754],
                    [17.32051, 10.0, 2.51794],
                    [-17.32051, 10.0, 2.51794]
                ]
            },
            'motion': {
                'max_linear_velocity': 0.5,
                'max_angular_velocity': 1.0,
                'max_linear_acceleration': 0.5,
                'max_angular_acceleration': 1.0
            }
        }
        
        # Mock组件 / Mock components
        with patch('bot_hardware.hardware_interface.omni_hardware_interface.ST3215Driver'), \
             patch('bot_hardware.hardware_interface.omni_hardware_interface.EncoderHandler'), \
             patch('bot_hardware.hardware_interface.omni_hardware_interface.OmniKinematics') as mock_kinematics, \
             patch('bot_hardware.hardware_interface.omni_hardware_interface.VelocityRamp') as mock_velocity_ramp:
            
            # Mock get_parameters()返回真实字典 / Mock get_parameters() to return real dict
            mock_kinematics.return_value.get_parameters.return_value = {
                'wheel_radius': 0.05,
                'L1': 0.126377,
                'L2': 0.125897,
                'L3': 0.125897
            }
            
            # Mock VelocityRamp属性 / Mock VelocityRamp attributes
            mock_velocity_ramp.return_value.max_linear_accel = 0.5
            mock_velocity_ramp.return_value.max_angular_accel = 1.0
            
            from hardware_interface import return_type
            result = hardware_interface.on_configure(None)
            
            # 验证 / Verify
            assert result == return_type.OK
            assert hardware_interface.driver is not None
            assert hardware_interface.encoder_handler is not None
            assert hardware_interface.kinematics is not None
            assert hardware_interface.velocity_ramp is not None
            hardware_interface.logger.info.assert_called()
    
    def test_on_activate_success(self, hardware_interface):
        """测试on_activate成功 / Test on_activate success"""
        # 准备 / Prepare
        hardware_interface.servo_ids = [1, 2, 3]
        hardware_interface.driver = Mock()
        hardware_interface.driver.initialize.return_value = True
        hardware_interface.driver.ping.return_value = True
        
        # Mock _read_current_wheel_velocities方法 / Mock _read_current_wheel_velocities
        from hardware_interface import return_type
        hardware_interface._read_current_wheel_velocities = Mock(return_value=return_type.OK)
        
        # 执行 / Execute
        result = hardware_interface.on_activate(None)
        
        # 验证 / Verify
        assert result == return_type.OK
        assert hardware_interface.last_time is not None
        assert hardware_interface.last_command_time is not None
        assert hardware_interface.driver.initialize.called
        assert hardware_interface.driver.ping.call_count == 3
        assert hardware_interface._read_current_wheel_velocities.called
        hardware_interface.logger.info.assert_called()
    
    def test_on_activate_servo_not_responding(self, hardware_interface):
        """测试on_activate舵机不响应 / Test on_activate servo not responding"""
        # 准备 / Prepare
        hardware_interface.servo_ids = [1, 2, 3]
        hardware_interface.driver = Mock()
        hardware_interface.driver.initialize.return_value = True
        # 第2个舵机不响应 / Servo 2 not responding
        hardware_interface.driver.ping.side_effect = [True, False, True]
        
        # 执行 / Execute
        from hardware_interface import return_type
        result = hardware_interface.on_activate(None)
        
        # 验证 / Verify
        assert result == return_type.ERROR
        hardware_interface.logger.error.assert_called()
    
    def test_on_deactivate_success(self, hardware_interface):
        """测试on_deactivate成功 / Test on_deactivate success"""
        # 准备 / Prepare
        hardware_interface.servo_ids = [1, 2, 3]
        hardware_interface.driver = Mock()
        hardware_interface.driver.write_speed.return_value = True
        hardware_interface.driver.close = Mock()
        hardware_interface.last_time = 123.456
        
        # 执行 / Execute
        from hardware_interface import return_type
        result = hardware_interface.on_deactivate(None)
        
        # 验证 / Verify
        assert result == return_type.OK
        assert hardware_interface.driver.write_speed.call_count == 3
        assert hardware_interface.driver.close.called
        assert hardware_interface.last_time is None
    
    def test_on_cleanup_success(self, hardware_interface):
        """测试on_cleanup成功 / Test on_cleanup success"""
        # 准备 / Prepare
        hardware_interface.driver = Mock()
        hardware_interface.encoder_handler = Mock()
        hardware_interface.velocity_ramp = Mock()
        hardware_interface.kinematics = Mock()
        
        # 执行 / Execute
        from hardware_interface import return_type
        result = hardware_interface.on_cleanup(None)
        
        # 验证 / Verify
        assert result == return_type.OK
        assert hardware_interface.driver is None
        assert hardware_interface.encoder_handler is None
        assert hardware_interface.velocity_ramp is None
        assert hardware_interface.kinematics is None


class TestReadMethod:
    """测试read()方法 / Test read() method"""
    
    def test_read_success(self, hardware_interface):
        """测试read成功 / Test read success"""
        # 准备 / Prepare
        hardware_interface.servo_ids = [1, 2, 3]
        hardware_interface.last_time = 100.0
        hardware_interface.config = {
            'odometry': {
                'frame_id': 'odom',
                'child_frame_id': 'base_link',
                'pose_covariance': {'xx': 0.1, 'yy': 0.1, 'tt': 0.3},
                'twist_covariance': {'vx': 0.01, 'vy': 0.01, 'omega': 0.01}
            }
        }
        
        # Mock组件 / Mock components
        hardware_interface.driver = Mock()
        hardware_interface.driver.read_position.side_effect = [100, 200, 300]
        
        hardware_interface.encoder_handler = Mock()
        hardware_interface.encoder_handler.get_velocity_rad_s.side_effect = [1.0, 1.0, 1.0]
        
        hardware_interface.kinematics = Mock()
        hardware_interface.kinematics.forward_kinematics.return_value = (0.1, 0.0, 0.0)
        
        hardware_interface.odom_pub = Mock()
        
        # Mock time / 模拟时间
        with patch('time.time', return_value=100.02):
            from hardware_interface import return_type
            result = hardware_interface.read(None, None)
        
        # 验证 / Verify
        assert result == return_type.OK
        assert hardware_interface.read_count == 1
        assert hardware_interface.driver.read_position.call_count == 3
        assert hardware_interface.encoder_handler.get_velocity_rad_s.call_count == 3
    
    def test_read_encoder_failure(self, hardware_interface):
        """测试read编码器读取失败 / Test read encoder failure"""
        # 准备 / Prepare
        hardware_interface.servo_ids = [1, 2, 3]
        hardware_interface.last_time = 100.0
        
        hardware_interface.driver = Mock()
        hardware_interface.driver.read_position.side_effect = [100, None, 300]  # 第2个失败
        
        # 执行 / Execute
        with patch('time.time', return_value=100.02):
            from hardware_interface import return_type
            result = hardware_interface.read(None, None)
        
        # 验证 / Verify
        assert result == return_type.ERROR
        assert hardware_interface.read_errors == 1
    
    def test_read_large_dt_handling(self, hardware_interface):
        """测试read大dt处理 / Test read large dt handling"""
        # 准备 / Prepare
        hardware_interface.servo_ids = [1, 2, 3]
        hardware_interface.last_time = 100.0
        
        hardware_interface.driver = Mock()
        hardware_interface.driver.read_position.side_effect = [100, 200, 300]
        
        hardware_interface.encoder_handler = Mock()
        hardware_interface.encoder_handler.get_velocity_rad_s.return_value = 0.0
        
        hardware_interface.kinematics = Mock()
        hardware_interface.kinematics.forward_kinematics.return_value = (0.0, 0.0, 0.0)
        
        # 模拟大时间跳变 (0.5s > 0.2s阈值) / Simulate large time jump
        with patch('time.time', return_value=100.5):
            from hardware_interface import return_type
            result = hardware_interface.read(None, None)
        
        # 验证: 应该警告并重置dt / Verify: should warn and reset dt
        assert result == return_type.OK
        hardware_interface.logger.warning.assert_called()


class TestWriteMethod:
    """测试write()方法 / Test write() method"""
    
    def test_write_success(self, hardware_interface):
        """测试write成功 / Test write success"""
        # 准备 / Prepare
        hardware_interface.servo_ids = [1, 2, 3]
        
        hardware_interface.velocity_ramp = Mock()
        hardware_interface.velocity_ramp.limit.return_value = (0.1, 0.0, 0.0)
        
        hardware_interface.kinematics = Mock()
        hardware_interface.kinematics.inverse_kinematics.return_value = (1.0, 1.0, 1.0)
        
        hardware_interface.driver = Mock()
        hardware_interface.driver.write_speed.return_value = True
        
        # 执行 / Execute
        from hardware_interface import return_type
        result = hardware_interface.write(None, None)
        
        # 验证 / Verify
        assert result == return_type.OK
        assert hardware_interface.write_count == 1
        assert hardware_interface.velocity_ramp.limit.called
        assert hardware_interface.kinematics.inverse_kinematics.called
        assert hardware_interface.driver.write_speed.call_count == 3
    
    def test_write_servo_write_failure(self, hardware_interface):
        """测试write舵机写入失败 / Test write servo write failure"""
        # 准备 / Prepare
        hardware_interface.servo_ids = [1, 2, 3]
        
        hardware_interface.velocity_ramp = Mock()
        hardware_interface.velocity_ramp.limit.return_value = (0.1, 0.0, 0.0)
        
        hardware_interface.kinematics = Mock()
        hardware_interface.kinematics.inverse_kinematics.return_value = (1.0, 1.0, 1.0)
        
        hardware_interface.driver = Mock()
        # 第2个舵机写入失败 / Servo 2 write fails
        hardware_interface.driver.write_speed.side_effect = [True, False, True]
        
        # 执行 / Execute
        from hardware_interface import return_type
        result = hardware_interface.write(None, None)
        
        # 验证 / Verify
        assert result == return_type.OK  # write不因单个舵机失败而失败
        assert hardware_interface.write_errors == 1
        hardware_interface.logger.warning.assert_called()


class TestP1ComponentIntegration:
    """测试P1组件集成 / Test P1 component integration"""
    
    def test_component_initialization_order(self, hardware_interface):
        """测试组件初始化顺序 / Test component initialization order"""
        # Round 7设计: driver → encoder → kinematics → velocity_ramp
        
        hardware_interface.config = {
            'serial': {'servo_port': '/dev/ttyUSB0', 'servo_baudrate': 1000000, 'servo_timeout': 0.1},
            'servo': {'encoder_resolution': 4096},
            'kinematics': {
                'wheel_radius': 0.05,
                'wheel_base': {'L1': 0.126377, 'L2': 0.125897, 'L3': 0.125897},
                'jacobian': [
                    [0.0, 20.0, 2.52754],
                    [17.32051, 10.0, 2.51794],
                    [-17.32051, 10.0, 2.51794]
                ]
            },
            'motion': {
                'max_linear_velocity': 0.5,
                'max_angular_velocity': 1.0,
                'max_linear_acceleration': 0.5,
                'max_angular_acceleration': 1.0
            }
        }
        
        initialization_order = []
        
        def track_init(name):
            """跟踪初始化顺序的工厂函数 / Factory to track initialization order"""
            def factory(*args, **kwargs):
                initialization_order.append(name)
                mock_obj = Mock()
                
                # 为特定Mock设置返回值 / Set return values for specific mocks
                if name == 'kinematics':
                    mock_obj.get_parameters.return_value = {
                        'wheel_radius': 0.05,
                        'L1': 0.126377,
                        'L2': 0.125897,
                        'L3': 0.125897
                    }
                elif name == 'velocity_ramp':
                    mock_obj.max_linear_accel = 0.5
                    mock_obj.max_angular_accel = 1.0
                
                return mock_obj
            return factory
        
        with patch('bot_hardware.hardware_interface.omni_hardware_interface.ST3215Driver', side_effect=track_init('driver')), \
             patch('bot_hardware.hardware_interface.omni_hardware_interface.EncoderHandler', side_effect=track_init('encoder')), \
             patch('bot_hardware.hardware_interface.omni_hardware_interface.OmniKinematics', side_effect=track_init('kinematics')), \
             patch('bot_hardware.hardware_interface.omni_hardware_interface.VelocityRamp', side_effect=track_init('velocity_ramp')):
            
            hardware_interface.on_configure(None)
        
        # 验证初始化顺序 / Verify initialization order
        assert initialization_order == ['driver', 'encoder', 'kinematics', 'velocity_ramp']


class TestStatistics:
    """测试统计功能 / Test statistics"""
    
    def test_get_statistics(self, hardware_interface):
        """测试统计信息获取 / Test get statistics"""
        # 设置统计数据 / Set statistics data
        hardware_interface.read_count = 100
        hardware_interface.write_count = 100
        hardware_interface.read_errors = 5
        hardware_interface.write_errors = 2
        
        # 获取统计 / Get statistics
        stats = hardware_interface.get_statistics()
        
        # 验证 / Verify
        assert stats['read_count'] == 100
        assert stats['write_count'] == 100
        assert stats['read_errors'] == 5
        assert stats['write_errors'] == 2
        assert stats['read_success_rate'] == 0.95  # (100-5)/100
        assert stats['write_success_rate'] == 0.98  # (100-2)/100


def test_main_function():
    """测试main函数 / Test main function"""
    from bot_hardware.hardware_interface.omni_hardware_interface import main
    
    try:
        main()
        success = True
    except Exception:
        success = False
    
    assert success is True
