#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
OmniHardwareInterface - ROS2 Control硬件接口 
ROS2 Control Hardware Interface for Omnidirectional Robot

功能特性 / Features:
- ros2_control SystemInterface实现 / ros2_control SystemInterface implementation
- 生命周期管理 (on_init → on_configure → on_activate) / Lifecycle management
- read()方法: 编码器 → 里程计 / read() method: encoder → odometry
- write()方法: cmd_vel → 轮速 / write() method: cmd_vel → wheel velocities
- 集成P1所有组件 / Integrates all P1 components

设计参考 / Design Reference:
- 主设计: HARDWARE_DEPLOYMENT_DESIGN.md §3.2.5 (行2649-2971)
- Round 7修订: 正确的初始化顺序（行2756-2780）
- 实现路线: IMPLEMENTATION_ROADMAP.md P2阶段（行209-286）
"""

import numpy as np
import time
import yaml
import os
from typing import List, Dict, Optional

# ROS2导入 / ROS2 imports
try:
    from hardware_interface import SystemInterface, return_type, StateInterface, CommandInterface
    from rclpy.time import Time
    from nav_msgs.msg import Odometry
    from geometry_msgs.msg import TransformStamped, Quaternion
    from tf2_ros import TransformBroadcaster
    from ament_index_python.packages import get_package_share_directory
except ImportError:
    # 单元测试环境可能没有ROS2依赖 / Unit test environment may not have ROS2 dependencies
    StateInterface = None
    CommandInterface = None

# P1组件导入 / P1 component imports
from bot_hardware.drivers.st3215_driver import ST3215Driver
from bot_hardware.utils.encoder_handler import EncoderHandler
from bot_hardware.utils.velocity_ramp import VelocityRamp
from bot_hardware.utils.omni_kinematics import OmniKinematics
from bot_hardware.utils.path_manager import PathManager


class OmniHardwareInterface(SystemInterface):
    """全向轮硬件接口 / Omnidirectional hardware interface
    
    ros2_control生命周期 / ros2_control Lifecycle:
    1. on_init(hardware_info)     - 加载配置，解析URDF参数
    2. on_configure(previous)     - 初始化驱动和工具类
    3. on_activate(previous)      - 激活硬件，启动控制循环
    4. read(time, duration)       - 读取编码器 → 里程计
    5. write(time, duration)      - cmd_vel → 轮速
    6. on_deactivate(previous)    - 停止控制循环
    7. on_cleanup(previous)       - 清理资源
    8. on_shutdown(previous)      - 关闭硬件
    
    数据流 / Data Flow:
    - write(): cmd_vel → VelocityRamp → OmniKinematics → ST3215Driver
    - read():  ST3215Driver → EncoderHandler → OmniKinematics → odom
    """
    
    def __init__(self):
        """构造函数 / Constructor
        
        注意: ros2_control框架会注入logger, 不在这里创建
        """
        super().__init__()
        
        # 配置管理 / Configuration management
        self.config = None
        self.path_manager = None
        
        # P1组件实例 / P1 component instances
        self.driver: Optional[ST3215Driver] = None
        self.encoder_handler: Optional[EncoderHandler] = None
        self.velocity_ramp: Optional[VelocityRamp] = None
        self.kinematics: Optional[OmniKinematics] = None
        
        # 状态变量 / State variables
        self.servo_ids: List[int] = []
        self.pose = np.array([0.0, 0.0, 0.0])  # [x, y, theta] in odom frame
        self.velocity = np.array([0.0, 0.0, 0.0])  # [vx, vy, omega] current velocity
        self.last_time: Optional[float] = None
        self.last_command_time: Optional[float] = None  # 看门狗超时检测 / Watchdog timeout detection
        
        # ros2_control接口 / ros2_control interfaces
        # State interfaces: 位姿+速度 (x, y, theta, vx, vy, omega)
        self.state_interfaces_data = {
            'base_link/position_x': 0.0,
            'base_link/position_y': 0.0,
            'base_link/position_theta': 0.0,
            'base_link/velocity_x': 0.0,
            'base_link/velocity_y': 0.0,
            'base_link/velocity_theta': 0.0
        }
        # Command interfaces: 目标速度 (vx, vy, omega)
        self.command_interfaces_data = {
            'base_link/linear_x': 0.0,
            'base_link/linear_y': 0.0,
            'base_link/angular_z': 0.0
        }
        
        # ROS2发布器 / ROS2 publishers
        self.odom_pub = None
        self.tf_broadcaster = None
        self.node = None  # ROS2 node for creating publishers
        
        # 性能监控 / Performance monitoring
        self.read_count = 0
        self.write_count = 0
        self.read_errors = 0
        self.write_errors = 0
    
    def on_init(self, hardware_info) -> return_type:
        """初始化硬件接口 / Initialize hardware interface
        
        Args:
            hardware_info: 从URDF解析的硬件参数 / Hardware parameters from URDF
        
        Returns:
            return_type.OK / return_type.ERROR
        
        步骤 / Steps:
        1. 创建PathManager
        2. 加载hardware_config.yaml
        3. 读取舵机ID列表
        4. 记录配置信息
        """
        try:
            # Step 1: 创建PathManager / Create PathManager
            self.path_manager = PathManager()
            
            # Step 2: 加载配置文件 / Load config file
            config_path = self.path_manager.resolve_path(
                'package:/config/hardware_config.yaml'
            )
            
            with open(config_path, 'r', encoding='utf-8') as f:
                self.config = yaml.safe_load(f)
            
            # Step 3: 读取舵机ID / Read servo IDs
            servo_cfg = self.config['servo']
            self.servo_ids = [
                servo_cfg['wheel_1_id'],  # 后轮 / Rear wheel
                servo_cfg['wheel_2_id'],  # 右前轮 / Right front wheel
                servo_cfg['wheel_3_id']   # 左前轮 / Left front wheel
            ]
            
            # Step 4: 记录配置信息 / Log configuration
            if hasattr(self, 'logger') and self.logger:
                self.logger.info(f'Hardware config loaded from: {config_path}')
                self.logger.info(f'Servo IDs: {self.servo_ids}')
                self.logger.info(
                    f'Control frequency: {self.config["ros2_control"]["update_rate"]} Hz'
                )
            
            return return_type.OK
            
        except Exception as e:
            if hasattr(self, 'logger') and self.logger:
                self.logger.error(f'Failed to initialize hardware interface: {e}')
            return return_type.ERROR
    
    def on_configure(self, previous_state) -> return_type:
        """配置硬件 / Configure hardware
        
        Round 7修订: 正确的初始化顺序，解决循环依赖
        
        初始化顺序 / Initialization Order (Critical):
        1. ST3215Driver (无依赖)
        2. EncoderHandler (依赖: config)
        3. OmniKinematics (依赖: config)
        4. VelocityRamp (依赖: config, 不需要当前速度)
        
        Returns:
            return_type.OK / return_type.ERROR
        """
        try:
            # Step 1: 初始化舵机驱动 / Initialize servo driver
            serial_cfg = self.config['serial']
            self.driver = ST3215Driver(
                port=serial_cfg['servo_port'],
                baudrate=serial_cfg['servo_baudrate'],
                timeout=serial_cfg['servo_timeout']
            )
            self._log_info('ST3215Driver initialized')
            
            # Step 2: 初始化编码器处理器 / Initialize encoder handler
            self.encoder_handler = EncoderHandler(self.config)
            self._log_info('EncoderHandler initialized')
            
            # Step 3: 初始化运动学工具 / Initialize kinematics tool
            self.kinematics = OmniKinematics(self.config)
            kinematics_params = self.kinematics.get_parameters()
            self._log_info(
                f'OmniKinematics initialized: '
                f'R={kinematics_params["wheel_radius"]}m, '
                f'L1={kinematics_params["L1"]:.4f}m'
            )
            
            # Step 4: 初始化速度斜坡 / Initialize velocity ramp
            # Round 7注释: 不需要读取当前速度，VelocityRamp在首次limit()调用时自动初始化
            self.velocity_ramp = VelocityRamp(self.config)
            self._log_info(
                f'VelocityRamp initialized: '
                f'linear_accel={self.velocity_ramp.max_linear_accel}m/s², '
                f'angular_accel={self.velocity_ramp.max_angular_accel}rad/s²'
            )
            
            # TODO: Step 5: 创建ROS2发布器 (需要Node上下文)
            # self.odom_pub = node.create_publisher(Odometry, '/wheel/odom', 10)
            # self.tf_broadcaster = TransformBroadcaster(node)
            
            return return_type.OK
            
        except Exception as e:
            self._log_error(f'Failed to configure hardware: {e}')
            return return_type.ERROR
    
    def export_state_interfaces(self) -> List:
        """导出状态接口 / Export state interfaces
        
        P2.2新增: 向ros2_control框架暴露状态数据
        
        State Interfaces (6个):
        - base_link/position_x: X位置 (m)
        - base_link/position_y: Y位置 (m)
        - base_link/position_theta: 航向角 (rad)
        - base_link/velocity_x: X速度 (m/s)
        - base_link/velocity_y: Y速度 (m/s)
        - base_link/velocity_theta: 角速度 (rad/s)
        
        Returns:
            List[StateInterface]: 状态接口列表
        """
        if StateInterface is None:
            return []  # 测试环境返回空列表
        
        state_interfaces = []
        for name, value in self.state_interfaces_data.items():
            state_interfaces.append(
                StateInterface(
                    interface_name=name,
                    initial_value=value
                )
            )
        
        self._log_info(f'Exported {len(state_interfaces)} state interfaces')
        return state_interfaces
    
    def export_command_interfaces(self) -> List:
        """导出命令接口 / Export command interfaces
        
        P2.2新增: 向ros2_control框架暴露命令数据
        
        Command Interfaces (3个):
        - base_link/linear_x: 目标X速度 (m/s)
        - base_link/linear_y: 目标Y速度 (m/s)
        - base_link/angular_z: 目标角速度 (rad/s)
        
        Returns:
            List[CommandInterface]: 命令接口列表
        """
        if CommandInterface is None:
            return []  # 测试环境返回空列表
        
        command_interfaces = []
        for name, value in self.command_interfaces_data.items():
            command_interfaces.append(
                CommandInterface(
                    interface_name=name,
                    initial_value=value
                )
            )
        
        self._log_info(f'Exported {len(command_interfaces)} command interfaces')
        return command_interfaces
    
    def on_activate(self, previous_state) -> return_type:
        """激活硬件 / Activate hardware
        
        步骤 / Steps:
        1. 初始化舵机串口
        2. 扫描舵机在线状态
        3. 记录激活时间
        
        Returns:
            return_type.OK / return_type.ERROR
        """
        try:
            # Step 1: 初始化串口 / Initialize serial port
            if not self.driver.initialize():
                self._log_error('Failed to initialize ST3215 serial port')
                return return_type.ERROR
            
            # Step 2: 扫描舵机在线状态 / Scan servo online status
            self._log_info('Scanning servos...')
            for servo_id in self.servo_ids:
                if not self.driver.ping(servo_id):
                    self._log_error(f'Servo {servo_id} not responding!')
                    return return_type.ERROR
            
            # Step 3: 读取当前轮速，初始化velocity_ramp / Read current velocities
            result = self._read_current_wheel_velocities()
            if result != return_type.OK:
                self._log_error('Failed to read current wheel velocities')
                return return_type.ERROR
            
            # Step 4: 记录激活时间 / Record activation time
            self.last_time = time.time()
            self.last_command_time = time.time()  # 初始化看门狗计时器 / Initialize watchdog timer
            self._log_info('All servos online, hardware activated successfully')
            
            return return_type.OK
            
        except Exception as e:
            self._log_error(f'Failed to activate hardware: {e}')
            return return_type.ERROR
    
    def read(self, time_arg, duration) -> return_type:
        """读取硬件状态 / Read hardware state
        
        数据流程 / Data Flow:
        1. 读取编码器位置 (ticks)
        2. 处理编码器溢出 → 位置增量 (ticks)
        3. 计算轮子角速度 (rad/s)
        4. 正向运动学 → 机器人速度 (vx, vy, omega)
        5. 积分更新位姿 → 发布里程计
        
        Args:
            time_arg: ros2_control传递的时间戳
            duration: 上次调用以来的时间间隔
        
        Returns:
            return_type.OK / return_type.ERROR
        """
        try:
            current_time = time.time()
            dt = current_time - self.last_time if self.last_time else 0.02
            
            # 防止首次调用或时间跳变 / Prevent first call or time jump
            if dt > 0.2:
                self._log_warning(f'Large dt detected: {dt:.3f}s, resetting to 0.02s')
                dt = 0.02
            
            # Step 1: 读取编码器位置 / Read encoder positions
            encoder_positions = []
            for servo_id in self.servo_ids:
                pos = self.driver.read_position(servo_id)
                if pos is None:
                    self._log_warning(f'Failed to read servo {servo_id} position')
                    self.read_errors += 1
                    return return_type.ERROR
                encoder_positions.append(pos)
            
            # Step 2: 计算轮子角速度 / Calculate wheel velocities
            wheel_velocities = []
            for i, (servo_id, current_pos) in enumerate(zip(self.servo_ids, encoder_positions)):
                # EncoderHandler已集成溢出检测和速度计算
                velocity_rad_s = self.encoder_handler.get_velocity_rad_s(
                    servo_id, current_pos, dt
                )
                wheel_velocities.append(velocity_rad_s)
            
            # Step 3: 正向运动学 / Forward kinematics
            vx, vy, omega = self.kinematics.forward_kinematics(
                wheel_velocities[0],
                wheel_velocities[1],
                wheel_velocities[2]
            )
            
            # Step 4: 更新位姿 / Update pose (Euler integration)
            if dt > 0:
                # 在odom坐标系下积分 / Integrate in odom frame
                cos_theta = np.cos(self.pose[2])
                sin_theta = np.sin(self.pose[2])
                
                # 从base_link坐标系转换到odom坐标系
                # Transform from base_link to odom frame
                dx_odom = vx * cos_theta - vy * sin_theta
                dy_odom = vx * sin_theta + vy * cos_theta
                dtheta = omega
                
                self.pose[0] += dx_odom * dt  # x
                self.pose[1] += dy_odom * dt  # y
                self.pose[2] += dtheta * dt   # theta
                
                # 归一化角度到[-π, π] / Normalize angle to [-π, π]
                self.pose[2] = np.arctan2(np.sin(self.pose[2]), np.cos(self.pose[2]))
            
            omega_z = omega  # 保持一致的命名
            
            # Step 5: 更新state_interfaces / Update state interfaces
            self.state_interfaces_data['base_link/position_x'] = self.pose[0]
            self.state_interfaces_data['base_link/position_y'] = self.pose[1]
            self.state_interfaces_data['base_link/position_theta'] = self.pose[2]
            self.state_interfaces_data['base_link/velocity_x'] = vx
            self.state_interfaces_data['base_link/velocity_y'] = vy
            self.state_interfaces_data['base_link/velocity_theta'] = omega_z
            self.velocity = np.array([vx, vy, omega_z])  # 保存速度供write()使用
            
            # Step 6: 发布里程计 / Publish odometry
            self._publish_odometry(vx, vy, omega_z, current_time)
            
            # 更新时间和统计 / Update time and statistics
            self.last_time = current_time
            self.read_count += 1
            
            return return_type.OK
            
        except Exception as e:
            self._log_error(f'Read error: {e}')
            self.read_errors += 1
            return return_type.ERROR
    
    def write(self, time_arg, duration) -> return_type:
        """写入控制指令 / Write control commands
        
        数据流程 / Data Flow:
        1. 读取速度指令 (from controller_manager)
        2. 速度斜坡限制 → 限制后的速度
        3. 逆向运动学 → 轮子角速度 (rad/s)
        4. 转换为舵机RPM
        5. 写入舵机驱动
        
        Args:
            time_arg: ros2_control传递的时间戳
            duration: 上次调用以来的时间间隔
        
        Returns:
            return_type.OK / return_type.ERROR
        """
        try:
            current_time = time.time()
            dt = current_time - self.last_time if self.last_time else 0.001
            
            # Step 1: 读取目标速度 / Read target velocity from command_interfaces
            target_vx = self.command_interfaces_data['base_link/linear_x']
            target_vy = self.command_interfaces_data['base_link/linear_y']
            target_omega_z = self.command_interfaces_data['base_link/angular_z']
            
            # 看门狗超时检测 (0.5s无cmd_vel则停止) / Watchdog timeout detection
            if self.last_command_time is not None:
                command_timeout = 0.5  # 从config读取 / Read from config
                if hasattr(self.config.get('motion', {}), 'get'):
                    command_timeout = self.config['motion'].get('command_timeout', 0.5)
                
                # 检测是否有非零指令 / Detect non-zero command
                if abs(target_vx) > 0.001 or abs(target_vy) > 0.001 or abs(target_omega_z) > 0.001:
                    self.last_command_time = current_time
                # 超时则清零指令 / Timeout then zero command
                elif (current_time - self.last_command_time) > command_timeout:
                    target_vx = target_vy = target_omega_z = 0.0
                    if self.write_count % 50 == 0:  # 每秒记录一次 / Log once per second
                        self._log_info('Watchdog timeout: no command received, stopping robot')
            
            # Step 2: 速度斜坡限制 / Velocity ramp limiting
            limited_vx, limited_vy, limited_omega = self.velocity_ramp.limit(
                target_vx, target_vy, target_omega_z, dt
            )
            
            # Step 3: 逆向运动学 / Inverse kinematics
            w1, w2, w3 = self.kinematics.inverse_kinematics(
                limited_vx, limited_vy, limited_omega
            )
            
            # Step 4 & 5: 转换为RPM并写入 / Convert to RPM and write
            wheel_velocities_rad_s = [w1, w2, w3]
            for i, wheel_vel_rad_s in enumerate(wheel_velocities_rad_s):
                # 转换: rad/s → RPM / Convert: rad/s → RPM
                rpm = wheel_vel_rad_s * 30 / np.pi
                
                servo_id = self.servo_ids[i]
                success = self.driver.write_speed(servo_id, rpm)
                
                if not success:
                    self._log_warning(f'Failed to write speed to servo {servo_id}')
                    self.write_errors += 1
            
            # 更新统计 / Update statistics
            self.write_count += 1
            
            return return_type.OK
            
        except Exception as e:
            self._log_error(f'Write error: {e}')
            self.write_errors += 1
            return return_type.ERROR
    
    def on_deactivate(self, previous_state) -> return_type:
        """停用硬件 / Deactivate hardware
        
        步骤 / Steps:
        1. 停止所有轮子
        2. 关闭串口
        3. 清理状态
        
        Returns:
            return_type.OK / return_type.ERROR
        """
        try:
            # Step 1: 停止所有轮子 / Stop all wheels
            for servo_id in self.servo_ids:
                self.driver.write_speed(servo_id, 0.0)
            
            self._log_info('All servos stopped')
            
            # Step 2: 关闭串口 / Close serial port
            if self.driver:
                self.driver.close()
            
            # Step 3: 清理状态 / Clean state
            self.last_time = None
            
            self._log_info('Hardware deactivated')
            
            return return_type.OK
            
        except Exception as e:
            self._log_error(f'Failed to deactivate hardware: {e}')
            return return_type.ERROR
    
    def on_cleanup(self, previous_state) -> return_type:
        """清理资源 / Cleanup resources"""
        try:
            # 重置组件引用 / Reset component references
            self.driver = None
            self.encoder_handler = None
            self.velocity_ramp = None
            self.kinematics = None
            
            self._log_info('Hardware resources cleaned up')
            
            return return_type.OK
            
        except Exception as e:
            self._log_error(f'Failed to cleanup hardware: {e}')
            return return_type.ERROR
    
    def on_shutdown(self, previous_state) -> return_type:
        """关闭硬件 / Shutdown hardware"""
        try:
            self._log_info('Hardware shutdown')
            return return_type.OK
        except Exception as e:
            self._log_error(f'Failed to shutdown hardware: {e}')
            return return_type.ERROR
    
    def _read_current_wheel_velocities(self) -> return_type:
        """读取当前轮子速度用于初始化 / Read current wheel velocities for initialization
        
        设计参考 / Design Reference:
        - IMPLEMENTATION_ROADMAP.md P2.4 (行254-262)
        - HARDWARE_DEPLOYMENT_DESIGN.md Round 7 完整实现
        
        步骤 / Steps:
        0. 预初始化EncoderHandler基准
        1. 等待20ms精确计时
        2. 第二次读取计算速度
        3. 异常处理
        
        Returns:
            return_type.OK / return_type.ERROR
        """
        try:
            # Step 0: 预初始化EncoderHandler基准 / Pre-initialize encoder baseline
            self._log_info('Reading current wheel velocities for VelocityRamp initialization...')
            
            for servo_id in self.servo_ids:
                pos = self.driver.read_position(servo_id)
                if pos is None:
                    self._log_error(f'Failed to read initial position from servo {servo_id}')
                    return return_type.ERROR
                # 调用get_position_delta会自动初始化基准 / Calling get_position_delta auto-initializes baseline
                self.encoder_handler.get_position_delta(servo_id, pos)
            
            # Step 1: 等待20ms精确计时 / Wait 20ms for precise timing
            time_start = time.time()
            time.sleep(0.02)  # 20ms
            
            # Step 2: 第二次读取计算速度 / Second read to calculate velocity
            wheel_velocities_rad_s = []
            for servo_id in self.servo_ids:
                pos = self.driver.read_position(servo_id)
                if pos is None:
                    self._log_error(f'Failed to read second position from servo {servo_id}')
                    return return_type.ERROR
                
                # 使用EncoderHandler的get_velocity_rad_s计算速度
                dt = time.time() - time_start
                wheel_vel = self.encoder_handler.get_velocity_rad_s(servo_id, pos, dt)
                wheel_velocities_rad_s.append(wheel_vel)
            
            # Step 3: 正向运动学计算机器人速度 / Forward kinematics for robot velocity
            vx, vy, omega = self.kinematics.forward_kinematics(
                wheel_velocities_rad_s[0],
                wheel_velocities_rad_s[1],
                wheel_velocities_rad_s[2]
            )
            
            # 更新velocity_ramp目标速度 / Update velocity_ramp target velocity
            self.velocity_ramp.target_velocity = (vx, vy, omega)
            self._log_info(
                f'Current velocities initialized: '
                f'vx={vx:.3f} m/s, vy={vy:.3f} m/s, omega={omega:.3f} rad/s'
            )
            
            return return_type.OK
            
        except Exception as e:
            self._log_error(f'Failed to read current wheel velocities: {e}')
            return return_type.ERROR
    
    def _publish_odometry(self, vx: float, vy: float, omega: float, timestamp: float):
        """发布里程计消息 / Publish odometry message
        
        P2.2完善: 实现完整的Odometry发布逻辑
        
        Args:
            vx: X方向线速度 (m/s)
            vy: Y方向线速度 (m/s)
            omega: 角速度 (rad/s)
            timestamp: 时间戳 (seconds since epoch)
        """
        if self.odom_pub is None:
            return  # Publisher未初始化，跳过发布
        
        try:
            # 创建里程计消息 / Create odometry message
            odom = Odometry()
            odom.header.stamp = Time(seconds=timestamp).to_msg()
            
            # 从配置读取frame_id或使用默认值
            if 'odometry' in self.config:
                odom.header.frame_id = self.config['odometry'].get('frame_id', 'odom')
                odom.child_frame_id = self.config['odometry'].get('child_frame_id', 'base_link')
            else:
                odom.header.frame_id = 'odom'
                odom.child_frame_id = 'base_link'
            
            # 位置 / Position
            odom.pose.pose.position.x = self.pose[0]
            odom.pose.pose.position.y = self.pose[1]
            odom.pose.pose.position.z = 0.0
            
            # 姿态 (四元数) / Orientation (quaternion)
            # 简化计算: 绕Z轴旋转 / Simplified: rotation around Z axis
            theta = self.pose[2]
            odom.pose.pose.orientation.x = 0.0
            odom.pose.pose.orientation.y = 0.0
            odom.pose.pose.orientation.z = np.sin(theta / 2)
            odom.pose.pose.orientation.w = np.cos(theta / 2)
            
            # 速度 / Velocity
            odom.twist.twist.linear.x = vx
            odom.twist.twist.linear.y = vy
            odom.twist.twist.angular.z = omega
            
            # 协方差矩阵 / Covariance matrix
            if 'odometry' in self.config and 'pose_covariance' in self.config['odometry']:
                pose_cov = self.config['odometry']['pose_covariance']
                twist_cov = self.config['odometry']['twist_covariance']
                odom.pose.covariance[0] = pose_cov['xx']   # x-x
                odom.pose.covariance[7] = pose_cov['yy']   # y-y
                odom.pose.covariance[35] = pose_cov['tt']  # theta-theta
                odom.twist.covariance[0] = twist_cov['vx']  # vx-vx
                odom.twist.covariance[7] = twist_cov['vy']  # vy-vy
                odom.twist.covariance[35] = twist_cov['omega']  # omega-omega
            else:
                # 使用默认协方差 / Use default covariance
                odom.pose.covariance[0] = 0.001   # x-x
                odom.pose.covariance[7] = 0.001   # y-y
                odom.pose.covariance[35] = 0.01   # theta-theta
                odom.twist.covariance[0] = 0.001  # vx-vx
                odom.twist.covariance[7] = 0.001  # vy-vy
                odom.twist.covariance[35] = 0.01  # omega-omega
            
            self.odom_pub.publish(odom)
            
        except Exception as e:
            self._log_error(f'Failed to publish odometry: {e}')
    
    def get_statistics(self) -> Dict[str, int]:
        """获取统计信息 / Get statistics
        
        Returns:
            统计字典 / Statistics dict
        """
        return {
            'read_count': self.read_count,
            'write_count': self.write_count,
            'read_errors': self.read_errors,
            'write_errors': self.write_errors,
            'read_success_rate': (
                (self.read_count - self.read_errors) / self.read_count 
                if self.read_count > 0 else 0.0
            ),
            'write_success_rate': (
                (self.write_count - self.write_errors) / self.write_count 
                if self.write_count > 0 else 0.0
            )
        }
    
    def _log_info(self, message: str):
        """日志记录 - INFO级别 / Logging - INFO level"""
        if hasattr(self, 'logger') and self.logger:
            self.logger.info(message)
    
    def _log_warning(self, message: str):
        """日志记录 - WARNING级别 / Logging - WARNING level"""
        if hasattr(self, 'logger') and self.logger:
            self.logger.warning(message)
    
    def _log_error(self, message: str):
        """日志记录 - ERROR级别 / Logging - ERROR level"""
        if hasattr(self, 'logger') and self.logger:
            self.logger.error(message)


def main():
    """测试函数 / Test function"""
    print("OmniHardwareInterface - ROS2 Control硬件接口")
    print("=" * 60)
    print("注意 / Note: 此类需要在ros2_control框架中运行")
    print("This class needs to run within ros2_control framework")
    print("=" * 60)
    
    # 创建实例（仅测试构造函数）/ Create instance (test constructor only)
    interface = OmniHardwareInterface()
    
    print(f"✅ OmniHardwareInterface实例创建成功")
    print(f"   Instance created successfully")
    print(f"\n组件状态 / Component Status:")
    print(f"  - config: {interface.config is not None}")
    print(f"  - driver: {interface.driver is not None}")
    print(f"  - encoder_handler: {interface.encoder_handler is not None}")
    print(f"  - velocity_ramp: {interface.velocity_ramp is not None}")
    print(f"  - kinematics: {interface.kinematics is not None}")


if __name__ == '__main__':
    main()
