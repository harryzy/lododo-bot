#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
OmniHardwareNode - 独立ROS2硬件控制节点
Standalone ROS2 Hardware Control Node

⚠️ 架构决策 (2026-01-19): 不使用ros2_control框架
Architecture Decision: Not using ros2_control framework

功能特性 / Features:
- 独立rclpy.Node实现 / Standalone rclpy.Node implementation
- 订阅/cmd_vel，发布/wheel/odom和/tf / Subscribe /cmd_vel, publish /wheel/odom and /tf
- 50Hz控制循环（Timer）/ 50Hz control loop (Timer)
- 内部调用OmniHardwareInterface核心逻辑 / Uses OmniHardwareInterface internally

设计参考 / Design Reference:
- 主设计: HARDWARE_DEPLOYMENT_DESIGN.md §3.2.5 (行2900-3200)
- 架构决策: ARCHITECTURE_DECISION_STANDALONE_NODE.md
- 实现路线: IMPLEMENTATION_ROADMAP.md P2.6 (v2.0)
"""

import time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from std_msgs.msg import Float64MultiArray
from tf2_ros import TransformBroadcaster
import yaml
import os
import sys
import numpy as np
from ament_index_python.packages import get_package_share_directory

# 为了避免循环导入，我们需要直接导入P1组件而非OmniHardwareInterface
from bot_hardware.drivers.st3215_driver import ST3215Driver
from bot_hardware.utils.encoder_handler import EncoderHandler
from bot_hardware.utils.velocity_ramp import VelocityRamp
from bot_hardware.utils.omni_kinematics import OmniKinematics
from bot_hardware.utils.servo_health_monitor import ServoHealthMonitor
from bot_hardware.utils.path_manager import PathManager


class OmniHardwareNode(Node):
    """全向轮硬件控制节点 / Omnidirectional hardware control node
    
    架构: 独立ROS2节点，直接集成P1组件
    频率: 50Hz控制循环（Timer）
    
    生命周期 / Lifecycle:
    1. __init__(): 加载配置 → 初始化组件 → 创建ROS2接口
    2. control_loop(): 50Hz Timer回调 (write → read → publish)
    3. destroy_node(): 清理硬件资源
    """
    
    def __init__(self):
        super().__init__('omni_hardware_node')
        
        self.get_logger().info('=== OmniHardwareNode Starting ===')
        
        # ============ 1. 加载配置文件 / Load configuration ============
        self.declare_parameter('config_file', '')
        config_file = self.get_parameter('config_file').get_parameter_value().string_value
        
        if not config_file:
            # 默认配置文件路径 / Default config file path
            config_file = os.path.join(
                get_package_share_directory('bot_hardware'),
                'config', 'hardware_config.yaml'
            )
        
        if not os.path.exists(config_file):
            self.get_logger().fatal(f'Configuration file not found: {config_file}')
            raise FileNotFoundError(f'Config file not found: {config_file}')
        
        with open(config_file, 'r') as f:
            self.config = yaml.safe_load(f)
        
        self.get_logger().info(f'Configuration loaded from: {config_file}')
        
        # 路径管理器 / Path manager
        self.path_manager = PathManager(self.get_logger())
        
        # ============ 2. 初始化P1组件 / Initialize P1 components ============
        self._initialize_hardware()
        
        # ============ 3. 创建ROS2接口 / Create ROS2 interfaces ============
        # 订阅/cmd_vel / Subscribe to /cmd_vel
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            qos_profile
        )
        
        # 订阅/wheel/direct_speeds用于直接控制轮子速度（测试用）
        # Subscribe to /wheel/direct_speeds for direct wheel speed control (testing)
        self.direct_speeds_sub = self.create_subscription(
            Float64MultiArray,
            '/wheel/direct_speeds',
            self.direct_speeds_callback,
            qos_profile
        )
        
        # 发布/wheel/odom / Publish /wheel/odom
        self.odom_pub = self.create_publisher(
            Odometry,
            '/wheel/odom',
            10
        )
        
        # 发布TF / Publish TF
        self.tf_broadcaster = TransformBroadcaster(self)
        
        # ============ 4. 控制循环状态 / Control loop state ============
        self.current_cmd = {
            'vx': 0.0,
            'vy': 0.0,
            'omega': 0.0
        }
        
        # 直接轮子速度控制模式标志 / Direct wheel speed control mode flag
        self.use_direct_speeds = False
        self.direct_wheel_speeds = [0.0, 0.0, 0.0]  # rad/s
        
        # 硬件状态 / Hardware state
        # ⚠️ CRITICAL FIX: 初始位姿设置为非零值，避免RTABMap identity pose检测
        # RTABMap会在检测到(0,0,0)位姿时触发"Odometry is reset"并重置地图
        # 使用极小的非零初始值（0.001m, 0.001m, 0.0001rad）避免触发identity检测
        # Initial pose set to non-zero to avoid RTABMap identity pose detection
        self.pose = np.array([0.001, 0.001, 0.0001])  # [x, y, theta] - small non-zero initial offset
        self.last_time = None
        self.last_command_time = None  # 看门狗超时检测
        self.watchdog_triggered = False  # 看门狗状态标志（防止重复警告）
        
        # ============ 5. 创建50Hz控制循环 / Create 50Hz control loop ============
        control_rate = self.config.get('ros2_control', {}).get('update_rate', 50)  # 50Hz
        timer_period = 1.0 / control_rate  # 0.02s
        self.control_timer = self.create_timer(timer_period, self.control_loop)
        
        self.get_logger().info(f'Control loop started at {control_rate}Hz')
        self.get_logger().info('=== OmniHardwareNode Initialized Successfully ===')
    
    def _initialize_hardware(self):
        """初始化硬件组件 / Initialize hardware components
        
        ⚠️ Round 7修订: 正确的初始化顺序
        Correct initialization order:
        1. driver (无依赖)
        2. kinematics (无依赖)
        3. encoder_handler (依赖: config)
        4. 读取当前速度 (依赖: driver, encoder_handler, kinematics)
        5. velocity_ramp (依赖: current_velocity)
        6. servo_health_monitor (依赖: velocity_ramp)
        7. 启动监控 (最后)
        """
        try:
            # 从配置读取舵机ID / Read servo IDs from config
            self.servo_ids = [
                self.config['servo']['wheel_1_id'],
                self.config['servo']['wheel_2_id'],
                self.config['servo']['wheel_3_id']
            ]
            self.get_logger().info(f'Servo IDs configured: {self.servo_ids}')
            
            # 1. 初始化舵机驱动 / Initialize servo driver
            self.driver = ST3215Driver(
                port=self.config['serial']['servo_port'],
                baudrate=self.config['serial']['servo_baudrate'],
                retry_delay=self.config['serial']['servo_timeout'],
                retry_count=self.config['serial']['servo_retry_times'],
                servo_ids=self.servo_ids,
                logger=self.get_logger()  # 传入logger / Pass logger
            )
            self.get_logger().info('ST3215 driver created')
            
            # 初始化串口并扫描舵机（内部会设置轮子模式）/ Initialize serial port and scan servos
            if not self.driver.initialize():
                raise RuntimeError('Failed to initialize ST3215 driver')
            self.get_logger().info('All servos online and set to wheel mode')
            
            # 2. 初始化运动学工具 / Initialize kinematics tool
            self.kinematics = OmniKinematics(self.config)
            self.get_logger().info('OmniKinematics initialized')
            
            # 3. 初始化编码器处理器 / Initialize encoder handler
            encoder_resolution = self.config['servo']['encoder_resolution']
            self.encoder_handler = EncoderHandler(
                encoder_resolution=encoder_resolution,
                logger=self.get_logger()
            )
            self.get_logger().info('EncoderHandler initialized')
            
            # 4. 读取当前速度 / Read current velocity
            current_wheel_velocities = self._read_current_wheel_velocities()
            if current_wheel_velocities is None:
                self.get_logger().error('Failed to initialize velocity, using zero velocity')
                current_wheel_velocities = [0.0, 0.0, 0.0]
            
            current_robot_velocity = self.kinematics.forward_kinematics(
                current_wheel_velocities[0],
                current_wheel_velocities[1],
                current_wheel_velocities[2]
            )
            self.get_logger().info(
                f'Current robot velocity: vx={current_robot_velocity[0]:.3f} m/s, '
                f'vy={current_robot_velocity[1]:.3f} m/s, '
                f'omega={current_robot_velocity[2]:.3f} rad/s'
            )
            
            # 5. 初始化VelocityRamp / Initialize VelocityRamp
            self.velocity_ramp = VelocityRamp(self.config, logger=self.get_logger())
            self.velocity_ramp.last_linear_velocity = np.array([
                current_robot_velocity[0], 
                current_robot_velocity[1]
            ])
            self.velocity_ramp.last_angular_velocity = current_robot_velocity[2]
            self.get_logger().info('VelocityRamp initialized with current velocity')
            
            # 6. 初始化ServoHealthMonitor / Initialize ServoHealthMonitor
            self.servo_health = ServoHealthMonitor(
                self.driver, 
                self.config,
                velocity_ramp=self.velocity_ramp
            )
            self.servo_health.set_logger(self.get_logger())  # 注入logger
            self.get_logger().info('ServoHealthMonitor initialized')
            
            # 7. 启动健康监控 / Start health monitoring
            self.servo_health.start()
            self.get_logger().info('Health monitoring started')
            
        except Exception as e:
            self.get_logger().fatal(f'Failed to initialize hardware: {e}')
            raise
    
    def _read_current_wheel_velocities(self):
        """从编码器读取当前轮子速度 / Read current wheel velocities
        
        ⚠️ Round 7完整实现: 包含预初始化步骤
        参见: HARDWARE_DEPLOYMENT_DESIGN.md §3.2.4 Q4
        
        Returns:
            list: [w1, w2, w3] in rad/s, or None on failure
        """
        try:
            import time
            
            # Step 0: 预初始化EncoderHandler基准位置
            self.get_logger().info('Initializing EncoderHandler baseline positions...')
            for i, servo_id in enumerate(self.servo_ids):
                pos = self.driver.read_position(servo_id)
                if pos is None:
                    raise RuntimeError(f'Failed to read initial position from servo {servo_id}')
                self.encoder_handler.last_position[i] = pos
                self.encoder_handler.position_initialized[i] = True
            
            self.get_logger().info(
                f'EncoderHandler baseline: {self.encoder_handler.last_position}'
            )
            
            # Step 1: 精确计时等待
            start_time = time.time()
            time.sleep(0.02)  # 20ms采样间隔
            actual_dt = time.time() - start_time
            
            # Step 2: 第二次读取，计算速度
            wheel_velocities = []
            for i, servo_id in enumerate(self.servo_ids):
                pos = self.driver.read_position(servo_id)
                if pos is None:
                    raise RuntimeError(f'Failed to read position from servo {servo_id}')
                
                # 使用Round 7新增的get_velocity_rad_s()方法
                vel = self.encoder_handler.get_velocity_rad_s(i, pos, actual_dt)
                wheel_velocities.append(vel)
            
            self.get_logger().info(
                f'Current wheel velocities: [{wheel_velocities[0]:.3f}, '
                f'{wheel_velocities[1]:.3f}, {wheel_velocities[2]:.3f}] rad/s '
                f'(dt={actual_dt*1000:.1f}ms)'
            )
            
            return wheel_velocities
            
        except Exception as e:
            self.get_logger().error(f'Failed to initialize wheel velocities: {e}')
            return None
    
    def cmd_vel_callback(self, msg):
        """接收/cmd_vel指令 / Receive /cmd_vel command
        
        Args:
            msg: geometry_msgs/Twist消息
        """
        self.current_cmd['vx'] = msg.linear.x
        self.current_cmd['vy'] = msg.linear.y
        self.current_cmd['omega'] = msg.angular.z
        
        # 切换到cmd_vel控制模式 / Switch to cmd_vel control mode
        self.use_direct_speeds = False
        
        # 记录最后命令时间（用于看门狗）
        self.last_command_time = time.time()
    
    def direct_speeds_callback(self, msg):
        """接收直接轮子速度指令 / Receive direct wheel speed command
        
        Args:
            msg: std_msgs/Float64MultiArray，数据格式: [wheel1_rad/s, wheel2_rad/s, wheel3_rad/s]
        """
        if len(msg.data) != 3:
            self.get_logger().warn(f'Invalid direct_speeds message: expected 3 values, got {len(msg.data)}')
            return
        
        self.direct_wheel_speeds = list(msg.data)
        self.use_direct_speeds = True
        
        # 记录最后命令时间（用于看门狗）
        self.last_command_time = time.time()
        
        self.get_logger().info(
            f'Direct wheel speeds: [{self.direct_wheel_speeds[0]:.2f}, '
            f'{self.direct_wheel_speeds[1]:.2f}, {self.direct_wheel_speeds[2]:.2f}] rad/s'
        )
    
    def control_loop(self):
        """50Hz控制循环 / 50Hz control loop
        
        执行流程:
        1. write(): 发送当前cmd_vel到硬件
        2. read(): 读取编码器，计算里程计
        3. publish: 发布/wheel/odom和/tf
        """
        try:
            current_time = time.time()
            
            # ============ 1. Write: 发送速度指令 / Send velocity command ============
            # 看门狗超时检测（默认2.0s无指令则置零）
            timeout = self.config.get('safety', {}).get('watchdog_timeout', 2.0)
            if self.last_command_time is None or (current_time - self.last_command_time) > timeout:
                # 看门狗超时：清零速度命令
                # 只在当前命令非零且首次超时时打印警告
                if self.last_command_time is not None and not self.watchdog_triggered:
                    # 检查当前命令是否非零 / Check if current command is non-zero
                    is_moving = (abs(self.current_cmd['vx']) > 0.001 or 
                                abs(self.current_cmd['vy']) > 0.001 or 
                                abs(self.current_cmd['omega']) > 0.001)
                    if is_moving:
                        # 只在从运动状态超时时警告（0→0的超时不需要警告）
                        # Only warn when timeout from moving state (0→0 timeout doesn't need warning)
                        self.get_logger().warn('Watchdog timeout, zeroing velocity commands')
                    self.watchdog_triggered = True
                target_vx, target_vy, target_omega = 0.0, 0.0, 0.0
                self.use_direct_speeds = False  # 超时时退出直接控制模式
            else:
                # 收到有效命令：重置看门狗标志
                if self.watchdog_triggered:
                    self.get_logger().info('Watchdog reset, resuming normal operation')
                    self.watchdog_triggered = False
                target_vx = self.current_cmd['vx']
                target_vy = self.current_cmd['vy']
                target_omega = self.current_cmd['omega']
            
            # 根据控制模式选择速度来源 / Select velocity source based on control mode
            if self.use_direct_speeds:
                # 直接轮子速度控制模式 / Direct wheel speed control mode
                wheel_velocities = self.direct_wheel_speeds
            else:
                # 正常cmd_vel控制模式 / Normal cmd_vel control mode
                # 速度斜坡限制 / Velocity ramp limiting
                limited_vx, limited_vy, limited_omega = self.velocity_ramp.limit(
                    target_vx, target_vy, target_omega, current_time
                )
                
                # 逆向运动学 / Inverse kinematics
                wheel_velocities = self.kinematics.inverse_kinematics(
                    limited_vx, limited_vy, limited_omega
                )
            
            # 转换为RPM并写入舵机 / Convert to RPM and write to servos
            if self.use_direct_speeds and (int(current_time * 2) % 10 == 0):  # 每5秒打印一次
                rpm_values = [wheel_velocities[i] * 30 / np.pi for i in range(3)]
                self.get_logger().debug(
                    f'[DEBUG] Sending RPM: [{rpm_values[0]:.1f}, {rpm_values[1]:.1f}, {rpm_values[2]:.1f}] to servos [{self.servo_ids[0]}, {self.servo_ids[1]}, {self.servo_ids[2]}]'
                )
            
            for i, wheel_vel in enumerate(wheel_velocities):
                rpm = wheel_vel * 30 / np.pi  # rad/s → RPM
                servo_id = self.servo_ids[i]
                
                success = self.driver.write_speed(servo_id, rpm)
                if not success:
                    self.get_logger().warn(f'Failed to write speed to servo {servo_id}')
            
            # ============ 2. Read: 读取硬件状态 / Read hardware state ============
            dt = current_time - self.last_time if self.last_time else 0.02
            
            # 读取编码器位置 / Read encoder positions
            encoder_positions = []
            for servo_id in self.servo_ids:
                pos = self.driver.read_position(servo_id)
                if pos is None:
                    self.get_logger().warn(f'Failed to read servo {servo_id} position')
                    return
                encoder_positions.append(pos)
            
            # 处理编码器溢出+计算速度 / Handle overflow + calculate velocity
            wheel_velocities_read = []
            for i, (servo_id, current_pos) in enumerate(zip(self.servo_ids, encoder_positions)):
                vel = self.encoder_handler.get_velocity_rad_s(i, current_pos, dt)
                wheel_velocities_read.append(vel)
            
            # 调试：打印编码器和速度信息（仅在直接控制模式下）/ Debug: print encoder and velocity info
            if self.use_direct_speeds and (int(current_time * 2) % 10 == 0):  # 每5秒打印一次
                self.get_logger().debug(
                    f'[DEBUG] Encoders: [{encoder_positions[0]:.1f}, {encoder_positions[1]:.1f}, {encoder_positions[2]:.1f}] | '
                    f'Velocities: [{wheel_velocities_read[0]:.2f}, {wheel_velocities_read[1]:.2f}, {wheel_velocities_read[2]:.2f}] rad/s'
                )
            
            # 正向运动学 / Forward kinematics
            vx, vy, omega = self.kinematics.forward_kinematics(
                wheel_velocities_read[0],
                wheel_velocities_read[1],
                wheel_velocities_read[2]
            )
            
            # 更新位姿 / Update pose (简单欧拉积分)
            self.pose[0] += vx * dt  # x
            self.pose[1] += vy * dt  # y
            self.pose[2] += omega * dt  # theta
            
            self.last_time = current_time
            
            # ============ 3. Publish: 发布里程计和TF / Publish odom and TF ============
            self.publish_odometry(vx, vy, omega, current_time)
            self.publish_tf(current_time)
            
        except Exception as e:
            self.get_logger().error(f'Control loop error: {e}', throttle_duration_sec=1.0)
    
    def publish_odometry(self, vx, vy, omega, timestamp):
        """发布里程计消息 / Publish odometry message
        
        Args:
            vx, vy, omega: 当前速度
            timestamp: 时间戳
        """
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = self.config['odometry']['frame_id']  # 'odom'
        odom.child_frame_id = self.config['odometry']['child_frame_id']  # 'base_link'
        
        # 位置 / Position
        odom.pose.pose.position.x = self.pose[0]
        odom.pose.pose.position.y = self.pose[1]
        odom.pose.pose.position.z = 0.0
        
        # 姿态 (四元数) / Orientation (quaternion)
        from tf_transformations import quaternion_from_euler
        q = quaternion_from_euler(0, 0, self.pose[2])
        odom.pose.pose.orientation.x = q[0]
        odom.pose.pose.orientation.y = q[1]
        odom.pose.pose.orientation.z = q[2]
        odom.pose.pose.orientation.w = q[3]
        
        # 速度 / Velocity
        odom.twist.twist.linear.x = vx
        odom.twist.twist.linear.y = vy
        odom.twist.twist.angular.z = omega
        
        # 协方差矩阵 / Covariance matrix
        pose_cov = self.config['odometry']['pose_covariance']
        twist_cov = self.config['odometry']['twist_covariance']
        odom.pose.covariance[0] = pose_cov['xx']
        odom.pose.covariance[7] = pose_cov['yy']
        odom.pose.covariance[35] = pose_cov['tt']
        odom.twist.covariance[0] = twist_cov['xx']
        odom.twist.covariance[7] = twist_cov['yy']
        odom.twist.covariance[35] = twist_cov['tt']
        
        self.odom_pub.publish(odom)
    
    def publish_tf(self, timestamp):
        """发布TF变换 / Publish TF transform
        
        Args:
            timestamp: 时间戳
        """
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = self.config['odometry']['frame_id']
        t.child_frame_id = self.config['odometry']['child_frame_id']
        
        # 平移 / Translation
        t.transform.translation.x = self.pose[0]
        t.transform.translation.y = self.pose[1]
        t.transform.translation.z = 0.0
        
        # 旋转 / Rotation
        from tf_transformations import quaternion_from_euler
        q = quaternion_from_euler(0, 0, self.pose[2])
        t.transform.rotation.x = q[0]
        t.transform.rotation.y = q[1]
        t.transform.rotation.z = q[2]
        t.transform.rotation.w = q[3]
        
        self.tf_broadcaster.sendTransform(t)
    
    def destroy_node(self):
        """节点销毁清理 / Node cleanup"""
        self.get_logger().info('Shutting down omni_hardware_node...')
        
        # 停止健康监控 / Stop health monitoring
        if hasattr(self, 'servo_health') and self.servo_health:
            self.servo_health.stop()
        
        # 停止所有舵机 / Stop all servos
        if hasattr(self, 'driver') and self.driver:
            for servo_id in self.servo_ids:
                self.driver.write_speed(servo_id, 0.0)
            self.get_logger().info('All servos stopped')
        
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = OmniHardwareNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'Fatal error: {e}')
        import traceback
        traceback.print_exc()
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
