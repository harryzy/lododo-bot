#!/usr/bin/env python3
"""
EncoderHandler - 编码器数据处理器 / Encoder Data Processor

本模块处理ST3215舵机的12位编码器数据，包括溢出检测、单位转换和速度计算。
This module processes ST3215 servo 12-bit encoder data, including overflow detection,
unit conversion, and velocity calculation.

参考设计文档 / Reference: HARDWARE_DEPLOYMENT_DESIGN.md §3.2.4 Q3 (Line 2315-2591)

核心功能 / Core Features:
- 编码器溢出处理 / Encoder overflow handling (0-4095 circular)
- 周数累积统计 / Revolution count tracking
- None输入容错 / None input fault tolerance (Round 7)
- 首次初始化标志 / First initialization flag (Round 7)
- 异常delta检查 / Abnormal delta check (Round 7)
- 一步速度计算 / One-step velocity calculation (Round 7)

Author: Hurry
Created: 2026-01-19
"""

import numpy as np
from typing import Optional


class EncoderHandler:
    """
    编码器数据处理器类 / Encoder Data Processor Class
    
    Round 7职责定位: 封装完整编码器数据处理逻辑（溢出+转换+速度计算）
    Round 7 responsibility: Encapsulate complete encoder processing (overflow+conversion+velocity)
    """
    
    def __init__(self, encoder_resolution: int = 4096, logger=None):
        """
        初始化编码器处理器 / Initialize encoder handler
        
        Args:
            encoder_resolution: 编码器分辨率 / Encoder resolution (default: 4096 for 12-bit)
            logger: ROS2 logger对象 / ROS2 logger object (optional)
        """
        self.encoder_resolution = encoder_resolution
        self.encoder_max_value = encoder_resolution - 1  # 4095
        self.logger = logger
        
        # 状态变量（支持3个舵机） / State variables (support 3 servos)
        self.last_position = [0, 0, 0]  # 上次编码器位置 / Last encoder position
        self.revolution_count = [0, 0, 0]  # 累积旋转周数 / Accumulated revolution count
        
        # Round 7新增: 初始化状态标志 / Round 7 added: Initialization flags
        self.position_initialized = [False, False, False]  # 是否已初始化 / Initialization status
        
        # Round 7新增: 连续失败计数 / Round 7 added: Consecutive failure counter
        self.consecutive_failures = [0, 0, 0]  # 连续失败次数 / Consecutive failure count
        self.max_consecutive_failures = 5  # 最大允许连续失败次数 / Max allowed failures
        
        # 预计算转换因子（提升性能） / Pre-calculated conversion factor (performance boost)
        self.ticks_to_rad_factor = 2 * np.pi / self.encoder_resolution  # ~0.001534 rad/tick
        
        # 异常delta阈值（容差100 ticks） / Abnormal delta threshold (tolerance 100 ticks)
        # 50Hz采样，轮子最大速度4.71 rad/s → 理论最大delta ~30 ticks
        # At 50Hz, max wheel speed 4.71 rad/s → theoretical max delta ~30 ticks
        self.abnormal_delta_threshold = 100
    
    def get_position_delta(self, servo_id: int, current_position: Optional[int]) -> int:
        """
        计算编码器位置增量（处理溢出） / Calculate encoder position delta with overflow handling
        
        Round 7增强: 处理None输入、首次初始化、异常delta合理性检查
        Round 7 enhancements: Handle None input, first initialization, abnormal delta check
        
        Args:
            servo_id: 舵机ID (0/1/2) / Servo ID (0/1/2)
            current_position: 当前编码器读数(0-4095)，可能为None(通信失败) /
                            Current encoder reading (0-4095), may be None (comm failure)
        
        Returns:
            int: 位置增量(ticks)，异常情况返回0 / Position delta (ticks), 0 on error
        
        Raises:
            RuntimeError: 连续失败超过阈值时抛出异常 / Raised when consecutive failures exceed threshold
        """
        # Round 7新增: None输入处理（防止程序崩溃） / Handle None input (prevent crash)
        if current_position is None:
            self.consecutive_failures[servo_id] += 1
            
            if self.consecutive_failures[servo_id] >= self.max_consecutive_failures:
                error_msg = (
                    f"Servo {servo_id} read failed {self.max_consecutive_failures} "
                    f"consecutive times, check hardware connection"
                )
                if self.logger:
                    self.logger.error(error_msg)
                raise RuntimeError(error_msg)
            
            if self.logger:
                self.logger.warn(
                    f"Servo {servo_id} position is None, "
                    f"consecutive failures: {self.consecutive_failures[servo_id]}"
                )
            
            return 0  # 保持last_position不变，返回0增量 / Keep last_position, return 0 delta
        
        # Round 7新增: 首次初始化处理（避免溢出误判） / First initialization (avoid false overflow)
        if not self.position_initialized[servo_id]:
            self.last_position[servo_id] = current_position
            self.position_initialized[servo_id] = True
            self.consecutive_failures[servo_id] = 0  # 重置失败计数 / Reset failure count
            
            if self.logger:
                self.logger.info(
                    f"Servo {servo_id} encoder initialized at position {current_position}"
                )
            
            return 0  # 首次读取，返回0增量 / First read, return 0 delta
        
        # 重置连续失败计数（读取成功） / Reset failure count (read success)
        self.consecutive_failures[servo_id] = 0
        
        # 计算位置增量 / Calculate position delta
        last_pos = self.last_position[servo_id]
        delta = current_position - last_pos
        
        # 检测正向溢出: 4095 → 0 / Detect forward overflow: 4095 → 0
        # 例: last=4090, current=5, delta=-4085 (< -2048) / Example: last=4090, current=5, delta=-4085
        if delta < -(self.encoder_resolution / 2):
            delta += self.encoder_resolution
            self.revolution_count[servo_id] += 1
            
            if self.logger and self.logger.get_effective_level() <= 10:  # DEBUG level
                self.logger.debug(
                    f"Servo {servo_id} forward overflow detected: "
                    f"{last_pos} → {current_position}, revolution_count={self.revolution_count[servo_id]}"
                )
        
        # 检测反向溢出: 0 → 4095 / Detect backward overflow: 0 → 4095
        # 例: last=5, current=4090, delta=4085 (> 2048) / Example: last=5, current=4090, delta=4085
        elif delta > (self.encoder_resolution / 2):
            delta -= self.encoder_resolution
            self.revolution_count[servo_id] -= 1
            
            if self.logger and self.logger.get_effective_level() <= 10:  # DEBUG level
                self.logger.debug(
                    f"Servo {servo_id} backward overflow detected: "
                    f"{last_pos} → {current_position}, revolution_count={self.revolution_count[servo_id]}"
                )
        
        # Round 7新增: 异常delta合理性检查 / Abnormal delta sanity check
        # 容差100 ticks可覆盖短暂的通信延迟或高速运动
        # Tolerance 100 ticks covers temporary comm delays or high-speed motion
        if abs(delta) > self.abnormal_delta_threshold:
            if self.logger:
                self.logger.warn(
                    f"Servo {servo_id} abnormal delta: {delta} ticks "
                    f"(exceeds {self.abnormal_delta_threshold}), possible packet loss or sudden motion, "
                    f"returning 0 to avoid incorrect integration"
                )
            delta = 0  # 异常delta，返回0避免错误积分 / Return 0 to avoid wrong integration
        
        # 更新上次位置 / Update last position
        self.last_position[servo_id] = current_position
        
        return delta
    
    def ticks_to_radians(self, ticks: float) -> float:
        """
        将编码器ticks转换为弧度 / Convert encoder ticks to radians
        
        使用预计算的转换因子，提升性能 / Use pre-calculated factor for performance
        
        Args:
            ticks: 编码器增量(ticks) / Encoder delta (ticks)
        
        Returns:
            float: 弧度值(radians) / Radians value
        
        Note:
            转换公式: radians = ticks * (2π / encoder_resolution)
            Conversion formula: radians = ticks * (2π / encoder_resolution)
            对于4096分辨率: factor ≈ 0.001534 rad/tick
            For 4096 resolution: factor ≈ 0.001534 rad/tick
        """
        return ticks * self.ticks_to_rad_factor
    
    def get_velocity_rad_s(
        self, 
        servo_id: int, 
        current_position: Optional[int], 
        dt: float
    ) -> float:
        """
        一步计算轮子角速度 / Calculate wheel angular velocity in one step
        
        Round 7新增方法(方案A): 封装完整数据处理逻辑（溢出+转换+速度）
        Round 7 new method (Scheme A): Encapsulate complete processing (overflow+conversion+velocity)
        
        这是推荐的调用方式，简化了OmniHardwareInterface.read()的实现：
        This is the recommended way, simplifying OmniHardwareInterface.read():
        - 旧方式(3步) / Old way (3 steps): 
          delta_ticks = get_position_delta() → delta_rad = ticks_to_radians() → vel = delta_rad / dt
        - 新方式(1步) / New way (1 step): 
          vel = get_velocity_rad_s()
        
        Args:
            servo_id: 舵机ID (0/1/2) / Servo ID (0/1/2)
            current_position: 当前编码器读数(0-4095)，可能为None / 
                            Current encoder reading (0-4095), may be None
            dt: 时间间隔(秒) / Time interval (seconds)
        
        Returns:
            float: 轮子角速度(rad/s) / Wheel angular velocity (rad/s)
        
        Example:
            >>> # In OmniHardwareInterface.read()
            >>> dt = (current_time - self.last_time).nanoseconds * 1e-9
            >>> for i, servo_id in enumerate(self.servo_ids):
            ...     pos = self.driver.read_position(servo_id)  # May return None
            ...     self.hw_velocities[i] = self.encoder_handler.get_velocity_rad_s(i, pos, dt)
        """
        # Step 1: 计算位置增量（内部处理None、溢出、初始化）
        # Calculate position delta (handles None, overflow, initialization internally)
        delta_ticks = self.get_position_delta(servo_id, current_position)
        
        # Step 2: 转换为弧度 / Convert to radians
        delta_rad = self.ticks_to_radians(delta_ticks)
        
        # Step 3: 计算速度（防止除零） / Calculate velocity (prevent division by zero)
        if dt > 0:
            velocity = delta_rad / dt
        else:
            velocity = 0.0
            if self.logger:
                self.logger.warn(f"Servo {servo_id} dt={dt} is invalid, returning 0 velocity")
        
        return velocity
    
    def get_absolute_position(self, servo_id: int, current_position: int) -> int:
        """
        获取绝对位置（包含周数） / Get absolute position with revolution count
        
        ⚠️ **注意**: 此方法仅用于调试和日志记录，不用于里程计计算
        ⚠️ **WARNING**: This method is for debugging/logging only, NOT for odometry calculation
        
        当前里程计实现使用位置增量(delta)进行速度积分，这是正确的设计。
        Current odometry uses position delta for velocity integration, which is correct.
        
        绝对位置方法保留用于 / Absolute position is kept for:
        - 调试时验证轮子是否正常旋转 / Debugging wheel rotation
        - 记录长期运行的累积旋转圈数 / Logging long-term revolution count
        - 故障分析和性能监控 / Fault analysis and performance monitoring
        
        Args:
            servo_id: 舵机ID (0/1/2) / Servo ID (0/1/2)
            current_position: 当前编码器读数(0-4095) / Current encoder reading (0-4095)
        
        Returns:
            int: 绝对位置 = 旋转圈数 * encoder_resolution + 当前位置 /
                 Absolute position = revolution_count * encoder_resolution + current_position
        
        Example:
            >>> # 正向旋转2圈后的位置1000 (假设encoder_resolution=4096)
            >>> # After 2 forward revolutions at position 1000 (assume encoder_resolution=4096)
            >>> # revolution_count = 2, current_position = 1000
            >>> # absolute_position = 2 * 4096 + 1000 = 9192
            >>> handler = EncoderHandler(encoder_resolution=4096)
            >>> # Simulate 2 revolutions...
            >>> abs_pos = handler.get_absolute_position(0, 1000)
            >>> print(abs_pos)  # 9192
        """
        return self.revolution_count[servo_id] * self.encoder_resolution + current_position
    
    def reset(self, servo_id: int):
        """
        重置特定舵机的编码器状态 / Reset encoder state for specific servo
        
        Args:
            servo_id: 舵机ID (0/1/2) / Servo ID (0/1/2)
        
        Note:
            用于故障恢复或重新初始化 / Used for fault recovery or re-initialization
        """
        self.last_position[servo_id] = 0
        self.revolution_count[servo_id] = 0
        self.position_initialized[servo_id] = False
        self.consecutive_failures[servo_id] = 0
        
        if self.logger:
            self.logger.info(f"Servo {servo_id} encoder state reset")
    
    def reset_all(self):
        """
        重置所有舵机的编码器状态 / Reset encoder state for all servos
        
        Note:
            用于系统重启或全局故障恢复 / Used for system restart or global fault recovery
        """
        for i in range(3):
            self.reset(i)
        
        if self.logger:
            self.logger.info("All encoder states reset")
    
    def get_statistics(self, servo_id: int) -> dict:
        """
        获取统计信息 / Get statistics
        
        Args:
            servo_id: 舵机ID (0/1/2) / Servo ID (0/1/2)
        
        Returns:
            dict: 统计信息字典 / Statistics dictionary
        """
        return {
            'servo_id': servo_id,
            'initialized': self.position_initialized[servo_id],
            'revolution_count': self.revolution_count[servo_id],
            'last_position': self.last_position[servo_id],
            'consecutive_failures': self.consecutive_failures[servo_id]
        }


def main():
    """
    测试函数 / Test function
    """
    print("EncoderHandler Test / EncoderHandler测试")
    print("=" * 60)
    
    # 创建EncoderHandler实例 / Create EncoderHandler instance
    handler = EncoderHandler(encoder_resolution=4096)
    
    # 测试1: 正常增量 / Test 1: Normal delta
    print("\n1. Normal Delta Test / 正常增量测试:")
    positions = [100, 150, 200, 250]
    for i, pos in enumerate(positions):
        delta = handler.get_position_delta(0, pos)
        print(f"   Step {i+1}: position={pos}, delta={delta}")
    
    # 测试2: 正向溢出 / Test 2: Forward overflow
    print("\n2. Forward Overflow Test / 正向溢出测试:")
    handler.reset(1)
    overflow_positions = [4000, 4090, 4095, 5, 50]
    for i, pos in enumerate(overflow_positions):
        delta = handler.get_position_delta(1, pos)
        rev_count = handler.revolution_count[1]
        print(f"   Step {i+1}: position={pos}, delta={delta}, revolutions={rev_count}")
    
    # 测试3: None输入处理 / Test 3: None input handling
    print("\n3. None Input Handling Test / None输入处理测试:")
    handler.reset(2)
    handler.get_position_delta(2, 100)  # 初始化
    for i in range(3):
        delta = handler.get_position_delta(2, None)
        failures = handler.consecutive_failures[2]
        print(f"   Attempt {i+1}: delta={delta}, consecutive_failures={failures}")
    
    # 测试4: get_velocity_rad_s方法 / Test 4: get_velocity_rad_s method
    print("\n4. Velocity Calculation Test / 速度计算测试:")
    handler.reset(0)
    handler.get_position_delta(0, 0)  # 初始化
    dt = 0.02  # 50Hz
    positions_for_vel = [0, 30, 60, 90]  # 每次增量30 ticks
    for pos in positions_for_vel:
        vel = handler.get_velocity_rad_s(0, pos, dt)
        print(f"   Position={pos}, velocity={vel:.3f} rad/s")
    
    # 测试5: 统计信息 / Test 5: Statistics
    print("\n5. Statistics / 统计信息:")
    for servo_id in range(3):
        stats = handler.get_statistics(servo_id)
        print(f"   Servo {servo_id}: {stats}")
    
    print("\n" + "=" * 60)
    print("Test completed / 测试完成")


if __name__ == '__main__':
    main()
