#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
VelocityRamp - 速度斜坡限制器（带急停功能）
Velocity Ramp Limiter with Emergency Stop

功能特性 / Features:
- 分别限制线速度和角速度的加速度 / Separate linear and angular acceleration limiting
- 急停功能 + 自动恢复机制 / Emergency stop with auto-recovery
- 速度突变检测 / Velocity jump detection
- 可配置的冷却期 / Configurable cooldown period

设计参考 / Design Reference:
- 主设计: HARDWARE_DEPLOYMENT_DESIGN.md §3.2.4 Q4 (行2411-2591)
- 急停增强: §3.1.4 Q5 (行1961-2209)
- Round 6修订: 统一使用config对象传递参数
"""

import numpy as np
from typing import Tuple, Optional

class VelocityRamp:
    """速度斜坡限制器（带急停功能）/ Velocity ramp limiter with emergency stop"""
    
    def __init__(self, config: dict, logger=None):
        """初始化速度斜坡限制器 / Initialize velocity ramp limiter
        
        Args:
            config: 完整的hardware_config.yaml配置字典 / Complete hardware config dict
            logger: ROS2 logger对象（可选）/ ROS2 logger object (optional)
        
        Round 6修订: 统一使用config对象传递，读取所有参数
        """
        self.logger = logger
        
        # 读取运动参数（从kinematics配置中读取）/ Read motion parameters from kinematics config
        kinematics_cfg = config['kinematics']
        
        # 加速度限制 / Acceleration limits
        self.max_linear_accel = kinematics_cfg['max_linear_acceleration']    # 0.5 m/s²
        self.max_linear_decel = kinematics_cfg.get('max_linear_deceleration', 
                                                 self.max_linear_accel)   # 默认同加速度 / Default same as accel
        self.max_angular_accel = kinematics_cfg['max_angular_acceleration']  # 1.0 rad/s²
        self.max_angular_decel = kinematics_cfg.get('max_angular_deceleration',
                                                  self.max_angular_accel) # 默认同加速度 / Default same as accel
        
        # 速度限制 / Velocity limits
        self.max_linear_velocity = max(
            kinematics_cfg.get('max_linear_velocity_x', 0.5),
            kinematics_cfg.get('max_linear_velocity_y', 0.5)
        )  # 取x和y方向最大值
        self.max_angular_velocity = kinematics_cfg['max_angular_velocity']   # 1.0 rad/s
        
        # 状态变量 / State variables
        self.last_linear_velocity = np.array([0.0, 0.0])  # [vx, vy]
        self.last_angular_velocity = 0.0  # omega_z
        self.last_time: Optional[float] = None  # 时间戳(秒) / Timestamp (seconds)
        
        # ✅ 从配置读取VelocityRamp高级选项 / Read VelocityRamp advanced options
        vr_cfg = kinematics_cfg.get('velocity_ramp', {})
        self.enable = vr_cfg.get('enable', True)
        
        # 速度突变检测阈值 / Velocity jump detection threshold
        self.velocity_jump_threshold = vr_cfg.get('velocity_jump_threshold', 0.3)  # m/s
        
        # 急停配置 / Emergency stop configuration
        es_cfg = vr_cfg.get('emergency_stop', {})
        self.enable_emergency_stop = es_cfg.get('enable', True)
        self.auto_recovery_enabled = not es_cfg.get('manual_recovery', False)  # 默认自动恢复 / Auto-recovery by default
        self.cooldown_duration = es_cfg.get('auto_recovery_time', 15.0)  # 默认15秒 / Default 15s
        
        # 急停状态变量 / Emergency stop state
        self.is_emergency_stopped = False
        self.emergency_stop_time: Optional[float] = None
        self.emergency_stop_reason: Optional[str] = None
        
        # 日志配置 / Logging configuration
        self.log_velocity_changes = vr_cfg.get('log_velocity_changes', False)
        
        if self.logger:
            self.logger.info(
                f'VelocityRamp initialized: '
                f'linear_accel={self.max_linear_accel:.2f} m/s², '
                f'angular_accel={self.max_angular_accel:.2f} rad/s², '
                f'emergency_stop={self.enable_emergency_stop}, '
                f'cooldown={self.cooldown_duration:.1f}s'
            )
    
    def emergency_stop(self, reason: str):
        """触发急停 / Trigger emergency stop
        
        Args:
            reason: 急停原因（用于日志）/ Emergency stop reason (for logging)
        
        设计要点 / Design Points:
        - 立即停止所有运动（后续limit()调用返回零速度）
        - 记录急停时间用于自动恢复倒计时
        - 使用CRITICAL级别日志（参考§1.4.2，行906-1006）
        """
        if not self.enable_emergency_stop:
            return
        
        import time
        self.is_emergency_stopped = True
        self.emergency_stop_time = time.time()
        self.emergency_stop_reason = reason
        
        # 重置速度状态 / Reset velocity state
        self.last_linear_velocity = np.array([0.0, 0.0])
        self.last_angular_velocity = 0.0
        
        if self.logger:
            self.logger.error(f'EMERGENCY STOP triggered: {reason}')
            if self.auto_recovery_enabled:
                self.logger.warn(
                    f'Auto-recovery will activate after {self.cooldown_duration:.1f}s cooldown'
                )
            else:
                self.logger.warn('Manual recovery required (call recover_from_emergency())')
    
    def check_recovery(self):
        """检查是否可以从急停恢复 / Check if recovery from emergency stop is possible
        
        自动恢复逻辑 / Auto-recovery logic:
        - 仅在auto_recovery_enabled=true时自动恢复
        - 冷却期结束后自动解除急停状态
        - 日志记录恢复事件
        """
        if not self.is_emergency_stopped:
            return
        
        if not self.auto_recovery_enabled:
            return  # 需要手动恢复 / Manual recovery required
        
        import time
        elapsed = time.time() - self.emergency_stop_time
        if elapsed >= self.cooldown_duration:
            self.is_emergency_stopped = False
            self.emergency_stop_reason = None
            if self.logger:
                self.logger.info(
                    f'Emergency stop recovered after {elapsed:.1f}s cooldown, '
                    'normal control resumed'
                )
    
    def recover_from_emergency(self):
        """手动恢复急停状态 / Manually recover from emergency stop
        
        用于manual_recovery=true时的手动恢复操作
        """
        if self.is_emergency_stopped:
            self.is_emergency_stopped = False
            self.emergency_stop_reason = None
            if self.logger:
                self.logger.info('Emergency stop manually recovered')
    
    def limit(
        self, 
        target_vx: float, 
        target_vy: float, 
        target_omega: float, 
        current_time: float  # 改为接受float时间戳 / Changed to accept float timestamp
    ) -> Tuple[float, float, float]:
        """限制速度变化率 / Limit velocity change rate
        
        Args:
            target_vx: 目标x方向线速度 / Target linear velocity x (m/s)
            target_vy: 目标y方向线速度 / Target linear velocity y (m/s)
            target_omega: 目标角速度 / Target angular velocity (rad/s)
            current_time: 当前时间戳(秒) / Current timestamp (seconds, from time.time())
        
        Returns:
            (limited_vx, limited_vy, limited_omega): 限制后的速度
        
        算法逻辑 / Algorithm Logic:
        1. 检查急停状态（返回零速度）
        2. 检查速度突变（触发急停）
        3. 分别限制线速度和角速度的加速度
        4. 应用速度上限
        """
        # Step 1: 检查是否可以恢复 / Check recovery
        self.check_recovery()
        
        # Step 2: 如果处于急停状态，拒绝所有速度指令 / Reject all commands during emergency stop
        if self.is_emergency_stopped:
            return 0.0, 0.0, 0.0
        
        # Step 3: 如果未启用，直接返回目标速度 / If disabled, return target directly
        if not self.enable:
            return target_vx, target_vy, target_omega
        
        # Step 4: 初始化时直接返回 / Return directly on first call
        if self.last_time is None:
            self.last_time = current_time
            self.last_linear_velocity = np.array([target_vx, target_vy])
            self.last_angular_velocity = target_omega
            return target_vx, target_vy, target_omega
        
        # Step 5: 计算时间增量 / Calculate time delta
        dt = current_time - self.last_time
        if dt <= 0:
            return self.last_linear_velocity[0], self.last_linear_velocity[1], self.last_angular_velocity
        
        # Step 6: 检测速度突变（从静止突然跳变到高速）/ Detect velocity jump
        current_speed = np.linalg.norm(self.last_linear_velocity)
        target_speed = np.linalg.norm([target_vx, target_vy])
        speed_jump = abs(target_speed - current_speed)
        
        if current_speed < 0.01 and speed_jump > self.velocity_jump_threshold:
            # 检测到从静止状态突然跳变 / Detected jump from stationary
            if self.logger and self.log_velocity_changes:
                self.logger.warn(
                    f'Velocity jump detected: {current_speed:.3f} → {target_speed:.3f} m/s '
                    f'(threshold: {self.velocity_jump_threshold:.2f} m/s), applying ramp'
                )
        
        # Step 7: 限制线速度 / Limit linear velocity
        target_linear = np.array([target_vx, target_vy])
        delta_linear = target_linear - self.last_linear_velocity
        delta_linear_norm = np.linalg.norm(delta_linear)
        
        if delta_linear_norm > 0:
            # 根据加速/减速选择限制 / Choose limit based on acceleration/deceleration
            if delta_linear_norm > 0:  # 加速或减速 / Accelerating or decelerating
                # 检测是加速还是减速 / Check if accelerating or decelerating
                dot_product = np.dot(delta_linear, self.last_linear_velocity)
                if dot_product >= 0:  # 加速 / Accelerating
                    max_delta = self.max_linear_accel * dt
                else:  # 减速 / Decelerating
                    max_delta = self.max_linear_decel * dt
                
                if delta_linear_norm > max_delta:
                    delta_linear = delta_linear * (max_delta / delta_linear_norm)
        
        limited_linear = self.last_linear_velocity + delta_linear
        
        # 应用速度上限 / Apply velocity limit
        limited_linear_norm = np.linalg.norm(limited_linear)
        if limited_linear_norm > self.max_linear_velocity:
            limited_linear = limited_linear * (self.max_linear_velocity / limited_linear_norm)
        
        # Step 8: 限制角速度 / Limit angular velocity
        delta_omega = target_omega - self.last_angular_velocity
        
        # 根据加速/减速选择限制 / Choose limit based on acceleration/deceleration
        if delta_omega * self.last_angular_velocity >= 0:  # 同向加速 / Same direction (accelerating)
            max_delta_omega = self.max_angular_accel * dt
        else:  # 反向减速 / Opposite direction (decelerating)
            max_delta_omega = self.max_angular_decel * dt
        
        if abs(delta_omega) > max_delta_omega:
            delta_omega = np.sign(delta_omega) * max_delta_omega
        
        limited_omega = self.last_angular_velocity + delta_omega
        
        # 应用角速度上限 / Apply angular velocity limit
        if abs(limited_omega) > self.max_angular_velocity:
            limited_omega = np.sign(limited_omega) * self.max_angular_velocity
        
        # Step 9: 更新状态 / Update state
        self.last_linear_velocity = limited_linear
        self.last_angular_velocity = limited_omega
        self.last_time = current_time
        
        # Step 10: 日志记录（可选）/ Logging (optional)
        if self.logger and self.log_velocity_changes:
            self.logger.debug(
                f'VelocityRamp: target=({target_vx:.3f}, {target_vy:.3f}, {target_omega:.3f}) → '
                f'limited=({limited_linear[0]:.3f}, {limited_linear[1]:.3f}, {limited_omega:.3f})'
            )
        
        return limited_linear[0], limited_linear[1], limited_omega
    
    def reset(self):
        """重置状态变量 / Reset state variables
        
        用于节点重启或状态清零场景
        """
        self.last_linear_velocity = np.array([0.0, 0.0])
        self.last_angular_velocity = 0.0
        self.last_time = None
        
        # 不重置急停状态（急停需要明确恢复）/ Don't reset emergency stop (requires explicit recovery)
        if self.logger:
            self.logger.info('VelocityRamp state reset')
    
    def get_status(self) -> dict:
        """获取状态信息 / Get status information
        
        Returns:
            状态字典，包含当前速度、急停状态等 / Status dict with current velocity, emergency stop state, etc.
        """
        status = {
            'enabled': self.enable,
            'last_linear_velocity': self.last_linear_velocity.tolist(),
            'last_angular_velocity': self.last_angular_velocity,
            'is_emergency_stopped': self.is_emergency_stopped,
        }
        
        if self.is_emergency_stopped:
            import time
            elapsed = time.time() - self.emergency_stop_time
            status['emergency_stop_reason'] = self.emergency_stop_reason
            status['emergency_elapsed_time'] = elapsed
            status['emergency_remaining_time'] = max(0, self.cooldown_duration - elapsed)
        
        return status


def main():
    """测试函数 / Test function"""
    print("VelocityRamp - 速度斜坡限制器测试")
    print("=" * 50)
    
    # 模拟配置 / Mock configuration
    config = {
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
                'log_velocity_changes': True,
                'emergency_stop': {
                    'enable': True,
                    'auto_recovery_time': 15.0,
                    'manual_recovery': False
                }
            }
        }
    }
    
    # 创建速度斜坡限制器 / Create velocity ramp limiter
    from rclpy.time import Time
    velocity_ramp = VelocityRamp(config)
    
    print(f"\n配置参数 / Configuration:")
    print(f"  最大线加速度 / Max linear acceleration: {velocity_ramp.max_linear_accel} m/s²")
    print(f"  最大角加速度 / Max angular acceleration: {velocity_ramp.max_angular_accel} rad/s²")
    print(f"  急停冷却时间 / Emergency cooldown: {velocity_ramp.cooldown_duration}s")
    
    # 测试1: 速度斜坡 / Test 1: Velocity ramping
    print(f"\n测试1: 速度斜坡（0→0.5m/s）")
    print("-" * 50)
    
    current_time = Time(seconds=0)
    target_vx, target_vy, target_omega = 0.5, 0.0, 0.0
    
    for i in range(5):
        limited_vx, limited_vy, limited_omega = velocity_ramp.limit(
            target_vx, target_vy, target_omega, current_time
        )
        print(f"  t={i*0.02:.2f}s: limited_vx={limited_vx:.3f} m/s")
        current_time = Time(seconds=(i + 1) * 0.02)
    
    # 测试2: 急停 / Test 2: Emergency stop
    print(f"\n测试2: 急停功能")
    print("-" * 50)
    velocity_ramp.emergency_stop("Test emergency stop")
    
    limited_vx, limited_vy, limited_omega = velocity_ramp.limit(
        0.5, 0.0, 0.0, Time(seconds=1.0)
    )
    print(f"  急停后速度 / Velocity after emergency: vx={limited_vx:.3f} m/s (expected: 0.0)")
    
    # 测试3: 状态查询 / Test 3: Status query
    print(f"\n测试3: 状态查询")
    print("-" * 50)
    status = velocity_ramp.get_status()
    print(f"  急停状态 / Emergency stopped: {status['is_emergency_stopped']}")
    if status['is_emergency_stopped']:
        print(f"  急停原因 / Reason: {status['emergency_stop_reason']}")
        print(f"  剩余冷却时间 / Remaining cooldown: {status['emergency_remaining_time']:.1f}s")
    
    print(f"\n✅ 测试完成 / Test completed")


if __name__ == '__main__':
    main()
