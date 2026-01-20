#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
ServoHealthMonitor - 舵机健康监控工具类
Servo Health Monitor Utility Class

功能特性 / Features:
- 周期性检查舵机状态（温度、负载、电压）
- 异常检测和报警
- 触发急停机制（通过VelocityRamp）
- 10Hz检查频率（避免影响控制循环）

设计参考 / Design Reference:
- HARDWARE_DEPLOYMENT_DESIGN.md §3.1.4 Q4（急停协调机制）
- Round 6: 独立配置区servo.health_monitor
"""

import threading
import time
from typing import Optional, Dict


class ServoHealthMonitor:
    """舵机健康监控 / Servo Health Monitor
    
    职责 / Responsibilities:
    - 周期性检查舵机温度、负载、电压
    - 检测异常状态并记录警告/错误
    - 触发急停（通过VelocityRamp）
    - 可选发布状态话题
    
    使用方式 / Usage:
        monitor = ServoHealthMonitor(driver, config, velocity_ramp)
        monitor.start()  # 启动后台线程
        ...
        monitor.stop()   # 停止监控
    """
    
    def __init__(self, driver, config: Dict, velocity_ramp=None):
        """初始化健康监控 / Initialize health monitor
        
        Args:
            driver: ST3215Driver实例
            config: 硬件配置字典
            velocity_ramp: VelocityRamp实例（用于触发急停）
        """
        self.driver = driver
        self.config = config
        self.velocity_ramp = velocity_ramp
        
        # 从配置读取参数
        monitor_config = config.get('servo', {}).get('health_monitor', {})
        
        self.enabled = monitor_config.get('enable', True)
        self.check_interval = monitor_config.get('check_interval', 0.1)  # 10Hz
        
        # 阈值配置
        thresholds = monitor_config.get('thresholds', {})
        self.temp_warning = thresholds.get('temperature_warning', 65)  # °C
        self.temp_critical = thresholds.get('temperature_critical', 75)
        self.load_warning = thresholds.get('load_warning', 80)  # %
        self.load_critical = thresholds.get('load_critical', 95)
        
        # 故障处理配置
        fault_handling = monitor_config.get('fault_handling', {})
        self.consecutive_failures_threshold = fault_handling.get('consecutive_failures_threshold', 3)
        self.trigger_emergency_stop = fault_handling.get('trigger_emergency_stop_on_critical', True)
        
        # 日志配置
        logging_config = monitor_config.get('logging', {})
        self.log_warnings = logging_config.get('log_warnings', True)
        self.log_critical = logging_config.get('log_critical', True)
        
        # 舵机ID列表
        self.servo_ids = [
            config['servo']['wheel_1_id'],
            config['servo']['wheel_2_id'],
            config['servo']['wheel_3_id']
        ]
        
        # 状态追踪
        self.consecutive_failures = {servo_id: 0 for servo_id in self.servo_ids}
        self.last_status = {}
        
        # 线程控制
        self._monitor_thread: Optional[threading.Thread] = None
        self._stop_event = threading.Event()
        self._running = False
        
        # 日志器（需要外部注入）
        self._logger = None
    
    def set_logger(self, logger):
        """设置日志器 / Set logger
        
        Args:
            logger: ROS2 Logger对象
        """
        self._logger = logger
    
    def start(self):
        """启动健康监控 / Start health monitoring"""
        if not self.enabled:
            if self._logger:
                self._logger.info('[ServoHealthMonitor] Disabled by configuration')
            return
        
        if self._running:
            if self._logger:
                self._logger.warn('[ServoHealthMonitor] Already running')
            return
        
        self._stop_event.clear()
        self._running = True
        self._monitor_thread = threading.Thread(target=self._monitor_loop, daemon=True)
        self._monitor_thread.start()
        
        if self._logger:
            self._logger.info(f'[ServoHealthMonitor] Started (interval={self.check_interval}s)')
    
    def stop(self):
        """停止健康监控 / Stop health monitoring"""
        if not self._running:
            return
        
        self._stop_event.set()
        if self._monitor_thread:
            self._monitor_thread.join(timeout=2.0)
        self._running = False
        
        if self._logger:
            self._logger.info('[ServoHealthMonitor] Stopped')
    
    def _monitor_loop(self):
        """监控循环（后台线程）/ Monitor loop (background thread)"""
        while not self._stop_event.is_set():
            try:
                self._check_all_servos()
            except Exception as e:
                if self._logger:
                    self._logger.error(f'[ServoHealthMonitor] Check failed: {e}')
            
            # 等待下一个检查周期
            self._stop_event.wait(self.check_interval)
    
    def _check_all_servos(self):
        """检查所有舵机状态 / Check all servo status"""
        for servo_id in self.servo_ids:
            try:
                # 读取舵机状态（温度、负载、电压等）
                # 注意：这里需要ST3215Driver支持read_temperature等方法
                # 如果当前驱动不支持，暂时跳过
                
                # 临时实现：仅检查ping连通性
                if not self.driver._ping_servo(servo_id):
                    self._handle_servo_failure(servo_id, 'Servo not responding')
                    continue
                
                # 重置连续失败计数
                self.consecutive_failures[servo_id] = 0
                
                # TODO: 完整的温度/负载/电压检查
                # temperature = self.driver.read_temperature(servo_id)
                # load = self.driver.read_load(servo_id)
                # voltage = self.driver.read_voltage(servo_id)
                # self._check_thresholds(servo_id, temperature, load, voltage)
                
            except Exception as e:
                self._handle_servo_failure(servo_id, str(e))
    
    def _handle_servo_failure(self, servo_id: int, reason: str):
        """处理舵机失败 / Handle servo failure
        
        Args:
            servo_id: 舵机ID
            reason: 失败原因
        """
        self.consecutive_failures[servo_id] += 1
        
        if self.consecutive_failures[servo_id] >= self.consecutive_failures_threshold:
            # 连续失败达到阈值
            if self._logger and self.log_critical:
                self._logger.error(
                    f'[ServoHealthMonitor] CRITICAL: Servo {servo_id} failed '
                    f'{self.consecutive_failures[servo_id]} times - {reason}'
                )
            
            # 触发急停
            if self.trigger_emergency_stop and self.velocity_ramp:
                self.velocity_ramp.emergency_stop(f'Servo {servo_id} critical failure')
                if self._logger:
                    self._logger.error(f'[ServoHealthMonitor] Emergency stop triggered!')
        
        elif self._logger and self.log_warnings:
            self._logger.warn(
                f'[ServoHealthMonitor] Servo {servo_id} failed '
                f'({self.consecutive_failures[servo_id]}/{self.consecutive_failures_threshold}) - {reason}'
            )
    
    def _check_thresholds(self, servo_id: int, temperature: float, load: float, voltage: float):
        """检查阈值 / Check thresholds
        
        Args:
            servo_id: 舵机ID
            temperature: 温度（°C）
            load: 负载（%）
            voltage: 电压（V）
        """
        # 温度检查
        if temperature >= self.temp_critical:
            if self._logger and self.log_critical:
                self._logger.error(
                    f'[ServoHealthMonitor] CRITICAL: Servo {servo_id} temperature '
                    f'{temperature}°C >= {self.temp_critical}°C'
                )
            if self.trigger_emergency_stop and self.velocity_ramp:
                self.velocity_ramp.emergency_stop(f'Servo {servo_id} overheat')
        
        elif temperature >= self.temp_warning:
            if self._logger and self.log_warnings:
                self._logger.warn(
                    f'[ServoHealthMonitor] WARNING: Servo {servo_id} temperature '
                    f'{temperature}°C >= {self.temp_warning}°C'
                )
        
        # 负载检查
        if load >= self.load_critical:
            if self._logger and self.log_critical:
                self._logger.error(
                    f'[ServoHealthMonitor] CRITICAL: Servo {servo_id} load '
                    f'{load}% >= {self.load_critical}%'
                )
        
        elif load >= self.load_warning:
            if self._logger and self.log_warnings:
                self._logger.warn(
                    f'[ServoHealthMonitor] WARNING: Servo {servo_id} load '
                    f'{load}% >= {self.load_warning}%'
                )
    
    def is_running(self) -> bool:
        """检查监控是否运行 / Check if monitoring is running
        
        Returns:
            bool: True表示运行中
        """
        return self._running
    
    def get_status(self) -> Dict:
        """获取监控状态 / Get monitor status
        
        Returns:
            dict: 监控状态信息
        """
        return {
            'enabled': self.enabled,
            'running': self._running,
            'consecutive_failures': self.consecutive_failures.copy(),
            'last_status': self.last_status.copy()
        }
