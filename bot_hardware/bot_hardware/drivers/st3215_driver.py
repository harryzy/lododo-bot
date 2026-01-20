#!/usr/bin/env python3
"""
ST3215Driver - ST3215舵机驱动封装 / ST3215 Servo Driver Wrapper

本模块封装ST3215舵机的TTL串口通信协议，提供简洁的Python API。
This module wraps ST3215 servo TTL serial communication protocol with clean Python API.

参考设计文档 / Reference: HARDWARE_DEPLOYMENT_DESIGN.md §3.1 (Line 1869-2233)

核心功能 / Core Features:
- 速度控制 / Speed control
- 位置读取 / Position reading  
- 状态查询 / Status query
- 自动重试机制 / Auto-retry mechanism
- 错误处理 / Error handling

Author: Hurry
Created: 2026-01-19
"""

import time
import threading
from typing import Dict, Optional, Tuple
from bot_hardware.ftservo.scservo_sdk import PortHandler, sms_sts, COMM_SUCCESS


class ST3215Driver:
    """
    ST3215舵机驱动类 / ST3215 Servo Driver Class
    
    封装scservo_sdk提供的底层API，提供高层接口。
    Wraps low-level scservo_sdk API to provide high-level interface.
    
    Note: ST3215使用SMS/STS协议（sms_sts packet handler）
          ST3215 uses SMS/STS protocol (sms_sts packet handler)
    """
    
    def __init__(self, port: str, baudrate: int, servo_ids: list, 
                 retry_count: int = 3, retry_delay: float = 0.001,
                 logger=None):
        """
        初始化ST3215Driver / Initialize ST3215Driver
        
        Args:
            port: 串口设备路径 / Serial port device path (e.g., '/dev/lekiwi_servo')
            baudrate: 波特率 / Baudrate (e.g., 1000000)
            servo_ids: 舵机ID列表 / List of servo IDs (e.g., [7, 8, 9])
            retry_count: 通信失败重试次数 / Retry count on communication failure
            retry_delay: 重试延迟(秒) / Retry delay (seconds)
            logger: ROS2 logger对象 / ROS2 logger object (optional)
        """
        self.port = port
        self.baudrate = baudrate
        self.servo_ids = servo_ids
        self.retry_count = retry_count
        self.retry_delay = retry_delay
        self.logger = logger
        
        # 串口访问线程锁 - 防止control_loop和health_monitor并发访问冲突
        # Serial port access lock - prevents concurrent access from control_loop and health_monitor
        self._serial_lock = threading.Lock()
        
        # 初始化SDK / Initialize SDK
        self.port_handler = PortHandler(port)
        self.packet_handler = sms_sts(self.port_handler)
        
        # 缓存上次有效位置和速度 / Cache last valid position and speed
        self.last_valid_position: Dict[int, int] = {}
        self.last_valid_speed: Dict[int, int] = {}
        
        # 舵机在线状态 / Servo online status
        self.servo_online: Dict[int, bool] = {servo_id: False for servo_id in servo_ids}
        self.last_success_time: Dict[int, float] = {}
        
        # 通信统计 / Communication statistics
        self.comm_success_count: Dict[int, int] = {servo_id: 0 for servo_id in servo_ids}
        self.comm_failure_count: Dict[int, int] = {servo_id: 0 for servo_id in servo_ids}
    
    def initialize(self) -> bool:
        """
        初始化串口连接并扫描舵机 / Initialize serial connection and scan servos
        
        Returns:
            bool: 初始化成功返回True / True if initialization successful
        """
        # 打开串口 / Open serial port
        if not self.port_handler.openPort():
            if self.logger:
                self.logger.error(f"Failed to open port: {self.port}")
            return False
        
        if self.logger:
            self.logger.info(f"Serial port opened: {self.port}")
        
        # 设置波特率 / Set baudrate
        if not self.port_handler.setBaudRate(self.baudrate):
            if self.logger:
                self.logger.error(f"Failed to set baudrate: {self.baudrate}")
            return False
        
        if self.logger:
            self.logger.info(f"Baudrate set to: {self.baudrate}")
        
        # 扫描所有舵机 / Scan all servos
        online_count = 0
        for servo_id in self.servo_ids:
            if self._ping_servo(servo_id):
                self.servo_online[servo_id] = True
                online_count += 1
                if self.logger:
                    self.logger.info(f"Servo {servo_id} detected and online")
                
                # 设置为轮子模式 / Set to wheel mode
                if self._set_wheel_mode(servo_id):
                    if self.logger:
                        self.logger.info(f"Servo {servo_id} set to wheel mode")
                else:
                    if self.logger:
                        self.logger.warn(f"Servo {servo_id} failed to set wheel mode")
            else:
                self.servo_online[servo_id] = False
                if self.logger:
                    self.logger.warn(f"Servo {servo_id} not responding")
        
        if online_count == 0:
            if self.logger:
                self.logger.error("No servos detected! Check connections.")
            return False
        
        if online_count < len(self.servo_ids):
            if self.logger:
                self.logger.warn(f"Only {online_count}/{len(self.servo_ids)} servos online")
        
        return True
    
    def _set_wheel_mode(self, servo_id: int) -> bool:
        """
        设置舵机为轮子模式 / Set servo to wheel mode
        
        Args:
            servo_id: 舵机ID / Servo ID
        
        Returns:
            bool: 设置成功返回True / True if set successfully
        
        Note:
            轮子模式下舵机可以连续旋转，不受位置限制 / 
            In wheel mode, servo can rotate continuously without position limits
            WheelMode返回 (result, error) 元组 / WheelMode returns (result, error) tuple
        """
        with self._serial_lock:
            try:
                result, error = self.packet_handler.WheelMode(servo_id)
                return result == COMM_SUCCESS
            except Exception as e:
                if self.logger:
                    self.logger.error(f"Exception setting wheel mode for servo {servo_id}: {e}")
                return False
    
    def _ping_servo(self, servo_id: int) -> bool:
        """
        Ping舵机检测在线状态 / Ping servo to check online status
        
        Args:
            servo_id: 舵机ID / Servo ID
        
        Returns:
            bool: 舵机在线返回True / True if servo is online
        
        Note:
            ping方法返回 (model_number, result, error) 元组
            ping method returns (model_number, result, error) tuple
        """
        with self._serial_lock:
            try:
                model_number, result, error = self.packet_handler.ping(servo_id)
                return result == COMM_SUCCESS
            except Exception as e:
                if self.logger:
                    self.logger.error(f"Exception during ping servo {servo_id}: {e}")
                return False
    
    def write_speed(self, servo_id: int, rpm: float) -> bool:
        """
        设置舵机速度（轮子模式） / Set servo speed (wheel mode)
        
        Args:
            servo_id: 舵机ID / Servo ID
            rpm: 转速(RPM)，支持负数表示反转 / Speed in RPM, negative for reverse
        
        Returns:
            bool: 发送成功返回True / True if command sent successfully
        
        Note:
            ST3215速度单位: 1 RPM ≈ 0.732 units
            ST3215速度范围约为±45 RPM（取决于负载）
            ST3215 speed range is approximately ±45 RPM (depends on load)
            
            使用WriteSpec方法 (速度, 加速度) / Use WriteSpec method (speed, acceleration)
        """
        # 转换RPM到SDK单位 / Convert RPM to SDK units
        # ST3215: 速度单位转换系数（根据实测数据校准）
        # 实测数据校准历史:
        # - 系数 73.2: 2 rad/s 命令 → 2.15 rad/s 实际 (超调 7.5%)
        # - 系数 68.0: 2 rad/s 命令 → 1.61 rad/s 实际 (欠调 -20%)
        # - 系数 70.0: 折中方案，预计误差在 ±5% 以内
        speed_value = int(rpm * 70.0)  # 校准后的平衡系数 / Calibrated balanced factor
        
        # 限制速度范围 / Limit speed range
        # SDK使用有符号16位整数 / SDK uses signed 16-bit integer
        speed_value = max(-32767, min(32767, speed_value))
        
        # 加速度参数 / Acceleration parameter
        # ST3215: 1 unit = 8.7 deg/s^2
        # 增加到100以获得更快响应和更稳定的控制
        # Increased to 100 for faster response and more stable control
        acceleration = 100
        
        # 发送速度指令（带重试）使用WriteSpec / Send speed command with retry using WriteSpec
        with self._serial_lock:
            for attempt in range(self.retry_count):
                try:
                    result, error = self.packet_handler.WriteSpec(
                        servo_id, 
                        speed_value, 
                        acceleration
                    )
                    
                    if result == COMM_SUCCESS:
                        self.comm_success_count[servo_id] += 1
                        self.last_success_time[servo_id] = time.time()
                        return True
                    else:
                        if self.logger:
                            self.logger.warn(
                                f"Servo {servo_id} write_speed failed: result={result}, error={error}, "
                                f"retry {attempt+1}/{self.retry_count}"
                            )
                except Exception as e:
                    if self.logger:
                        self.logger.error(f"Exception writing speed to servo {servo_id}: {e}")
                
                # 重试延迟 / Retry delay
                if attempt < self.retry_count - 1:
                    time.sleep(self.retry_delay)
            
            # 所有重试失败 / All retries failed
            self.comm_failure_count[servo_id] += 1
            if self.logger:
                self.logger.error(
                    f"Servo {servo_id} write_speed failed after {self.retry_count} attempts"
                )
            return False
    
    def read_position(self, servo_id: int) -> Optional[int]:
        """
        读取编码器绝对位置和速度 / Read encoder absolute position and speed
        
        Args:
            servo_id: 舵机ID / Servo ID
        
        Returns:
            Optional[int]: 编码器位置值(0-4095)，失败返回None / 
                          Encoder position (0-4095), None on failure
        
        Note:
            ST3215使用12位编码器，范围0-4095 / 
            ST3215 uses 12-bit encoder, range 0-4095
            使用ReadPosSpeed同时读取位置和速度 / Use ReadPosSpeed to read both position and speed
        """
        with self._serial_lock:
            for attempt in range(self.retry_count):
                try:
                    position, speed, result, error = self.packet_handler.ReadPosSpeed(servo_id)
                    
                    if result == COMM_SUCCESS:
                        self.comm_success_count[servo_id] += 1
                        self.last_success_time[servo_id] = time.time()
                        self.last_valid_position[servo_id] = position
                        self.last_valid_speed[servo_id] = speed
                        return position
                    else:
                        if self.logger:
                            self.logger.warn(
                                f"Servo {servo_id} read_position failed: result={result}, error={error}, "
                                f"retry {attempt+1}/{self.retry_count}"
                            )
                except Exception as e:
                    if self.logger:
                        self.logger.error(f"Exception reading position from servo {servo_id}: {e}")
                
                # 重试延迟 / Retry delay
                if attempt < self.retry_count - 1:
                    time.sleep(self.retry_delay)
            
            # 所有重试失败，返回上次有效值或None / All retries failed, return last valid or None
            self.comm_failure_count[servo_id] += 1
            last_valid = self.last_valid_position.get(servo_id, None)
            
            if last_valid is None:
                if self.logger:
                    self.logger.error(
                        f"Servo {servo_id} read_position failed after {self.retry_count} attempts, "
                        f"no valid data available"
                    )
            else:
                if self.logger:
                    self.logger.warn(
                        f"Servo {servo_id} read_position failed, using last valid value: {last_valid}"
                    )
            
            return last_valid
    
    def read_status(self, servo_id: int) -> Optional[Dict[str, float]]:
        """
        读取舵机状态（温度、电压、负载） / Read servo status (temperature, voltage, load)
        
        Args:
            servo_id: 舵机ID / Servo ID
        
        Returns:
            Optional[Dict]: 状态字典，包含position、speed /
                           Status dict with position, speed
                           失败返回None / None on failure
        
        Note:
            简化版本，只返回位置和速度信息 / Simplified version, only returns position and speed
            温度、电压、负载监控需要额外的寄存器读取实现 / 
            Temperature, voltage, load monitoring requires additional register read implementation
        """
        with self._serial_lock:
            try:
                position, speed, result, error = self.packet_handler.ReadPosSpeed(servo_id)
                
                if result == COMM_SUCCESS:
                    return {
                        'position': position,
                        'speed': speed,
                        'temperature': 0.0,  # TODO: 实现温度读取 / TODO: Implement temperature reading
                        'voltage': 0.0,      # TODO: 实现电压读取 / TODO: Implement voltage reading
                        'load': 0.0          # TODO: 实现负载读取 / TODO: Implement load reading
                    }
                else:
                    if self.logger:
                        self.logger.error(f"Failed to read status from servo {servo_id}: result={result}, error={error}")
                    return None
            except Exception as e:
                if self.logger:
                    self.logger.error(f"Exception reading status from servo {servo_id}: {e}")
                return None
    
    def get_communication_stats(self, servo_id: int) -> Tuple[int, int, float]:
        """
        获取通信统计信息 / Get communication statistics
        
        Args:
            servo_id: 舵机ID / Servo ID
        
        Returns:
            Tuple[int, int, float]: (成功次数, 失败次数, 成功率%) /
                                    (success_count, failure_count, success_rate%)
        """
        success = self.comm_success_count.get(servo_id, 0)
        failure = self.comm_failure_count.get(servo_id, 0)
        total = success + failure
        
        if total == 0:
            return 0, 0, 0.0
        
        success_rate = (success / total) * 100.0
        return success, failure, success_rate
    
    def is_servo_online(self, servo_id: int, timeout: float = 1.0) -> bool:
        """
        检查舵机是否在线 / Check if servo is online
        
        Args:
            servo_id: 舵机ID / Servo ID
            timeout: 超时时间(秒)，超过此时间无响应认为离线 / 
                    Timeout (seconds), consider offline if no response
        
        Returns:
            bool: 在线返回True / True if online
        """
        last_time = self.last_success_time.get(servo_id, 0)
        elapsed = time.time() - last_time
        return elapsed < timeout
    
    def close(self):
        """
        关闭串口连接 / Close serial port connection
        """
        self.port_handler.closePort()
        if self.logger:
            self.logger.info(f"Serial port closed: {self.port}")


def main():
    """
    测试函数 / Test function
    """
    print("ST3215Driver Test / ST3215Driver测试")
    print("=" * 60)
    
    # 测试配置 / Test configuration
    test_config = {
        'port': '/dev/lekiwi_servo',
        'baudrate': 1000000,
        'servo_ids': [7, 8, 9]
    }
    
    # 创建驱动实例 / Create driver instance
    driver = ST3215Driver(
        port=test_config['port'],
        baudrate=test_config['baudrate'],
        servo_ids=test_config['servo_ids']
    )
    
    # 初始化 / Initialize
    print("\n1. Initializing driver / 初始化驱动...")
    if driver.initialize():
        print("   ✓ Initialization successful / 初始化成功")
    else:
        print("   ✗ Initialization failed / 初始化失败")
        return
    
    # 读取位置 / Read positions
    print("\n2. Reading positions / 读取位置...")
    for servo_id in test_config['servo_ids']:
        position = driver.read_position(servo_id)
        if position is not None:
            print(f"   Servo {servo_id}: position = {position}")
        else:
            print(f"   Servo {servo_id}: read failed")
    
    # 读取状态 / Read status
    print("\n3. Reading status / 读取状态...")
    for servo_id in test_config['servo_ids']:
        status = driver.read_status(servo_id)
        if status:
            print(f"   Servo {servo_id}:")
            print(f"     - Temperature: {status['temperature']:.1f}°C")
            print(f"     - Voltage: {status['voltage']:.1f}V")
            print(f"     - Load: {status['load']:.1f}%")
        else:
            print(f"   Servo {servo_id}: status read failed")
    
    # 通信统计 / Communication statistics
    print("\n4. Communication statistics / 通信统计...")
    for servo_id in test_config['servo_ids']:
        success, failure, rate = driver.get_communication_stats(servo_id)
        print(f"   Servo {servo_id}: success={success}, failure={failure}, rate={rate:.1f}%")
    
    # 关闭连接 / Close connection
    print("\n5. Closing connection / 关闭连接...")
    driver.close()
    print("   ✓ Connection closed / 连接已关闭")
    
    print("\n" + "=" * 60)
    print("Test completed / 测试完成")


if __name__ == '__main__':
    main()
