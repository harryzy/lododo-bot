#!/usr/bin/env python3
"""
MissionPlanner 功能集成测试脚本（增强版）

新功能：
  1. 命令行参数支持 - 可指定运行特定测试case
  2. 自动状态检测 - 订阅/cmd_vel等话题，验证机器人实际行为
  3. 更详细的测试报告

使用方法：
  # 运行所有测试
  python3 test_mission_integration_v2.py
  
  # 运行特定测试
  python3 test_mission_integration_v2.py --test test_navigation_pause_resume
  python3 test_mission_integration_v2.py --test 1  # 按编号
  
  # 列出所有测试
  python3 test_mission_integration_v2.py --list
  
  # 详细模式
  python3 test_mission_integration_v2.py --test 1 --verbose

Author: LeKiwi Bot Development Team
Date: 2025-12-30
"""

import rclpy
from rclpy.node import Node
import time
import argparse
import sys
from datetime import datetime
from colorama import Fore, Style, init
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
from collections import deque
import math

# 导入服务消息类型
from bot_navigation_msgs.srv import (
    NavigateToPose, StartExploration, StartPatrol,
    TaskControl, GetTaskStatus, EmergencyStop, ClearTasks
)

# 初始化colorama
init(autoreset=True)


class RobotStateMonitor:
    """机器人状态监控器 - 自动检测机器人是否真的在移动"""
    
    def __init__(self, node):
        self.node = node
        self.cmd_vel_history = deque(maxlen=10)  # 最近10个速度命令
        self.odom_history = deque(maxlen=20)  # 最近20个里程计数据
        self.current_twist = Twist()
        self.current_pose = None
        
        # 订阅速度命令
        self.cmd_vel_sub = node.create_subscription(
            Twist,
            '/cmd_vel',
            self._cmd_vel_callback,
            10
        )
        
        # 订阅里程计
        self.odom_sub = node.create_subscription(
            Odometry,
            '/wheel/odom',
            self._odom_callback,
            10
        )
        
        node.get_logger().info('RobotStateMonitor initialized')
    
    def _cmd_vel_callback(self, msg):
        """记录速度命令"""
        self.current_twist = msg
        self.cmd_vel_history.append({
            'linear_x': msg.linear.x,
            'linear_y': msg.linear.y,
            'angular_z': msg.angular.z,
            'timestamp': time.time()
        })
    
    def _odom_callback(self, msg):
        """记录里程计数据"""
        self.current_pose = msg.pose.pose
        self.odom_history.append({
            'x': msg.pose.pose.position.x,
            'y': msg.pose.pose.position.y,
            'linear_x': msg.twist.twist.linear.x,
            'linear_y': msg.twist.twist.linear.y,
            'angular_z': msg.twist.twist.angular.z,
            'timestamp': time.time()
        })
    
    def is_robot_moving(self, threshold=0.005, window=0.5):
        """
        检测机器人是否在移动
        
        策略：检查最近window秒内的平均速度
        
        Args:
            threshold: 速度阈值（m/s 或 rad/s）
            window: 时间窗口（秒）
        
        Returns:
            bool: True表示机器人在移动
        """
        current_time = time.time()
        
        # 检查最近的速度命令
        recent_cmd_vel = [
            cmd for cmd in self.cmd_vel_history 
            if current_time - cmd['timestamp'] < window
        ]
        
        if not recent_cmd_vel:
            return False
        
        # 计算平均速度
        avg_linear_x = sum(abs(cmd['linear_x']) for cmd in recent_cmd_vel) / len(recent_cmd_vel)
        avg_linear_y = sum(abs(cmd['linear_y']) for cmd in recent_cmd_vel) / len(recent_cmd_vel)
        avg_angular_z = sum(abs(cmd['angular_z']) for cmd in recent_cmd_vel) / len(recent_cmd_vel)
        
        # 如果平均速度超过阈值，认为在移动
        is_moving = (avg_linear_x > threshold or 
                     avg_linear_y > threshold or 
                     avg_angular_z > threshold)
        
        return is_moving
    
    def is_robot_stopped(self, threshold=0.005, window=0.5):
        """
        检测机器人是否已停止
        
        Args:
            threshold: 速度阈值
            window: 时间窗口（秒）
        
        Returns:
            bool: True表示机器人已停止
        """
        return not self.is_robot_moving(threshold, window)
    
    def get_robot_speed(self):
        """
        获取机器人当前速度（基于cmd_vel）
        
        Returns:
            dict: {'linear': float, 'angular': float}
        """
        if not self.cmd_vel_history:
            return {'linear': 0.0, 'angular': 0.0, 'linear_x': 0.0, 'linear_y': 0.0, 'angular_z': 0.0}
        
        # 只看最近1秒内的数据
        current_time = time.time()
        recent_cmd_vel = [
            cmd for cmd in self.cmd_vel_history
            if current_time - cmd['timestamp'] < 1.0
        ]
        
        if not recent_cmd_vel:
            return {'linear': 0.0, 'angular': 0.0, 'linear_x': 0.0, 'linear_y': 0.0, 'angular_z': 0.0}
        
        latest = recent_cmd_vel[-1]
        linear_speed = math.sqrt(latest['linear_x']**2 + latest['linear_y']**2)
        angular_speed = abs(latest['angular_z'])
        
        return {
            'linear': linear_speed,
            'angular': angular_speed,
            'linear_x': latest['linear_x'],
            'linear_y': latest['linear_y'],
            'angular_z': latest['angular_z']
        }
    
    def get_position_change(self, duration=1.0):
        """
        获取指定时间段内的位置变化
        
        Args:
            duration: 时间段（秒）
        
        Returns:
            float: 位移距离（米）
        """
        if len(self.odom_history) < 2:
            return 0.0
        
        current_time = time.time()
        recent_odom = [
            odom for odom in self.odom_history
            if current_time - odom['timestamp'] < duration
        ]
        
        if len(recent_odom) < 2:
            return 0.0
        
        start = recent_odom[0]
        end = recent_odom[-1]
        
        dx = end['x'] - start['x']
        dy = end['y'] - start['y']
        distance = math.sqrt(dx**2 + dy**2)
        
        return distance


class MissionIntegrationTesterV2(Node):
    """MissionPlanner 功能集成测试类（增强版）"""
    
    def __init__(self, verbose=False):
        super().__init__('mission_integration_tester_v2')
        
        self.verbose = verbose
        
        # 机器人状态监控器
        self.robot_monitor = RobotStateMonitor(self)
        
        # 测试统计
        self.test_results = {
            'total': 0,
            'passed': 0,
            'failed': 0,
            'details': []
        }
        
        # 测试用例列表
        self.test_cases = [
            {
                'id': 1,
                'name': 'test_navigation_pause_resume',
                'description': '测试导航任务的暂停/恢复控制',
                'function': self.test_navigation_pause_resume
            },
            {
                'id': 2,
                'name': 'test_navigation_cancel',
                'description': '测试导航任务的取消功能',
                'function': self.test_navigation_cancel
            },
            {
                'id': 3,
                'name': 'test_multiple_navigation_tasks',
                'description': '测试多个导航任务的队列执行',
                'function': self.test_multiple_navigation_tasks
            },
            {
                'id': 4,
                'name': 'test_emergency_stop',
                'description': '测试紧急停止功能',
                'function': self.test_emergency_stop
            },
            {
                'id': 5,
                'name': 'test_patrol_pause_resume',
                'description': '测试巡航任务的暂停/恢复',
                'function': self.test_patrol_pause_resume
            }
        ]
        
        self.get_logger().info('Mission Integration Tester V2 initialized')
    
    def call_service(self, service_name, service_type, request, timeout=5.0):
        """通用服务调用方法"""
        client = self.create_client(service_type, service_name)
        
        if not client.wait_for_service(timeout_sec=timeout):
            self.get_logger().error(f'Service {service_name} not available')
            return False, None
        
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
        
        if future.result() is not None:
            return True, future.result()
        else:
            self.get_logger().error(f'Service call to {service_name} failed')
            return False, None
    
    def log_verbose(self, message):
        """详细模式日志"""
        if self.verbose:
            print(f"{Fore.CYAN}[VERBOSE]{Style.RESET_ALL} {message}")
    
    def record_test(self, test_name, passed, message='', details=None):
        """记录测试结果"""
        self.test_results['total'] += 1
        
        if passed:
            self.test_results['passed'] += 1
            status = f"{Fore.GREEN}✓ PASS{Style.RESET_ALL}"
        else:
            self.test_results['failed'] += 1
            status = f"{Fore.RED}✗ FAIL{Style.RESET_ALL}"
        
        result = {
            'name': test_name,
            'status': 'PASS' if passed else 'FAIL',
            'message': message,
            'details': details,
            'timestamp': datetime.now().isoformat()
        }
        self.test_results['details'].append(result)
        
        print(f"{status} {test_name}: {message}")
        if details:
            print(f"     {Fore.CYAN}Details:{Style.RESET_ALL} {details}")
    
    def get_task_status(self, task_id):
        """获取任务状态"""
        req = GetTaskStatus.Request()
        req.task_id = task_id
        
        success, response = self.call_service('/mission/get_task_status', GetTaskStatus, req)
        
        if success and response.found:
            return {
                'state': response.state,
                'progress': response.progress,
                'error': response.error_message
            }
        return None
    
    # ========== 测试用例 ==========
    
    def test_navigation_pause_resume(self):
        """
        TEST 1: 导航任务暂停/恢复控制
        
        验证点：
        1. 导航任务启动后机器人开始移动
        2. 暂停后机器人立即停止
        3. 暂停期间机器人保持静止
        4. 恢复后机器人重新开始移动
        """
        print(f"\n{Fore.YELLOW}{'='*60}{Style.RESET_ALL}")
        print(f"{Fore.YELLOW}TEST 1: Navigation Pause/Resume Control{Style.RESET_ALL}")
        print(f"{Fore.YELLOW}{'='*60}{Style.RESET_ALL}\n")
        
        task_id = None
        
        try:
            # 步骤1：启动导航
           
            req = NavigateToPose.Request()

            req.x = 1.7891924381256104
            req.y = 0.12015881389379501
            req.yaw = -0.9204096538260552
            req.frame_id = 'map'
            
            print(f"{Fore.BLUE}[Step 1]{Style.RESET_ALL} 启动导航到目标点 (1.789, 0.120)")
            
            success, response = self.call_service('/mission/navigate_to_pose', NavigateToPose, req)
            if not success or not response.success:
                self.record_test('Navigation Start', False, '启动导航失败')
                return
            
            task_id = response.task_id
            self.record_test('Navigation Start', True, f'Task ID: {task_id}')
            
            # 步骤2：等待机器人开始移动
            print(f"{Fore.BLUE}[Step 2]{Style.RESET_ALL} 等待机器人开始移动...")
            time.sleep(2.0)
            
            # 多次spin确保接收到cmd_vel消息
            for _ in range(10):
                rclpy.spin_once(self, timeout_sec=0.1)
            
            # 验证：机器人应该在移动
            speed = self.robot_monitor.get_robot_speed()
            self.log_verbose(f"cmd_vel history size: {len(self.robot_monitor.cmd_vel_history)}")
            self.log_verbose(f"Current speed: {speed}")
            
            is_moving = self.robot_monitor.is_robot_moving()
            self.log_verbose(f"Robot moving: {is_moving}")
            
            self.record_test(
                'Robot Moving After Start',
                is_moving,
                f"机器人{'已开始移动' if is_moving else '未移动'}",
                f"Linear: {speed['linear']:.3f} m/s, Angular: {speed['angular']:.3f} rad/s"
            )
            
            # 步骤3：暂停任务
            print(f"{Fore.BLUE}[Step 3]{Style.RESET_ALL} 暂停任务")
            pause_req = TaskControl.Request()
            pause_req.task_id = task_id
            
            success, response = self.call_service('/mission/pause_task', TaskControl, pause_req)
            if not success or not response.success:
                self.record_test('Pause Task', False, '暂停失败')
                return
            
            self.record_test('Pause Task', True, '暂停命令发送成功')
            
            # 步骤4：验证机器人停止
            print(f"{Fore.BLUE}[Step 4]{Style.RESET_ALL} 验证机器人是否停止...")
            time.sleep(2.5)  # 给MissionPlanner时间处理PAUSED状态并取消导航
            
            # 多次spin确保接收最新cmd_vel
            for _ in range(10):
                rclpy.spin_once(self, timeout_sec=0.1)
            
            speed_after_pause = self.robot_monitor.get_robot_speed()
            self.log_verbose(f"cmd_vel history size: {len(self.robot_monitor.cmd_vel_history)}")
            if self.robot_monitor.cmd_vel_history:
                latest_time = self.robot_monitor.cmd_vel_history[-1]['timestamp']
                age = time.time() - latest_time
                self.log_verbose(f"Latest cmd_vel age: {age:.2f}s")
            self.log_verbose(f"Speed after pause: {speed_after_pause}")
            
            is_stopped = self.robot_monitor.is_robot_stopped()
            self.log_verbose(f"Robot stopped: {is_stopped}")
            
            self.record_test(
                'Robot Stopped After Pause',
                is_stopped,
                f"机器人{'已停止' if is_stopped else '仍在移动'}",
                f"Linear: {speed_after_pause['linear']:.3f} m/s, Angular: {speed_after_pause['angular']:.3f} rad/s"
            )
            
            # 步骤5：保持暂停，验证机器人保持静止
            print(f"{Fore.BLUE}[Step 5]{Style.RESET_ALL} 保持暂停3秒，验证机器人保持静止...")
            time.sleep(3.0)
            
            position_change = self.robot_monitor.get_position_change(duration=3.0)
            self.log_verbose(f"Position change during pause: {position_change:.4f} m")
            
            self.record_test(
                'Robot Stationary During Pause',
                position_change < 0.1,  # 位移小于10cm认为静止
                f"位移: {position_change:.4f} m"
            )
            
            # 步骤6：恢复任务
            print(f"{Fore.BLUE}[Step 6]{Style.RESET_ALL} 恢复任务")
            resume_req = TaskControl.Request()
            resume_req.task_id = task_id
            
            success, response = self.call_service('/mission/resume_task', TaskControl, resume_req)
            if not success or not response.success:
                self.record_test('Resume Task', False, '恢复失败')
                return
            
            self.record_test('Resume Task', True, '恢复命令发送成功')
            
            # 步骤7：验证机器人重新开始移动
            print(f"{Fore.BLUE}[Step 7]{Style.RESET_ALL} 验证机器人是否重新开始移动...")
            time.sleep(3.0)  # 给MissionPlanner时间检测CANCELED状态并重新发送导航目标
            
            # 多次spin确保接收cmd_vel
            for _ in range(10):
                rclpy.spin_once(self, timeout_sec=0.1)
            
            speed_after_resume = self.robot_monitor.get_robot_speed()
            self.log_verbose(f"cmd_vel history size: {len(self.robot_monitor.cmd_vel_history)}")
            if self.robot_monitor.cmd_vel_history:
                latest_time = self.robot_monitor.cmd_vel_history[-1]['timestamp']
                age = time.time() - latest_time
                self.log_verbose(f"Latest cmd_vel age: {age:.2f}s")
            self.log_verbose(f"Speed after resume: {speed_after_resume}")
            
            is_moving_again = self.robot_monitor.is_robot_moving()
            self.log_verbose(f"Robot moving: {is_moving_again}")
            
            self.record_test(
                'Robot Moving After Resume',
                is_moving_again,
                f"机器人{'重新开始移动' if is_moving_again else '未移动'}",
                f"Linear: {speed_after_resume['linear']:.3f} m/s, Angular: {speed_after_resume['angular']:.3f} rad/s"
            )
            
            # 步骤8：取消任务（清理）
            print(f"{Fore.BLUE}[Step 8]{Style.RESET_ALL} 取消任务（清理）")
            cancel_req = TaskControl.Request()
            cancel_req.task_id = task_id
            
            self.call_service('/mission/cancel_task', TaskControl, cancel_req)
            time.sleep(1.0)
            
            # 验证机器人停止
            is_stopped_final = self.robot_monitor.is_robot_stopped()
            self.record_test(
                'Robot Stopped After Cancel',
                is_stopped_final,
                f"机器人{'已停止' if is_stopped_final else '仍在移动'}"
            )
            
        except Exception as e:
            self.get_logger().error(f'Test failed with exception: {str(e)}')
            self.record_test('Navigation Pause/Resume Test', False, f'异常: {str(e)}')
    
    def test_navigation_cancel(self):
        """
        TEST 2: 导航任务取消功能
        
        验证点：
        1. 导航任务启动后机器人开始移动
        2. 取消任务后机器人立即停止
        3. 任务状态变为CANCELED
        """
        print(f"\n{Fore.YELLOW}{'='*60}{Style.RESET_ALL}")
        print(f"{Fore.YELLOW}TEST 2: Navigation Cancel{Style.RESET_ALL}")
        print(f"{Fore.YELLOW}{'='*60}{Style.RESET_ALL}\n")
        
        task_id = None
        
        try:
            # 步骤1：启动导航
            req = NavigateToPose.Request()
            req.x = 2.0
            req.y = 1.0
            req.yaw = 0.0
            req.frame_id = 'map'
            
            print(f"{Fore.BLUE}[Step 1]{Style.RESET_ALL} 启动导航到目标点 (2.0, 1.0)")
            
            success, response = self.call_service('/mission/navigate_to_pose', NavigateToPose, req)
            if not success or not response.success:
                self.record_test('Navigation Start', False, '启动导航失败')
                return
            
            task_id = response.task_id
            self.record_test('Navigation Start', True, f'Task ID: {task_id}')
            
            # 步骤2：等待机器人开始移动
            print(f"{Fore.BLUE}[Step 2]{Style.RESET_ALL} 等待机器人开始移动...")
            time.sleep(2.0)
            
            for _ in range(10):
                rclpy.spin_once(self, timeout_sec=0.1)
            
            is_moving = self.robot_monitor.is_robot_moving()
            speed = self.robot_monitor.get_robot_speed()
            self.log_verbose(f"Robot moving: {is_moving}, Speed: {speed}")
            
            self.record_test(
                'Robot Moving After Start',
                is_moving,
                f"机器人{'已开始移动' if is_moving else '未移动'}",
                f"Linear: {speed['linear']:.3f} m/s"
            )
            
            # 步骤3：取消任务
            print(f"{Fore.BLUE}[Step 3]{Style.RESET_ALL} 取消任务")
            cancel_req = TaskControl.Request()
            cancel_req.task_id = task_id
            
            success, response = self.call_service('/mission/cancel_task', TaskControl, cancel_req)
            if not success or not response.success:
                self.record_test('Cancel Task', False, '取消失败')
                return
            
            self.record_test('Cancel Task', True, '取消命令发送成功')
            
            # 步骤4：验证机器人停止
            print(f"{Fore.BLUE}[Step 4]{Style.RESET_ALL} 验证机器人是否停止...")
            time.sleep(2.0)
            
            for _ in range(10):
                rclpy.spin_once(self, timeout_sec=0.1)
            
            is_stopped = self.robot_monitor.is_robot_stopped()
            speed_after = self.robot_monitor.get_robot_speed()
            self.log_verbose(f"Robot stopped: {is_stopped}, Speed: {speed_after}")
            
            self.record_test(
                'Robot Stopped After Cancel',
                is_stopped,
                f"机器人{'已停止' if is_stopped else '仍在移动'}",
                f"Linear: {speed_after['linear']:.3f} m/s"
            )
            
            # 步骤5：验证任务状态
            print(f"{Fore.BLUE}[Step 5]{Style.RESET_ALL} 检查任务状态")
            status_req = GetTaskStatus.Request()
            status_req.task_id = task_id
            
            success, response = self.call_service('/mission/get_task_status', GetTaskStatus, status_req)
            if success:
                self.log_verbose(f"Task status: {response.state}")
                self.record_test(
                    'Task State Check',
                    response.state == 'CANCELED',
                    f"状态: {response.state}"
                )
            
        except Exception as e:
            self.get_logger().error(f'Test failed with exception: {str(e)}')
            self.record_test('Navigation Cancel Test', False, f'异常: {str(e)}')
    
    def test_multiple_navigation_tasks(self):
        """
        TEST 3: 多个导航任务队列执行
        
        验证点：
        1. 可以连续提交多个导航任务
        2. 任务按顺序执行
        3. 第一个任务完成后自动开始第二个
        """
        print(f"\n{Fore.YELLOW}{'='*60}{Style.RESET_ALL}")
        print(f"{Fore.YELLOW}TEST 3: Multiple Navigation Tasks{Style.RESET_ALL}")
        print(f"{Fore.YELLOW}{'='*60}{Style.RESET_ALL}\n")
        
        task_ids = []
        
        try:
            # 步骤1：提交第一个导航任务
            print(f"{Fore.BLUE}[Step 1]{Style.RESET_ALL} 提交第一个导航任务")
            req1 = NavigateToPose.Request()
            req1.x = 1.0
            req1.y = 0.5
            req1.yaw = 0.0
            req1.frame_id = 'map'
            
            success, response = self.call_service('/mission/navigate_to_pose', NavigateToPose, req1)
            if success and response.success:
                task_ids.append(response.task_id)
                self.record_test('Submit Task 1', True, f'Task ID: {response.task_id}')
            else:
                self.record_test('Submit Task 1', False, '提交失败')
                return
            
            # 步骤2：立即提交第二个导航任务
            print(f"{Fore.BLUE}[Step 2]{Style.RESET_ALL} 提交第二个导航任务")
            req2 = NavigateToPose.Request()
            req2.x = 1.5
            req2.y = 1.0
            req2.yaw = 1.57
            req2.frame_id = 'map'
            
            success, response = self.call_service('/mission/navigate_to_pose', NavigateToPose, req2)
            if success and response.success:
                # 检查是否真的创建了新任务（ID不同）
                if response.task_id == task_ids[0]:
                    self.record_test('Submit Task 2', False, f'返回了相同的任务ID: {response.task_id}')
                    return
                task_ids.append(response.task_id)
                self.record_test('Submit Task 2', True, f'Task ID: {response.task_id}')
            else:
                self.record_test('Submit Task 2', False, '提交失败')
            
            # 步骤3：验证第一个任务正在执行
            print(f"{Fore.BLUE}[Step 3]{Style.RESET_ALL} 检查任务状态")
            time.sleep(1.0)
            
            status_req = GetTaskStatus.Request()
            status_req.task_id = task_ids[0]
            
            success, response = self.call_service('/mission/get_task_status', GetTaskStatus, status_req)
            if success:
                self.log_verbose(f"Task 1 status: {response.state}")
                self.record_test(
                    'Task 1 Executing',
                    response.state in ['RUNNING', 'COMPLETED'],  # 可能已经完成
                    f"状态: {response.state}"
                )
            
            # 步骤4：检查第二个任务状态
            # 注意：由于导航任务可能很快完成，第二个任务可能已经开始或仍在等待
            status_req.task_id = task_ids[1]
            success, response = self.call_service('/mission/get_task_status', GetTaskStatus, status_req)
            if success:
                self.log_verbose(f"Task 2 status: {response.state}")
                # 允许三种情况：
                # 1. PENDING/BLOCKED - 等待第一个任务
                # 2. RUNNING - 第一个任务已完成，开始执行
                # 3. COMPLETED - 两个任务都已完成
                is_valid = response.state in ['PENDING', 'BLOCKED', 'RUNNING', 'COMPLETED']
                self.record_test(
                    'Task 2 Queued or Executing',
                    is_valid,
                    f"状态: {response.state} ({'正常' if is_valid else '异常'})"
                )
            
            # 步骤5：取消所有任务（清理）
            print(f"{Fore.BLUE}[Step 5]{Style.RESET_ALL} 取消所有任务")
            for task_id in task_ids:
                cancel_req = TaskControl.Request()
                cancel_req.task_id = task_id
                self.call_service('/mission/cancel_task', TaskControl, cancel_req)
            
            time.sleep(1.0)
            
        except Exception as e:
            self.get_logger().error(f'Test failed with exception: {str(e)}')
            self.record_test('Multiple Navigation Tasks Test', False, f'异常: {str(e)}')
    
    def test_emergency_stop(self):
        """
        TEST 4: 紧急停止功能
        
        验证点：
        1. 导航过程中可以触发紧急停止
        2. 紧急停止后机器人立即停止
        3. 所有任务被取消
        """
        print(f"\n{Fore.YELLOW}{'='*60}{Style.RESET_ALL}")
        print(f"{Fore.YELLOW}TEST 4: Emergency Stop{Style.RESET_ALL}")
        print(f"{Fore.YELLOW}{'='*60}{Style.RESET_ALL}\n")
        
        task_id = None
        
        try:
            # 步骤1：启动导航
            print(f"{Fore.BLUE}[Step 1]{Style.RESET_ALL} 启动导航任务")
            req = NavigateToPose.Request()
            req.x = 2.5
            req.y = 1.5
            req.yaw = 0.0
            req.frame_id = 'map'
            
            success, response = self.call_service('/mission/navigate_to_pose', NavigateToPose, req)
            if not success or not response.success:
                self.record_test('Navigation Start', False, '启动导航失败')
                return
            
            task_id = response.task_id
            self.record_test('Navigation Start', True, f'Task ID: {task_id}')
            
            # 步骤2：等待机器人开始移动
            print(f"{Fore.BLUE}[Step 2]{Style.RESET_ALL} 等待机器人开始移动...")
            time.sleep(2.0)
            
            for _ in range(10):
                rclpy.spin_once(self, timeout_sec=0.1)
            
            is_moving = self.robot_monitor.is_robot_moving()
            self.log_verbose(f"Robot moving before emergency stop: {is_moving}")
            
            self.record_test(
                'Robot Moving Before Emergency Stop',
                is_moving,
                f"机器人{'正在移动' if is_moving else '未移动'}"
            )
            
            # 步骤3：触发紧急停止
            print(f"{Fore.BLUE}[Step 3]{Style.RESET_ALL} 触发紧急停止")
            estop_req = EmergencyStop.Request()
            
            success, response = self.call_service('/mission/emergency_stop', EmergencyStop, estop_req)
            if not success or not response.success:
                self.record_test('Emergency Stop', False, '紧急停止失败')
                return
            
            self.record_test('Emergency Stop', True, '紧急停止命令发送成功')
            
            # 步骤4：验证机器人立即停止
            print(f"{Fore.BLUE}[Step 4]{Style.RESET_ALL} 验证机器人是否停止...")
            time.sleep(2.0)
            
            for _ in range(10):
                rclpy.spin_once(self, timeout_sec=0.1)
            
            is_stopped = self.robot_monitor.is_robot_stopped()
            speed = self.robot_monitor.get_robot_speed()
            self.log_verbose(f"Robot stopped: {is_stopped}, Speed: {speed}")
            
            self.record_test(
                'Robot Stopped After Emergency Stop',
                is_stopped,
                f"机器人{'已停止' if is_stopped else '仍在移动'}",
                f"Linear: {speed['linear']:.3f} m/s"
            )
            
            # 步骤5：验证任务被取消
            print(f"{Fore.BLUE}[Step 5]{Style.RESET_ALL} 检查任务状态")
            status_req = GetTaskStatus.Request()
            status_req.task_id = task_id
            
            success, response = self.call_service('/mission/get_task_status', GetTaskStatus, status_req)
            if success:
                self.log_verbose(f"Task status after emergency stop: {response.state}")
                self.record_test(
                    'Task Canceled After Emergency Stop',
                    response.state in ['CANCELED', 'FAILED'],
                    f"状态: {response.state}"
                )
            
            # 步骤6：清理
            print(f"{Fore.BLUE}[Step 6]{Style.RESET_ALL} 清理所有任务")
            clear_req = ClearTasks.Request()
            clear_req.states = ['all']  # 清除所有状态的任务
            self.call_service('/mission/clear_tasks', ClearTasks, clear_req)
            
        except Exception as e:
            self.get_logger().error(f'Test failed with exception: {str(e)}')
            self.record_test('Emergency Stop Test', False, f'异常: {str(e)}')
    
    def test_patrol_pause_resume(self):
        """
        TEST 5: 巡航任务暂停/恢复
        
        验证点：
        1. 巡航任务启动后机器人开始移动
        2. 暂停后机器人停止
        3. 恢复后机器人继续巡航
        
        注意：此测试需要预先准备好路点文件
        """
        print(f"\n{Fore.YELLOW}{'='*60}{Style.RESET_ALL}")
        print(f"{Fore.YELLOW}TEST 5: Patrol Pause/Resume{Style.RESET_ALL}")
        print(f"{Fore.YELLOW}{'='*60}{Style.RESET_ALL}\n")
        
        task_id = None
        
        try:
            # 步骤1：检查是否有测试路点文件
            import os
            waypoint_file = os.path.expanduser('~/lododo_bot/waypoints/test_patrol_short.yaml')
            
            if not os.path.exists(waypoint_file):
                print(f"{Fore.YELLOW}警告: 未找到测试路点文件 {waypoint_file}{Style.RESET_ALL}")
                print(f"{Fore.YELLOW}跳过巡航测试，请先使用waypoint_recorder录制路点{Style.RESET_ALL}")
                self.record_test('Patrol Test', False, '缺少路点文件，跳过测试')
                return
            
            # 步骤2：启动巡航
            print(f"{Fore.BLUE}[Step 1]{Style.RESET_ALL} 启动巡航任务")
            req = StartPatrol.Request()
            req.waypoint_file = waypoint_file
            req.patrol_mode = 'loop'  # 循环巡航模式
            req.speed_factor = 1.0
            
            success, response = self.call_service('/mission/start_patrol', StartPatrol, req)
            if not success or not response.success:
                self.record_test('Patrol Start', False, '启动巡航失败')
                return
            
            task_id = response.task_id
            self.record_test('Patrol Start', True, f'Task ID: {task_id}')
            
            # 步骤3：等待机器人开始移动
            print(f"{Fore.BLUE}[Step 2]{Style.RESET_ALL} 等待机器人开始巡航...")
            time.sleep(3.0)
            
            for _ in range(10):
                rclpy.spin_once(self, timeout_sec=0.1)
            
            is_moving = self.robot_monitor.is_robot_moving()
            self.log_verbose(f"Robot moving: {is_moving}")
            
            self.record_test(
                'Robot Moving During Patrol',
                is_moving,
                f"机器人{'正在巡航' if is_moving else '未移动'}"
            )
            
            # 步骤4：暂停巡航
            print(f"{Fore.BLUE}[Step 3]{Style.RESET_ALL} 暂停巡航")
            pause_req = TaskControl.Request()
            pause_req.task_id = task_id
            
            success, response = self.call_service('/mission/pause_task', TaskControl, pause_req)
            if not success or not response.success:
                self.record_test('Pause Patrol', False, '暂停失败')
                return
            
            self.record_test('Pause Patrol', True, '暂停命令发送成功')
            
            # 步骤5：验证机器人停止
            print(f"{Fore.BLUE}[Step 4]{Style.RESET_ALL} 验证机器人是否停止...")
            time.sleep(2.0)
            
            for _ in range(10):
                rclpy.spin_once(self, timeout_sec=0.1)
            
            is_stopped = self.robot_monitor.is_robot_stopped()
            self.log_verbose(f"Robot stopped: {is_stopped}")
            
            self.record_test(
                'Robot Stopped After Pause',
                is_stopped,
                f"机器人{'已停止' if is_stopped else '仍在移动'}"
            )
            
            # 步骤6：恢复巡航
            print(f"{Fore.BLUE}[Step 5]{Style.RESET_ALL} 恢复巡航")
            resume_req = TaskControl.Request()
            resume_req.task_id = task_id
            
            success, response = self.call_service('/mission/resume_task', TaskControl, resume_req)
            if not success or not response.success:
                self.record_test('Resume Patrol', False, '恢复失败')
                return
            
            self.record_test('Resume Patrol', True, '恢复命令发送成功')
            
            # 步骤7：验证机器人重新开始巡航
            print(f"{Fore.BLUE}[Step 6]{Style.RESET_ALL} 验证机器人是否继续巡航...")
            time.sleep(3.0)
            
            for _ in range(10):
                rclpy.spin_once(self, timeout_sec=0.1)
            
            is_moving_again = self.robot_monitor.is_robot_moving()
            self.log_verbose(f"Robot moving: {is_moving_again}")
            
            self.record_test(
                'Robot Moving After Resume',
                is_moving_again,
                f"机器人{'继续巡航' if is_moving_again else '未移动'}"
            )
            
            # 步骤8：取消巡航（清理）
            print(f"{Fore.BLUE}[Step 7]{Style.RESET_ALL} 取消巡航任务")
            cancel_req = TaskControl.Request()
            cancel_req.task_id = task_id
            self.call_service('/mission/cancel_task', TaskControl, cancel_req)
            
            time.sleep(1.0)
            
        except Exception as e:
            self.get_logger().error(f'Test failed with exception: {str(e)}')
            self.record_test('Patrol Pause/Resume Test', False, f'异常: {str(e)}')
    
    # ========== 测试管理 ==========
    
    def list_tests(self):
        """列出所有可用的测试"""
        print(f"\n{Fore.CYAN}Available Tests:{Style.RESET_ALL}\n")
        for test in self.test_cases:
            print(f"  {test['id']}. {Fore.GREEN}{test['name']}{Style.RESET_ALL}")
            print(f"     {test['description']}\n")
    
    def run_test(self, test_identifier):
        """
        运行指定的测试
        
        Args:
            test_identifier: 测试名称或编号
        """
        # 查找测试
        test = None
        if isinstance(test_identifier, int) or test_identifier.isdigit():
            test_id = int(test_identifier)
            test = next((t for t in self.test_cases if t['id'] == test_id), None)
        else:
            test = next((t for t in self.test_cases if t['name'] == test_identifier), None)
        
        if test is None:
            print(f"{Fore.RED}Error: Test '{test_identifier}' not found{Style.RESET_ALL}")
            return
        
        # 运行测试
        print(f"\n{Fore.MAGENTA}Running: {test['name']}{Style.RESET_ALL}")
        print(f"{Fore.MAGENTA}Description: {test['description']}{Style.RESET_ALL}")
        
        test['function']()
    
    def run_all_tests(self):
        """运行所有测试"""
        print(f"\n{Fore.MAGENTA}{'='*60}{Style.RESET_ALL}")
        print(f"{Fore.MAGENTA}Running All Integration Tests{Style.RESET_ALL}")
        print(f"{Fore.MAGENTA}{'='*60}{Style.RESET_ALL}\n")
        
        for test in self.test_cases:
            self.run_test(test['id'])
            time.sleep(2.0)  # 测试间隔
    
    def print_summary(self):
        """打印测试摘要"""
        print(f"\n{Fore.CYAN}{'='*60}{Style.RESET_ALL}")
        print(f"{Fore.CYAN}Test Summary{Style.RESET_ALL}")
        print(f"{Fore.CYAN}{'='*60}{Style.RESET_ALL}")
        
        total = self.test_results['total']
        passed = self.test_results['passed']
        failed = self.test_results['failed']
        
        pass_rate = (passed / total * 100) if total > 0 else 0
        
        print(f"Total Tests:  {total}")
        print(f"{Fore.GREEN}Passed:       {passed}{Style.RESET_ALL}")
        print(f"{Fore.RED}Failed:       {failed}{Style.RESET_ALL}")
        print(f"Pass Rate:    {pass_rate:.1f}%")
        
        print(f"\n{Fore.CYAN}Detailed Results:{Style.RESET_ALL}")
        for result in self.test_results['details']:
            status_color = Fore.GREEN if result['status'] == 'PASS' else Fore.RED
            print(f"  {status_color}{result['status']}{Style.RESET_ALL} - {result['name']}")
            if result['message']:
                print(f"       {result['message']}")
            if result['details']:
                print(f"       {result['details']}")
        
        print(f"{Fore.CYAN}{'='*60}{Style.RESET_ALL}\n")


def main():
    """主函数"""
    parser = argparse.ArgumentParser(
        description='MissionPlanner Integration Tests (Enhanced)',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # 运行所有测试
  python3 test_mission_integration_v2.py
  
  # 运行特定测试（按名称）
  python3 test_mission_integration_v2.py --test test_navigation_pause_resume
  
  # 运行特定测试（按编号）
  python3 test_mission_integration_v2.py --test 1
  
  # 列出所有测试
  python3 test_mission_integration_v2.py --list
  
  # 详细模式
  python3 test_mission_integration_v2.py --test 1 --verbose
        """
    )
    
    parser.add_argument('--test', type=str, help='指定要运行的测试（名称或编号）')
    parser.add_argument('--list', action='store_true', help='列出所有可用的测试')
    parser.add_argument('--verbose', action='store_true', help='详细模式（显示更多调试信息）')
    
    args = parser.parse_args()
    
    rclpy.init()
    tester = MissionIntegrationTesterV2(verbose=args.verbose)
    
    try:
        if args.list:
            tester.list_tests()
        elif args.test:
            tester.run_test(args.test)
            tester.print_summary()
        else:
            tester.run_all_tests()
            tester.print_summary()
    
    except KeyboardInterrupt:
        print(f"\n{Fore.YELLOW}Tests interrupted by user{Style.RESET_ALL}")
    
    finally:
        tester.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
