#!/usr/bin/env python3
"""
Exploration Mapping Test Script - 探索建图功能测试脚本

功能：测试完整的自动探索建图流程
- 启动探索任务
- 监控探索进度
- 等待探索完成或超时
- 自动保存地图

使用方法：
  # 基础用法（默认参数）
  python3 test_exploration_mapping.py
  
  # 指定地图名称
  python3 test_exploration_mapping.py --map-name my_office
  
  # 设置探索时长（秒）
  python3 test_exploration_mapping.py --duration 600
  
  # 设置覆盖率阈值（0.0-1.0）
  python3 test_exploration_mapping.py --coverage 0.9
  
  # 详细输出模式
  python3 test_exploration_mapping.py --verbose

Author: LeKiwi Bot Development Team
Date: 2026-01-04
"""

import rclpy
from rclpy.node import Node
import time
import argparse
import sys
from datetime import datetime
from colorama import Fore, Style, init
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from collections import deque
import math

# 导入服务消息类型
from bot_navigation_msgs.srv import (
    StartExploration, GetTaskStatus, EmergencyStop
)

# 初始化colorama
init(autoreset=True)


class RobotStateMonitor:
    """监控机器人运动状态"""
    
    def __init__(self):
        self.cmd_vel_history = deque(maxlen=5)  # 保留最近5个速度命令
        self.odom_history = deque(maxlen=5)
        
    def update_cmd_vel(self, msg: Twist):
        """更新速度命令历史"""
        self.cmd_vel_history.append({
            'timestamp': time.time(),
            'linear': msg.linear.x,
            'angular': msg.angular.z
        })
    
    def update_odom(self, msg: Odometry):
        """更新里程计历史"""
        self.odom_history.append({
            'timestamp': time.time(),
            'linear': msg.twist.twist.linear.x,
            'angular': msg.twist.twist.angular.z
        })
    
    def is_robot_moving(self, threshold=0.01):
        """判断机器人是否在移动"""
        if len(self.cmd_vel_history) == 0:
            return False
        
        # 检查最近的速度命令
        recent = list(self.cmd_vel_history)[-3:]  # 最近3个
        for cmd in recent:
            if abs(cmd['linear']) > threshold or abs(cmd['angular']) > threshold:
                return True
        return False
    
    def get_robot_speed(self):
        """获取当前机器人速度"""
        if len(self.cmd_vel_history) > 0:
            latest = self.cmd_vel_history[-1]
            return {
                'linear': latest['linear'],
                'angular': latest['angular']
            }
        return {'linear': 0.0, 'angular': 0.0}


class ExplorationMappingTest(Node):
    """探索建图测试节点"""
    
    def __init__(self, args):
        super().__init__('exploration_mapping_test')
        
        # 测试参数
        self.map_name = args.map_name
        self.max_duration = args.duration
        self.coverage_threshold = args.coverage
        self.verbose = args.verbose
        
        # 机器人状态监控
        self.robot_monitor = RobotStateMonitor()
        
        # 订阅机器人状态话题
        self.cmd_vel_sub = self.create_subscription(
            Twist, '/cmd_vel', self.robot_monitor.update_cmd_vel, 10
        )
        self.odom_sub = self.create_subscription(
            Odometry, '/wheel/odom', self.robot_monitor.update_odom, 10
        )
        
        # 服务客户端
        self.start_exploration_client = self.create_client(
            StartExploration, '/mission/start_exploration'
        )
        self.get_status_client = self.create_client(
            GetTaskStatus, '/mission/get_task_status'
        )
        self.emergency_stop_client = self.create_client(
            EmergencyStop, '/mission/emergency_stop'
        )
        
        # 测试状态
        self.task_id = None
        self.start_time = None
        self.test_passed = False
        
        self.get_logger().info(f'Exploration Mapping Test initialized')
        self.get_logger().info(f'  Map name: {self.map_name}')
        self.get_logger().info(f'  Max duration: {self.max_duration}s')
        self.get_logger().info(f'  Coverage threshold: {self.coverage_threshold*100:.1f}%')
    
    def wait_for_services(self, timeout=10.0):
        """等待所有服务可用"""
        print(f"{Fore.CYAN}[Init] Waiting for mission services...{Style.RESET_ALL}")
        
        services = {
            'start_exploration': self.start_exploration_client,
            'get_task_status': self.get_status_client,
            'emergency_stop': self.emergency_stop_client
        }
        
        for name, client in services.items():
            if not client.wait_for_service(timeout_sec=timeout):
                self.get_logger().error(f'Service {name} not available')
                return False
            self.get_logger().info(f'✓ Service {name} ready')
        
        return True
    
    def call_service(self, client, request, timeout=10.0):
        """同步调用服务"""
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=timeout)
        
        if future.done():
            try:
                return True, future.result()
            except Exception as e:
                self.get_logger().error(f'Service call exception: {str(e)}')
                return False, None
        else:
            self.get_logger().error('Service call timeout')
            return False, None
    
    def get_task_status(self, task_id):
        """获取任务状态"""
        req = GetTaskStatus.Request()
        req.task_id = task_id
        
        success, response = self.call_service(self.get_status_client, req)
        if success and response and response.success:
            return {
                'state': response.state,
                'progress': response.progress,
                'message': response.message
            }
        return None
    
    def start_exploration(self):
        """启动探索任务"""
        print(f"\n{Fore.YELLOW}{'='*70}{Style.RESET_ALL}")
        print(f"{Fore.YELLOW}Starting Exploration Mapping Test{Style.RESET_ALL}")
        print(f"{Fore.YELLOW}{'='*70}{Style.RESET_ALL}\n")
        
        # 步骤1：启动探索任务
        print(f"{Fore.BLUE}[Step 1]{Style.RESET_ALL} Starting exploration task...")
        print(f"  Map name: {self.map_name}")
        print(f"  Max duration: {self.max_duration}s")
        print(f"  Coverage threshold: {self.coverage_threshold*100:.1f}%")
        
        req = StartExploration.Request()
        req.map_name = self.map_name
        req.save_map = True  # 完成后保存地图
        req.max_duration = float(self.max_duration)
        req.coverage_threshold = float(self.coverage_threshold)
        
        # 探索任务初始化需要更长时间（地图订阅、边界检测器等）
        success, response = self.call_service(self.start_exploration_client, req, timeout=15.0)
        
        if not success or not response or not response.success:
            error_msg = response.message if response else 'No response'
            print(f"{Fore.RED}✗ Failed to start exploration: {error_msg}{Style.RESET_ALL}")
            return False
        
        self.task_id = response.task_id
        self.start_time = time.time()
        
        print(f"{Fore.GREEN}✓ Exploration task started{Style.RESET_ALL}")
        print(f"  Task ID: {self.task_id}")
        print()
        
        return True
    
    def monitor_exploration(self):
        """监控探索进度"""
        print(f"{Fore.BLUE}[Step 2]{Style.RESET_ALL} Monitoring exploration progress...\n")
        
        # 等待任务获取执行器
        print(f"{Fore.CYAN}Waiting for task to acquire NavigationExecutor (3s)...{Style.RESET_ALL}")
        time.sleep(3.0)
        for _ in range(30):
            rclpy.spin_once(self, timeout_sec=0.1)
        
        last_progress = -1
        last_state = None
        motion_check_interval = 10.0  # 每10秒检查一次运动
        last_motion_check = time.time()
        
        while True:
            # 检查超时
            elapsed = time.time() - self.start_time
            if elapsed > self.max_duration:
                print(f"\n{Fore.YELLOW}⚠ Exploration timeout ({self.max_duration}s){Style.RESET_ALL}")
                return False
            
            # 获取任务状态
            status = self.get_task_status(self.task_id)
            
            if not status:
                print(f"{Fore.RED}✗ Failed to get task status{Style.RESET_ALL}")
                time.sleep(1.0)
                continue
            
            # 更新状态显示
            current_state = status['state']
            current_progress = status['progress']
            
            # 状态变化时输出
            if current_state != last_state:
                print(f"{Fore.CYAN}Task state: {current_state}{Style.RESET_ALL}")
                last_state = current_state
            
            # 进度更新（每变化5%输出一次）
            if abs(current_progress - last_progress) >= 5.0:
                print(f"{Fore.GREEN}Progress: {current_progress:.1f}%{Style.RESET_ALL} "
                      f"({elapsed:.0f}s elapsed)")
                last_progress = current_progress
            
            # 详细输出模式
            if self.verbose and abs(current_progress - last_progress) >= 1.0:
                speed = self.robot_monitor.get_robot_speed()
                print(f"  [Detail] Progress: {current_progress:.1f}%, "
                      f"Linear: {speed['linear']:.3f} m/s, "
                      f"Angular: {speed['angular']:.3f} rad/s")
                last_progress = current_progress
            
            # 定期检查机器人运动
            if time.time() - last_motion_check > motion_check_interval:
                is_moving = self.robot_monitor.is_robot_moving(threshold=0.005)
                speed = self.robot_monitor.get_robot_speed()
                
                if current_state == 'RUNNING':
                    if is_moving:
                        print(f"{Fore.GREEN}✓ Robot is exploring{Style.RESET_ALL} "
                              f"(Linear: {speed['linear']:.3f} m/s, "
                              f"Angular: {speed['angular']:.3f} rad/s)")
                    else:
                        print(f"{Fore.YELLOW}⚠ Robot not moving{Style.RESET_ALL} "
                              f"(may be planning or repositioning)")
                
                last_motion_check = time.time()
            
            # 检查完成状态
            if current_state == 'COMPLETED':
                print(f"\n{Fore.GREEN}✓ Exploration completed!{Style.RESET_ALL}")
                print(f"  Final progress: {current_progress:.1f}%")
                print(f"  Total time: {elapsed:.1f}s")
                return True
            elif current_state == 'FAILED':
                print(f"\n{Fore.RED}✗ Exploration failed{Style.RESET_ALL}")
                print(f"  Message: {status['message']}")
                return False
            elif current_state == 'CANCELED':
                print(f"\n{Fore.YELLOW}⚠ Exploration was canceled{Style.RESET_ALL}")
                return False
            
            # 等待并更新ROS2
            time.sleep(1.0)
            for _ in range(10):
                rclpy.spin_once(self, timeout_sec=0.1)
    
    def emergency_stop(self):
        """紧急停止"""
        print(f"\n{Fore.RED}[Emergency Stop] Stopping exploration...{Style.RESET_ALL}")
        
        req = EmergencyStop.Request()
        req.reason = "User interrupted exploration test"
        
        success, response = self.call_service(self.emergency_stop_client, req)
        
        if success and response and response.success:
            print(f"{Fore.GREEN}✓ Emergency stop executed{Style.RESET_ALL}")
            print(f"  Stopped tasks: {response.stopped_tasks}")
        else:
            print(f"{Fore.RED}✗ Failed to execute emergency stop{Style.RESET_ALL}")
    
    def run_test(self):
        """运行完整测试"""
        try:
            # 等待服务
            if not self.wait_for_services():
                print(f"{Fore.RED}✗ Services not available{Style.RESET_ALL}")
                return False
            
            # 启动探索
            if not self.start_exploration():
                return False
            
            # 监控进度
            self.test_passed = self.monitor_exploration()
            
            # 输出结果
            print(f"\n{Fore.YELLOW}{'='*70}{Style.RESET_ALL}")
            if self.test_passed:
                print(f"{Fore.GREEN}✓ EXPLORATION MAPPING TEST PASSED{Style.RESET_ALL}")
                print(f"\n{Fore.CYAN}Map saved to:{Style.RESET_ALL}")
                print(f"  ~/lododo_bot/maps/{self.map_name}/rtabmap.db")
                print(f"  ~/lododo_bot/maps/{self.map_name}/map.pgm")
                print(f"  ~/lododo_bot/maps/{self.map_name}/map.yaml")
            else:
                print(f"{Fore.RED}✗ EXPLORATION MAPPING TEST FAILED{Style.RESET_ALL}")
            print(f"{Fore.YELLOW}{'='*70}{Style.RESET_ALL}\n")
            
            return self.test_passed
            
        except KeyboardInterrupt:
            print(f"\n{Fore.YELLOW}Test interrupted by user{Style.RESET_ALL}")
            if self.task_id:
                self.emergency_stop()
            return False
        except Exception as e:
            self.get_logger().error(f'Test exception: {str(e)}')
            import traceback
            traceback.print_exc()
            return False


def main():
    # 环境检查 / Environment check
    try:
        from bot_navigation_msgs.srv import StartExploration
    except ImportError:
        print(f"{Fore.RED}{'='*70}{Style.RESET_ALL}")
        print(f"{Fore.RED}Error: bot_navigation_msgs package not found!{Style.RESET_ALL}")
        print(f"{Fore.YELLOW}Please source the ROS2 workspace first:{Style.RESET_ALL}")
        print(f"  cd ~/lododo_bot")
        print(f"  source install/setup.bash")
        print(f"  python3 src/bot_navigation/scripts/test_exploration_mapping.py [options]")
        print(f"{Fore.RED}{'='*70}{Style.RESET_ALL}\n")
        sys.exit(1)
    
    # 解析命令行参数
    parser = argparse.ArgumentParser(
        description='Exploration Mapping Test Script - 探索建图功能测试',
        formatter_class=argparse.RawDescriptionHelpFormatter
    )
    
    parser.add_argument(
        '--map-name',
        type=str,
        default='exploration_test',
        help='Name of the map to create (default: exploration_test)'
    )
    
    parser.add_argument(
        '--duration',
        type=int,
        default=600,
        help='Maximum exploration duration in seconds (default: 600)'
    )
    
    parser.add_argument(
        '--coverage',
        type=float,
        default=0.85,
        help='Coverage threshold 0.0-1.0 (default: 0.85)'
    )
    
    parser.add_argument(
        '--verbose',
        action='store_true',
        help='Enable verbose output'
    )
    
    args = parser.parse_args()
    
    # 参数验证
    if args.coverage < 0.0 or args.coverage > 1.0:
        print(f"{Fore.RED}Error: Coverage threshold must be between 0.0 and 1.0{Style.RESET_ALL}")
        sys.exit(1)
    
    if args.duration < 60:
        print(f"{Fore.YELLOW}Warning: Duration less than 60s may be too short for exploration{Style.RESET_ALL}")
    
    # 初始化ROS2
    rclpy.init()
    
    # 创建测试节点
    test_node = ExplorationMappingTest(args)
    
    # 运行测试
    test_passed = test_node.run_test()
    
    # 清理
    test_node.destroy_node()
    rclpy.shutdown()
    
    # 返回退出码
    sys.exit(0 if test_passed else 1)


if __name__ == '__main__':
    main()
