#!/usr/bin/env python3
"""
CMD Interface Mock Terminal
交互式命令行终端 - 演示统一命令接口

Features:
  - Interactive command-line interface
  - Command auto-completion
  - Command history
  - Real-time status display
  - Colored output
  - Help system

Usage:
  python3 cmd_terminal.py

Prerequisites:
  ros2 launch bot_bringup simulation_cmd_interface_test.launch.py slam:=false map_name:=exploration_test

Author: LeKiwi Bot Development Team
Date: 2026-01-08
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import sys
import time
import threading
from typing import Dict, List, Optional
from collections import defaultdict, deque
import readline  # For command history and editing
import atexit
import os

from bot_cmd_interface.sdk.message import CommandRequest, CommandResponse
from bot_cmd_interface.sdk.action_types import ActionType
from bot_cmd_interface.sdk.message import ErrorCode, ResponseStatus

# 颜色定义
try:
    from colorama import Fore, Style, Back, init
    init(autoreset=True)
    HAS_COLOR = True
except ImportError:
    HAS_COLOR = False
    class Fore:
        GREEN = RED = YELLOW = CYAN = MAGENTA = BLUE = WHITE = BLACK = ''
    class Style:
        BRIGHT = RESET_ALL = DIM = ''
    class Back:
        GREEN = RED = YELLOW = CYAN = MAGENTA = BLUE = WHITE = BLACK = ''


class CommandHistory:
    """命令历史管理"""
    
    def __init__(self, max_size=100):
        self.history = deque(maxlen=max_size)
        self.history_file = os.path.expanduser('~/.cmd_terminal_history')
        self._load_history()
    
    def add(self, command: str):
        if command and command.strip():
            self.history.append(command)
            self._save_history()
    
    def _load_history(self):
        """从文件加载历史"""
        try:
            if os.path.exists(self.history_file):
                with open(self.history_file, 'r') as f:
                    for line in f:
                        readline.add_history(line.strip())
                        self.history.append(line.strip())
        except Exception:
            pass
    
    def _save_history(self):
        """保存历史到文件"""
        try:
            with open(self.history_file, 'w') as f:
                for cmd in self.history:
                    f.write(cmd + '\n')
        except Exception:
            pass


class TerminalNode(Node):
    """终端节点"""
    
    def __init__(self):
        super().__init__('cmd_terminal')
        
        # 发布器和订阅器
        self.publisher = self.create_publisher(String, '/cmd/request', 10)
        self.subscription = self.create_subscription(
            String,
            '/cmd/response',
            self._response_callback,
            10
        )
        
        # 存储请求和响应
        self.pending_requests = {}  # {request_id: {'command': str, 'send_time': float}}
        self.recent_responses = deque(maxlen=50)
        self.lock = threading.Lock()
        
        # 统计
        self.total_sent = 0
        self.total_received = 0
        self.total_completed = 0
        self.total_failed = 0
        
        # 后台线程处理ROS2消息
        self.running = True
        self.spin_thread = threading.Thread(target=self._spin_loop, daemon=True)
        self.spin_thread.start()
    
    def _response_callback(self, msg: String):
        """接收响应"""
        try:
            response = CommandResponse.from_json(msg.data)
            
            with self.lock:
                self.recent_responses.append(response)
                self.total_received += 1
                
                if response.status == 'completed':
                    self.total_completed += 1
                elif response.status == 'failed':
                    self.total_failed += 1
        
        except Exception as e:
            self.get_logger().error(f'Failed to parse response: {e}')
    
    def send_request(self, action: str, params: dict, priority: int = 3, timeout: float = 300.0) -> str:
        """发送请求"""
        request = CommandRequest(
            action=action,
            params=params,
            priority=priority,
            timeout=timeout
        )
        
        msg = String()
        msg.data = request.to_json()
        
        with self.lock:
            self.pending_requests[request.request_id] = {
                'command': f"{action} {params}",
                'send_time': time.time()
            }
            self.total_sent += 1
        
        self.publisher.publish(msg)
        return request.request_id
    
    def get_recent_response(self, request_id: str, status: Optional[str] = None) -> Optional[CommandResponse]:
        """获取最近的响应"""
        with self.lock:
            for response in reversed(self.recent_responses):
                if response.request_id == request_id:
                    if status is None or response.status == status:
                        return response
        return None
    
    def get_statistics(self) -> dict:
        """获取统计信息"""
        with self.lock:
            return {
                'sent': self.total_sent,
                'received': self.total_received,
                'completed': self.total_completed,
                'failed': self.total_failed,
                'pending': len(self.pending_requests)
            }
    
    def _spin_loop(self):
        """后台循环处理ROS2消息"""
        while self.running and rclpy.ok():
            try:
                rclpy.spin_once(self, timeout_sec=0.1)
            except Exception:
                break
    
    def shutdown(self):
        """关闭节点"""
        self.running = False
        if self.spin_thread.is_alive():
            self.spin_thread.join(timeout=2.0)


class MockTerminal:
    """交互式终端"""
    
    def __init__(self, node: TerminalNode):
        self.node = node
        self.history = CommandHistory()
        self.running = True
        
        # 命令映射
        self.commands = {
            'nav': self.cmd_navigate,
            'navigate': self.cmd_navigate,
            'goto': self.cmd_navigate,
            'explore': self.cmd_explore,
            'patrol': self.cmd_patrol,
            'stop': self.cmd_emergency_stop,
            'emergency': self.cmd_emergency_stop,
            'status': self.cmd_status,
            'task': self.cmd_task_status,
            'pause': self.cmd_pause,
            'resume': self.cmd_resume,
            'cancel': self.cmd_cancel,
            'stats': self.cmd_statistics,
            'help': self.cmd_help,
            'exit': self.cmd_exit,
            'quit': self.cmd_exit,
            'clear': self.cmd_clear,
            'history': self.cmd_history,
        }
        
        # 设置自动补全
        self._setup_completion()
    
    def _setup_completion(self):
        """设置命令自动补全"""
        def completer(text, state):
            options = [cmd for cmd in self.commands.keys() if cmd.startswith(text)]
            if state < len(options):
                return options[state]
            return None
        
        readline.set_completer(completer)
        readline.parse_and_bind('tab: complete')
    
    def print_banner(self):
        """打印欢迎信息"""
        print(f"\n{Fore.CYAN}{Style.BRIGHT}{'='*60}")
        print(f"  LeKiwi Robot - Unified Command Interface")
        print(f"  统一命令接口 - 交互式终端")
        print(f"{'='*60}{Style.RESET_ALL}\n")
        
        print(f"{Fore.GREEN}Connected to CommandAdapter{Style.RESET_ALL}")
        print(f"{Fore.YELLOW}Type 'help' for available commands{Style.RESET_ALL}")
        print(f"{Fore.YELLOW}Type 'exit' or 'quit' to exit{Style.RESET_ALL}\n")
    
    def run(self):
        """运行终端"""
        self.print_banner()
        
        while self.running:
            try:
                # 显示提示符
                prompt = f"{Fore.CYAN}cmd>{Style.RESET_ALL} "
                user_input = input(prompt).strip()
                
                if not user_input:
                    continue
                
                # 添加到历史
                self.history.add(user_input)
                
                # 解析命令
                parts = user_input.split()
                cmd = parts[0].lower()
                args = parts[1:]
                
                # 执行命令
                if cmd in self.commands:
                    try:
                        self.commands[cmd](args)
                    except Exception as e:
                        print(f"{Fore.RED}Error executing command: {e}{Style.RESET_ALL}")
                else:
                    print(f"{Fore.RED}Unknown command: {cmd}{Style.RESET_ALL}")
                    print(f"{Fore.YELLOW}Type 'help' for available commands{Style.RESET_ALL}")
            
            except KeyboardInterrupt:
                print(f"\n{Fore.YELLOW}Use 'exit' or 'quit' to exit{Style.RESET_ALL}")
            except EOFError:
                break
    
    # ========== 命令实现 ==========
    
    def cmd_navigate(self, args):
        """导航到指定位置
        Usage: nav <x> <y> [yaw]
        Example: nav 2.0 3.0 1.57
        """
        if len(args) < 2:
            print(f"{Fore.RED}Usage: nav <x> <y> [yaw]{Style.RESET_ALL}")
            return
        
        try:
            x = float(args[0])
            y = float(args[1])
            yaw = float(args[2]) if len(args) > 2 else 0.0
            
            print(f"{Fore.CYAN}➜ Sending navigation request to ({x}, {y}, {yaw})...{Style.RESET_ALL}")
            
            request_id = self.node.send_request(
                ActionType.NAVIGATE_TO_POSE,
                {'x': x, 'y': y, 'yaw': yaw}
            )
            
            # 等待响应
            self._wait_and_show_response(request_id, action_name="Navigate")
        
        except ValueError:
            print(f"{Fore.RED}Invalid coordinates. Use numbers.{Style.RESET_ALL}")
    
    def cmd_explore(self, args):
        """开始探索
        Usage: explore [map_name]
        Example: explore office_map
        """
        map_name = args[0] if args else f'explore_{int(time.time())}'
        
        print(f"{Fore.CYAN}➜ Starting exploration (map: {map_name})...{Style.RESET_ALL}")
        
        request_id = self.node.send_request(
            ActionType.START_EXPLORATION,
            {
                'map_name': map_name,
                'save_map': True,
                'max_duration': 300.0,
                'coverage_threshold': 0.8
            }
        )
        
        self._wait_and_show_response(request_id, action_name="Explore")
    
    def cmd_patrol(self, args):
        """开始巡航
        Usage: patrol <waypoint_file> [mode]
        Example: patrol route1.yaml loop
        """
        if len(args) < 1:
            print(f"{Fore.RED}Usage: patrol <waypoint_file> [mode]{Style.RESET_ALL}")
            print(f"{Fore.YELLOW}Modes: once, loop, pingpong{Style.RESET_ALL}")
            return
        
        waypoint_file = args[0]
        mode = args[1] if len(args) > 1 else 'loop'
        
        print(f"{Fore.CYAN}➜ Starting patrol (file: {waypoint_file}, mode: {mode})...{Style.RESET_ALL}")
        
        request_id = self.node.send_request(
            ActionType.START_PATROL,
            {
                'waypoint_file': waypoint_file,
                'mode': mode,
                'speed_scale': 1.0
            }
        )
        
        self._wait_and_show_response(request_id, action_name="Patrol")
    
    def cmd_emergency_stop(self, args):
        """紧急停止
        Usage: stop [clear]
        Example: stop clear
        """
        clear_tasks = 'clear' in args
        
        print(f"{Fore.RED}{Style.BRIGHT}➜ EMERGENCY STOP!{Style.RESET_ALL}")
        
        request_id = self.node.send_request(
            ActionType.EMERGENCY_STOP,
            {'clear_tasks': clear_tasks},
            priority=1  # 最高优先级
        )
        
        self._wait_and_show_response(request_id, action_name="Emergency Stop", timeout=5.0)
    
    def cmd_status(self, args):
        """查询机器人状态
        Usage: status
        """
        print(f"{Fore.CYAN}➜ Querying robot status...{Style.RESET_ALL}")
        
        request_id = self.node.send_request(
            ActionType.GET_ROBOT_STATUS,
            {}
        )
        
        self._wait_and_show_response(request_id, action_name="Status", timeout=3.0)
    
    def cmd_task_status(self, args):
        """查询任务状态
        Usage: task <task_id>
        Example: task nav_20260108_123456
        """
        if len(args) < 1:
            print(f"{Fore.RED}Usage: task <task_id>{Style.RESET_ALL}")
            return
        
        task_id = args[0]
        
        print(f"{Fore.CYAN}➜ Querying task status: {task_id}...{Style.RESET_ALL}")
        
        request_id = self.node.send_request(
            ActionType.GET_TASK_STATUS,
            {'task_id': task_id}
        )
        
        self._wait_and_show_response(request_id, action_name="Task Status", timeout=3.0)
    
    def cmd_pause(self, args):
        """暂停任务
        Usage: pause <task_id>
        """
        if len(args) < 1:
            print(f"{Fore.RED}Usage: pause <task_id>{Style.RESET_ALL}")
            return
        
        task_id = args[0]
        print(f"{Fore.CYAN}➜ Pausing task: {task_id}...{Style.RESET_ALL}")
        
        request_id = self.node.send_request(
            ActionType.PAUSE_TASK,
            {'task_id': task_id}
        )
        
        self._wait_and_show_response(request_id, action_name="Pause", timeout=3.0)
    
    def cmd_resume(self, args):
        """恢复任务
        Usage: resume <task_id>
        """
        if len(args) < 1:
            print(f"{Fore.RED}Usage: resume <task_id>{Style.RESET_ALL}")
            return
        
        task_id = args[0]
        print(f"{Fore.CYAN}➜ Resuming task: {task_id}...{Style.RESET_ALL}")
        
        request_id = self.node.send_request(
            ActionType.RESUME_TASK,
            {'task_id': task_id}
        )
        
        self._wait_and_show_response(request_id, action_name="Resume", timeout=3.0)
    
    def cmd_cancel(self, args):
        """取消任务
        Usage: cancel <task_id>
        """
        if len(args) < 1:
            print(f"{Fore.RED}Usage: cancel <task_id>{Style.RESET_ALL}")
            return
        
        task_id = args[0]
        print(f"{Fore.CYAN}➜ Cancelling task: {task_id}...{Style.RESET_ALL}")
        
        request_id = self.node.send_request(
            ActionType.CANCEL_TASK,
            {'task_id': task_id}
        )
        
        self._wait_and_show_response(request_id, action_name="Cancel", timeout=3.0)
    
    def cmd_statistics(self, args):
        """显示统计信息
        Usage: stats
        """
        stats = self.node.get_statistics()
        
        print(f"\n{Fore.CYAN}{'='*50}")
        print(f"  Statistics / 统计信息")
        print(f"{'='*50}{Style.RESET_ALL}")
        
        print(f"{Fore.GREEN}Requests Sent:     {stats['sent']}{Style.RESET_ALL}")
        print(f"{Fore.GREEN}Responses Received: {stats['received']}{Style.RESET_ALL}")
        print(f"{Fore.GREEN}Completed:         {stats['completed']}{Style.RESET_ALL}")
        print(f"{Fore.RED}Failed:            {stats['failed']}{Style.RESET_ALL}")
        print(f"{Fore.YELLOW}Pending:           {stats['pending']}{Style.RESET_ALL}")
        
        success_rate = stats['completed'] / stats['sent'] * 100 if stats['sent'] > 0 else 0
        print(f"\n{Fore.CYAN}Success Rate:      {success_rate:.1f}%{Style.RESET_ALL}")
        print()
    
    def cmd_help(self, args):
        """显示帮助信息
        Usage: help [command]
        """
        if args:
            # 显示特定命令的帮助
            cmd = args[0].lower()
            if cmd in self.commands:
                doc = self.commands[cmd].__doc__
                if doc:
                    print(f"\n{Fore.CYAN}{doc}{Style.RESET_ALL}\n")
                else:
                    print(f"{Fore.YELLOW}No help available for '{cmd}'{Style.RESET_ALL}")
            else:
                print(f"{Fore.RED}Unknown command: {cmd}{Style.RESET_ALL}")
        else:
            # 显示所有命令
            print(f"\n{Fore.CYAN}{'='*50}")
            print(f"  Available Commands / 可用命令")
            print(f"{'='*50}{Style.RESET_ALL}\n")
            
            print(f"{Fore.GREEN}Navigation:{Style.RESET_ALL}")
            print(f"  nav, goto       - Navigate to position (导航)")
            print(f"  explore         - Start exploration (探索)")
            print(f"  patrol          - Start patrol (巡航)")
            print(f"  stop, emergency - Emergency stop (紧急停止)")
            
            print(f"\n{Fore.GREEN}Task Control:{Style.RESET_ALL}")
            print(f"  pause           - Pause task (暂停)")
            print(f"  resume          - Resume task (恢复)")
            print(f"  cancel          - Cancel task (取消)")
            
            print(f"\n{Fore.GREEN}Query:{Style.RESET_ALL}")
            print(f"  status          - Robot status (状态)")
            print(f"  task            - Task status (任务状态)")
            print(f"  stats           - Statistics (统计)")
            
            print(f"\n{Fore.GREEN}Utility:{Style.RESET_ALL}")
            print(f"  help            - Show help (帮助)")
            print(f"  history         - Show command history (历史)")
            print(f"  clear           - Clear screen (清屏)")
            print(f"  exit, quit      - Exit terminal (退出)")
            
            print(f"\n{Fore.YELLOW}Tip: Use Tab for auto-completion{Style.RESET_ALL}")
            print(f"{Fore.YELLOW}Tip: Type 'help <command>' for detailed usage{Style.RESET_ALL}\n")
    
    def cmd_exit(self, args):
        """退出终端
        Usage: exit
        """
        print(f"\n{Fore.CYAN}Goodbye! 再见！{Style.RESET_ALL}\n")
        self.running = False
    
    def cmd_clear(self, args):
        """清屏
        Usage: clear
        """
        os.system('clear' if os.name != 'nt' else 'cls')
        self.print_banner()
    
    def cmd_history(self, args):
        """显示命令历史
        Usage: history [n]
        """
        count = int(args[0]) if args and args[0].isdigit() else 10
        
        print(f"\n{Fore.CYAN}Recent Commands:{Style.RESET_ALL}")
        for i, cmd in enumerate(list(self.history.history)[-count:], 1):
            print(f"  {i}. {cmd}")
        print()
    
    def _wait_and_show_response(self, request_id: str, action_name: str = "Request", timeout: float = 3.0):
        """等待并显示响应"""
        start_time = time.time()
        shown_statuses = set()
        
        while time.time() - start_time < timeout:
            # 检查所有响应
            response = self.node.get_recent_response(request_id)
            
            if response and response.status not in shown_statuses:
                shown_statuses.add(response.status)
                
                # 显示响应
                if response.status == 'queued':
                    print(f"{Fore.YELLOW}  ⟳ Queued{Style.RESET_ALL}")
                
                elif response.status == 'executing':
                    print(f"{Fore.CYAN}  ⟳ Executing{Style.RESET_ALL}")
                
                elif response.status == 'completed':
                    print(f"{Fore.GREEN}  ✓ Completed{Style.RESET_ALL}")
                    
                    # 显示结果
                    if response.result:
                        for key, value in response.result.items():
                            print(f"    {key}: {value}")
                    
                    if response.message:
                        print(f"    {response.message}")
                    
                    return
                
                elif response.status == 'failed':
                    print(f"{Fore.RED}  ✗ Failed{Style.RESET_ALL}")
                    print(f"    {response.message}")
                    return
            
            time.sleep(0.1)
        
        print(f"{Fore.YELLOW}  ⏱ Timeout waiting for response{Style.RESET_ALL}")


def main():
    """主函数"""
    # 初始化ROS2
    rclpy.init()
    
    try:
        # 创建节点
        node = TerminalNode()
        time.sleep(1.0)  # 等待初始化
        
        # 检查CommandAdapter
        if 'command_adapter' not in node.get_node_names():
            print(f"{Fore.RED}ERROR: CommandAdapter not found!{Style.RESET_ALL}")
            print(f"{Fore.YELLOW}Please start the system first:{Style.RESET_ALL}")
            print("  ros2 launch bot_bringup simulation_cmd_interface_test.launch.py slam:=false map_name:=exploration_test")
            return 1
        
        # 创建并运行终端
        terminal = MockTerminal(node)
        
        # 注册清理函数
        def cleanup():
            node.shutdown()
        
        atexit.register(cleanup)
        
        # 运行终端
        terminal.run()
        
        return 0
    
    except KeyboardInterrupt:
        print(f"\n{Fore.YELLOW}Interrupted by user{Style.RESET_ALL}")
        return 0
    
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
