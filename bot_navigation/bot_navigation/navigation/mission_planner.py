#!/usr/bin/env python3
"""
MissionPlanner - 任务规划器（重构版）

功能 / Features:
  - 任务调度与协调
  - 服务接口管理
  - 组件集成协调
  - 系统状态监控

重构说明:
  - 服务处理逻辑拆分到独立模块
  - 主文件只保留核心调度和集成逻辑
  - 提高可维护性和可扩展性

Author: LeKiwi Bot Development Team
Date: 2025-12-26
"""

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup

from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String, Bool

# 导入自定义服务消息
from bot_navigation_msgs.srv import (
    CreateTask, TaskControl, GetTaskStatus, ListTasks,
    StartExploration, StartPatrol, WaypointControl,
    RecordWaypoints, NavigateToPose, EmergencyStop
)
from std_srvs.srv import Trigger

from .task_manager import TaskManager, TaskType, TaskState
from ..patrol.patrol_manager import PatrolManager
from .waypoint_recorder import WaypointRecorder
from .navigation_executor import NavigationExecutor

# 导入服务处理模块
from .service_handlers import (
    WaypointServiceHandler,
    NavigationServiceHandler,
    TaskExecutionHandler
)

from typing import Optional, Dict, Any
from datetime import datetime


class MissionPlanner(Node):
    """
    任务规划器节点（重构版）
    
    系统的中央协调器，职责：
    - 接收外部命令（服务调用）
    - 任务创建和调度
    - 组件协调和状态管理
    - 委托具体任务执行给处理模块
    """
    
    def __init__(self):
        super().__init__('mission_planner')
        
        self.get_logger().info('Initializing MissionPlanner (Refactored)...')
        
        # ========== 参数配置 ==========
        self.declare_parameters(
            namespace='',
            parameters=[
                ('update_rate', 10.0),
                ('enable_auto_recovery', True),
                ('task_timeout', 300.0),
            ]
        )
        
        self._update_rate = self.get_parameter('update_rate').value
        self._enable_auto_recovery = self.get_parameter('enable_auto_recovery').value
        self._task_timeout = self.get_parameter('task_timeout').value
        
        # ========== 核心组件 ==========
        self._task_manager = TaskManager()
        self._patrol_manager = PatrolManager()
        self._waypoint_recorder = WaypointRecorder()
        self._navigation_executor = NavigationExecutor(self)
        
        # 加载持久化数据
        self._task_manager.load_active_tasks()
        self._patrol_manager.load_all_routes()
        
        # ========== WaypointRecorder 服务客户端 ==========
        waypoint_clients = self._create_waypoint_clients()
        
        # ========== 服务处理器 ==========
        self._waypoint_handler = WaypointServiceHandler(self, waypoint_clients)
        self._navigation_handler = NavigationServiceHandler(
            self, self._task_manager, self._navigation_executor
        )
        self._task_execution_handler = TaskExecutionHandler(
            self, self._task_manager, self._patrol_manager
        )
        
        # ========== 状态变量 ==========
        self._system_state = 'idle'  # idle, busy, error
        self._current_operation = None
        
        # ========== 回调组 ==========
        self._callback_group = ReentrantCallbackGroup()
        
        # ========== 发布器 ==========
        self._state_pub = self.create_publisher(String, '/mission_planner/state', 10)
        self._task_status_pub = self.create_publisher(String, '/mission_planner/task_status', 10)
        
        # ========== 服务接口 ==========
        self._create_service_servers()
        
        # ========== 定时器 ==========
        self._update_timer = self.create_timer(
            1.0 / self._update_rate,
            self._update_callback
        )
        
        self.get_logger().info('MissionPlanner initialized successfully')
    
    def _create_waypoint_clients(self) -> Dict:
        """创建 WaypointRecorder 服务客户端"""
        return {
            'save': self.create_client(WaypointControl, '/waypoint_recorder/save_waypoints'),
            'load': self.create_client(WaypointControl, '/waypoint_recorder/load_waypoints'),
            'record_current': self.create_client(Trigger, '/waypoint_recorder/record_current'),
            'start_recording': self.create_client(Trigger, '/waypoint_recorder/start_recording'),
            'stop_recording': self.create_client(Trigger, '/waypoint_recorder/stop_recording'),
            'clear': self.create_client(Trigger, '/waypoint_recorder/clear_waypoints'),
        }
    
    def _create_service_servers(self):
        """创建所有服务接口"""
        # 任务管理服务
        self._create_task_srv = self.create_service(
            CreateTask, '/mission/create_task',
            self._handle_create_task, callback_group=self._callback_group
        )
        self._start_task_srv = self.create_service(
            TaskControl, '/mission/start_task',
            self._handle_start_task, callback_group=self._callback_group
        )
        self._pause_task_srv = self.create_service(
            TaskControl, '/mission/pause_task',
            self._handle_pause_task, callback_group=self._callback_group
        )
        self._resume_task_srv = self.create_service(
            TaskControl, '/mission/resume_task',
            self._handle_resume_task, callback_group=self._callback_group
        )
        self._cancel_task_srv = self.create_service(
            TaskControl, '/mission/cancel_task',
            self._handle_cancel_task, callback_group=self._callback_group
        )
        self._get_status_srv = self.create_service(
            GetTaskStatus, '/mission/get_task_status',
            self._handle_get_task_status, callback_group=self._callback_group
        )
        self._list_tasks_srv = self.create_service(
            ListTasks, '/mission/list_tasks',
            self._handle_list_tasks, callback_group=self._callback_group
        )
        
        # 任务执行服务（委托给 TaskExecutionHandler）
        self._start_exploration_srv = self.create_service(
            StartExploration, '/mission/start_exploration',
            self._task_execution_handler.handle_start_exploration,
            callback_group=self._callback_group
        )
        self._start_patrol_srv = self.create_service(
            StartPatrol, '/mission/start_patrol',
            self._task_execution_handler.handle_start_patrol,
            callback_group=self._callback_group
        )
        
        # 路点服务（委托给 WaypointServiceHandler）
        self._waypoint_control_srv = self.create_service(
            WaypointControl, '/mission/waypoint_control',
            self._waypoint_handler.handle_waypoint_control,
            callback_group=self._callback_group
        )
        self._record_waypoints_srv = self.create_service(
            RecordWaypoints, '/mission/record_waypoints',
            self._waypoint_handler.handle_record_waypoints,
            callback_group=self._callback_group
        )
        
        # 导航服务（委托给 NavigationServiceHandler）
        self._navigate_to_pose_srv = self.create_service(
            NavigateToPose, '/mission/navigate_to_pose',
            self._navigation_handler.handle_navigate_to_pose,
            callback_group=self._callback_group
        )
        self._emergency_stop_srv = self.create_service(
            EmergencyStop, '/mission/emergency_stop',
            self._navigation_handler.handle_emergency_stop,
            callback_group=self._callback_group
        )
        
        self.get_logger().info('All service servers created')
    
    # ========== 任务管理服务处理（核心调度逻辑） ==========
    
    def _handle_create_task(self, request, response):
        """处理创建任务请求"""
        try:
            task = self._task_manager.create_task(
                task_id=request.task_id,
                task_type=TaskType(request.task_type),
                priority=request.priority,
                parameters=self._parse_parameters(request.parameters)
            )
            
            response.success = True
            response.message = f"Task '{request.task_id}' created"
            response.task_id = task.task_id
            
            self.get_logger().info(f"Created task: {request.task_id}")
            
        except Exception as e:
            response.success = False
            response.message = f"Failed to create task: {str(e)}"
            response.task_id = ""
            self.get_logger().error(f"Error creating task: {str(e)}")
        
        return response
    
    def _handle_start_task(self, request, response):
        """处理启动任务请求"""
        try:
            self._task_manager.start_task(request.task_id)
            response.success = True
            response.message = f"Task '{request.task_id}' started"
            self.get_logger().info(f"Started task: {request.task_id}")
        except Exception as e:
            response.success = False
            response.message = f"Failed to start task: {str(e)}"
            self.get_logger().error(f"Error starting task: {str(e)}")
        return response
    
    def _handle_pause_task(self, request, response):
        """处理暂停任务请求"""
        try:
            self._task_manager.pause_task(request.task_id)
            response.success = True
            response.message = f"Task '{request.task_id}' paused"
            self.get_logger().info(f"Paused task: {request.task_id}")
        except Exception as e:
            response.success = False
            response.message = f"Failed to pause task: {str(e)}"
            self.get_logger().error(f"Error pausing task: {str(e)}")
        return response
    
    def _handle_resume_task(self, request, response):
        """处理恢复任务请求"""
        try:
            self._task_manager.resume_task(request.task_id)
            response.success = True
            response.message = f"Task '{request.task_id}' resumed"
            self.get_logger().info(f"Resumed task: {request.task_id}")
        except Exception as e:
            response.success = False
            response.message = f"Failed to resume task: {str(e)}"
            self.get_logger().error(f"Error resuming task: {str(e)}")
        return response
    
    def _handle_cancel_task(self, request, response):
        """处理取消任务请求"""
        try:
            self._task_manager.cancel_task(request.task_id)
            response.success = True
            response.message = f"Task '{request.task_id}' cancelled"
            self.get_logger().info(f"Cancelled task: {request.task_id}")
        except Exception as e:
            response.success = False
            response.message = f"Failed to cancel task: {str(e)}"
            self.get_logger().error(f"Error cancelling task: {str(e)}")
        return response
    
    def _handle_get_task_status(self, request, response):
        """处理获取任务状态请求"""
        try:
            task = self._task_manager.get_task(request.task_id)
            if task:
                response.success = True
                response.task_id = task.task_id
                response.state = task.state.name
                response.progress = task.progress
                response.message = task.error_message if task.error_message else "Task running"
            else:
                response.success = False
                response.message = f"Task '{request.task_id}' not found"
        except Exception as e:
            response.success = False
            response.message = f"Error getting task status: {str(e)}"
            self.get_logger().error(f"Error getting task status: {str(e)}")
        
        return response
    
    def _handle_list_tasks(self, request, response):
        """处理列出任务请求"""
        try:
            if request.filter_state:
                filter_state = TaskState[request.filter_state.upper()]
                tasks = self._task_manager.get_tasks_by_state(filter_state)
            else:
                tasks = self._task_manager.get_all_tasks()
            
            response.success = True
            response.task_ids = [task.task_id for task in tasks]
            response.task_types = [task.task_type.name for task in tasks]
            response.states = [task.state.name for task in tasks]
            response.priorities = [task.priority for task in tasks]
            response.message = f"Found {len(tasks)} tasks"
            
        except Exception as e:
            response.success = False
            response.message = f"Error listing tasks: {str(e)}"
            response.task_ids = []
            response.task_types = []
            response.states = []
            response.priorities = []
            self.get_logger().error(f"Error listing tasks: {str(e)}")
        
        return response
    
    # ========== 任务执行调度 ==========
    
    def _update_callback(self):
        """定期更新回调 - 任务调度和状态发布"""
        try:
            # 执行待执行的任务
            self._execute_pending_tasks()
            
            # 发布系统状态
            self._publish_system_state()
            
            # 检查任务超时
            self._check_task_timeouts()
            
        except Exception as e:
            self.get_logger().error(f"Error in update callback: {str(e)}")
    
    def _execute_pending_tasks(self):
        """执行待执行的任务"""
        running_tasks = self._task_manager.get_tasks_by_state(TaskState.RUNNING)
        
        for task in running_tasks:
            if task.task_type == TaskType.FRONTIER_EXPLORATION:
                self._task_execution_handler.execute_exploration_task(task)
            elif task.task_type == TaskType.WAYPOINT_PATROL:
                self._task_execution_handler.execute_patrol_task(task)
            elif task.task_type == TaskType.POINT_TO_POINT:
                self._navigation_handler.execute_navigation_task(task)
    
    def _publish_system_state(self):
        """发布系统状态"""
        state_msg = String()
        state_msg.data = self._system_state
        self._state_pub.publish(state_msg)
    
    def _check_task_timeouts(self):
        """检查任务超时"""
        # TODO: 实现任务超时检查
        pass
    
    # ========== 辅助方法 ==========
    
    def _parse_parameters(self, param_string: str) -> Dict[str, Any]:
        """解析参数字符串为字典"""
        if not param_string:
            return {}
        try:
            import json
            return json.loads(param_string)
        except:
            return {}
    
    def shutdown(self):
        """关闭节点"""
        self.get_logger().info('Shutting down MissionPlanner...')
        self._task_execution_handler.stop_all_tasks()
        self._task_manager.save_active_tasks()


def main(args=None):
    rclpy.init(args=args)
    
    mission_planner = MissionPlanner()
    
    try:
        rclpy.spin(mission_planner)
    except KeyboardInterrupt:
        pass
    finally:
        mission_planner.shutdown()
        mission_planner.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
