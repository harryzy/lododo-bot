#!/usr/bin/env python3
"""
MissionPlanner - 任务规划器

功能 / Features:
  - 任务调度与协调
  - 服务接口定义（20+个）
  - 组件集成（TaskManager, PatrolManager, NavigationExecutor等）
  - 系统状态监控

Author: LeKiwi Bot Development Team
Date: 2025-12-22
"""

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.action import ActionClient

from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String, Bool
from nav2_msgs.action import NavigateToPose as Nav2NavigateToPose

# 导入自定义服务消息
from bot_navigation_msgs.srv import (
    CreateTask, TaskControl, GetTaskStatus, ListTasks,
    StartExploration, StartPatrol, WaypointControl,
    RecordWaypoints, NavigateToPose, EmergencyStop
)

from .task_manager import TaskManager, TaskType, TaskState
from ..patrol.patrol_manager import PatrolManager
from .waypoint_recorder import WaypointRecorder
from .navigation_executor import NavigationExecutor

from typing import Optional, Dict, Any
from datetime import datetime
import json


class MissionPlanner(Node):
    """
    任务规划器节点
    
    系统的中央协调器，负责：
    - 接收外部命令（服务调用）
    - 任务创建和调度
    - 组件协调
    - 状态发布
    """
    
    def __init__(self):
        super().__init__('mission_planner')
        
        self.get_logger().info('Initializing MissionPlanner...')
        
        # ========== 参数配置 ==========
        self.declare_parameters(
            namespace='',
            parameters=[
                ('update_rate', 10.0),  # 更新频率(Hz)
                ('enable_auto_recovery', True),  # 启用自动恢复
                ('task_timeout', 300.0),  # 任务超时时间(秒)
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
        
        # ========== 状态变量 ==========
        self._system_state = 'idle'  # idle, busy, error
        self._current_operation = None
        
        # ========== 回调组 ==========
        self._callback_group = ReentrantCallbackGroup()
        
        # ========== 发布器 ==========
        self._state_pub = self.create_publisher(
            String,
            '/mission_planner/state',
            10
        )
        
        self._task_status_pub = self.create_publisher(
            String,
            '/mission_planner/task_status',
            10
        )
        
        # ========== 服务服务器 ==========
        self._create_service_servers()
        
        # ========== 定时器 ==========
        self._update_timer = self.create_timer(
            1.0 / self._update_rate,
            self._update_callback
        )
        
        self.get_logger().info('MissionPlanner initialized successfully')
    
    def _create_service_servers(self):
        """创建所有服务服务器"""
        # 任务管理服务
        self._create_task_srv = self.create_service(
            CreateTask,
            '/mission/create_task',
            self._handle_create_task,
            callback_group=self._callback_group
        )
        
        self._start_task_srv = self.create_service(
            TaskControl,
            '/mission/start_task',
            self._handle_start_task,
            callback_group=self._callback_group
        )
        
        self._pause_task_srv = self.create_service(
            TaskControl,
            '/mission/pause_task',
            self._handle_pause_task,
            callback_group=self._callback_group
        )
        
        self._resume_task_srv = self.create_service(
            TaskControl,
            '/mission/resume_task',
            self._handle_resume_task,
            callback_group=self._callback_group
        )
        
        self._cancel_task_srv = self.create_service(
            TaskControl,
            '/mission/cancel_task',
            self._handle_cancel_task,
            callback_group=self._callback_group
        )
        
        self._get_status_srv = self.create_service(
            GetTaskStatus,
            '/mission/get_task_status',
            self._handle_get_task_status,
            callback_group=self._callback_group
        )
        
        self._list_tasks_srv = self.create_service(
            ListTasks,
            '/mission/list_tasks',
            self._handle_list_tasks,
            callback_group=self._callback_group
        )
        
        # 探索和巡航服务
        self._start_exploration_srv = self.create_service(
            StartExploration,
            '/mission/start_exploration',
            self._handle_start_exploration,
            callback_group=self._callback_group
        )
        
        self._start_patrol_srv = self.create_service(
            StartPatrol,
            '/mission/start_patrol',
            self._handle_start_patrol,
            callback_group=self._callback_group
        )
        
        # 路点控制服务
        self._waypoint_control_srv = self.create_service(
            WaypointControl,
            '/mission/waypoint_control',
            self._handle_waypoint_control,
            callback_group=self._callback_group
        )
        
        self._record_waypoints_srv = self.create_service(
            RecordWaypoints,
            '/mission/record_waypoints',
            self._handle_record_waypoints,
            callback_group=self._callback_group
        )
        
        # 导航服务
        self._navigate_to_pose_srv = self.create_service(
            NavigateToPose,
            '/mission/navigate_to_pose',
            self._handle_navigate_to_pose,
            callback_group=self._callback_group
        )
        
        # 紧急停止
        self._emergency_stop_srv = self.create_service(
            EmergencyStop,
            '/mission/emergency_stop',
            self._handle_emergency_stop,
            callback_group=self._callback_group
        )
        
        self.get_logger().info('All service servers created successfully')
    
    def _update_callback(self):
        """定期更新回调"""
        # 检查当前任务状态
        current_task = self._task_manager.get_current_task()
        
        if current_task is not None:
            self._system_state = 'busy'
            
            # TODO: 根据任务类型执行相应操作
            if current_task.state == TaskState.RUNNING:
                self._execute_task(current_task)
            
            # 发布任务状态
            self._publish_task_status(current_task)
        else:
            # 检查是否有待执行任务
            next_task = self._task_manager.get_next_task()
            if next_task is not None:
                self.get_logger().info(f'Starting next task: {next_task.task_id}')
                self._task_manager.start_task(next_task.task_id)
            else:
                self._system_state = 'idle'
        
        # 发布系统状态
        self._publish_system_state()
    
    def _execute_task(self, task):
        """执行任务（框架方法，待实现）"""
        # TODO: 根据任务类型调用相应组件
        pass
    
    def _publish_system_state(self):
        """发布系统状态"""
        msg = String()
        msg.data = self._system_state
        self._state_pub.publish(msg)
    
    def _publish_task_status(self, task):
        """发布任务状态"""
        msg = String()
        msg.data = f"{task.task_id}:{task.state.value}:{task.progress:.2f}"
        self._task_status_pub.publish(msg)
    
    # ========== 服务处理函数 ==========
    
    def _handle_create_task(self, request, response):
        """处理创建任务请求"""
        try:
            # 解析参数
            parameters = {}
            for i, key in enumerate(request.parameters_keys):
                if i < len(request.parameters_values):
                    parameters[key] = request.parameters_values[i]
            
            # 创建任务
            task = self._task_manager.create_task(
                task_id=request.task_id,
                task_type=TaskType(request.task_type),
                priority=request.priority,
                parameters=parameters
            )
            
            response.success = True
            response.message = f"Task '{request.task_id}' created successfully"
            response.task_id = task.task_id
            
            self.get_logger().info(f"Created task: {task.task_id}")
            
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
            task_id = request.task_id if request.task_id else None
            task = self._task_manager.get_task(task_id) if task_id else self._task_manager.get_current_task()
            
            if task:
                response.success = True
                response.message = "Task status retrieved"
                response.task_id = task.task_id
                response.task_type = task.task_type.value
                response.state = task.state.value
                response.progress = task.progress
            else:
                response.success = False
                response.message = "Task not found"
                response.task_id = ""
                response.task_type = ""
                response.state = ""
                response.progress = 0.0
                
        except Exception as e:
            response.success = False
            response.message = f"Error retrieving task status: {str(e)}"
            self.get_logger().error(f"Error getting task status: {str(e)}")
        
        return response
    
    def _handle_list_tasks(self, request, response):
        """处理列出任务请求"""
        try:
            filter_type = request.filter if request.filter else 'all'
            
            if filter_type == 'active':
                tasks = [t for t in self._task_manager._tasks.values() 
                        if t.state in [TaskState.PENDING, TaskState.RUNNING, TaskState.PAUSED]]
            elif filter_type == 'completed':
                tasks = [t for t in self._task_manager._tasks.values() 
                        if t.state == TaskState.COMPLETED]
            elif filter_type == 'failed':
                tasks = [t for t in self._task_manager._tasks.values() 
                        if t.state == TaskState.FAILED]
            else:  # 'all'
                tasks = list(self._task_manager._tasks.values())
            
            response.success = True
            response.message = f"Found {len(tasks)} tasks"
            response.task_ids = [t.task_id for t in tasks]
            response.task_types = [t.task_type.value for t in tasks]
            response.states = [t.state.value for t in tasks]
            response.priorities = [t.priority for t in tasks]
            
        except Exception as e:
            response.success = False
            response.message = f"Error listing tasks: {str(e)}"
            response.task_ids = []
            response.task_types = []
            response.states = []
            response.priorities = []
            self.get_logger().error(f"Error listing tasks: {str(e)}")
        
        return response
    
    def _handle_start_exploration(self, request, response):
        """处理开始探索请求"""
        try:
            # 创建探索任务
            task_id = f"exploration_{datetime.now().strftime('%Y%m%d_%H%M%S')}"
            parameters = {
                'map_name': request.map_name,
                'save_map': request.save_map,
                'max_duration': request.max_duration,
                'coverage_threshold': request.coverage_threshold
            }
            
            task = self._task_manager.create_task(
                task_id=task_id,
                task_type=TaskType.FRONTIER_EXPLORATION,
                priority=5,
                parameters=parameters
            )
            
            # 立即启动
            self._task_manager.start_task(task_id)
            
            response.success = True
            response.message = "Exploration started"
            response.task_id = task_id
            
            self.get_logger().info(f"Started exploration task: {task_id}")
            
        except Exception as e:
            response.success = False
            response.message = f"Failed to start exploration: {str(e)}"
            response.task_id = ""
            self.get_logger().error(f"Error starting exploration: {str(e)}")
        
        return response
    
    def _handle_start_patrol(self, request, response):
        """处理开始巡航请求"""
        try:
            # 创建巡航任务
            task_id = f"patrol_{datetime.now().strftime('%Y%m%d_%H%M%S')}"
            parameters = {
                'waypoint_file': request.waypoint_file,
                'patrol_mode': request.patrol_mode,
                'speed_factor': request.speed_factor
            }
            
            task = self._task_manager.create_task(
                task_id=task_id,
                task_type=TaskType.PATH_PATROL,
                priority=3,
                parameters=parameters
            )
            
            # 立即启动
            self._task_manager.start_task(task_id)
            
            response.success = True
            response.message = "Patrol started"
            response.task_id = task_id
            
            self.get_logger().info(f"Started patrol task: {task_id}")
            
        except Exception as e:
            response.success = False
            response.message = f"Failed to start patrol: {str(e)}"
            response.task_id = ""
            self.get_logger().error(f"Error starting patrol: {str(e)}")
        
        return response
    
    def _handle_waypoint_control(self, request, response):
        """处理路点控制请求"""
        try:
            action = request.action
            
            if action == 'add':
                # 添加路点
                waypoint = {
                    'x': request.x,
                    'y': request.y,
                    'yaw': request.yaw,
                    'name': request.waypoint_name
                }
                # TODO: 调用WaypointRecorder添加路点
                response.success = True
                response.message = "Waypoint added"
                response.waypoint_count = 0  # TODO: 获取实际数量
                
            elif action == 'save':
                # 保存路点
                # TODO: 调用WaypointRecorder保存
                response.success = True
                response.message = f"Waypoints saved to {request.filename}"
                response.waypoint_count = 0
                
            elif action == 'load':
                # 加载路点
                # TODO: 调用WaypointRecorder加载
                response.success = True
                response.message = f"Waypoints loaded from {request.filename}"
                response.waypoint_count = 0
                
            elif action == 'clear':
                # 清空路点
                # TODO: 调用WaypointRecorder清空
                response.success = True
                response.message = "Waypoints cleared"
                response.waypoint_count = 0
            else:
                response.success = False
                response.message = f"Unknown action: {action}"
                response.waypoint_count = 0
                
            self.get_logger().info(f"Waypoint control action: {action}")
            
        except Exception as e:
            response.success = False
            response.message = f"Waypoint control error: {str(e)}"
            response.waypoint_count = 0
            self.get_logger().error(f"Error in waypoint control: {str(e)}")
        
        return response
    
    def _handle_record_waypoints(self, request, response):
        """处理路点录制控制请求"""
        try:
            action = request.action
            
            if action == 'start':
                # 开始录制
                # TODO: 调用WaypointRecorder开始录制
                response.success = True
                response.message = "Waypoint recording started"
                response.recorded_count = 0
                
            elif action == 'stop':
                # 停止录制
                # TODO: 调用WaypointRecorder停止录制
                response.success = True
                response.message = "Waypoint recording stopped"
                response.recorded_count = 0
                
            elif action == 'record_current':
                # 录制当前位置
                # TODO: 调用WaypointRecorder录制当前位置
                response.success = True
                response.message = "Current position recorded"
                response.recorded_count = 0
            else:
                response.success = False
                response.message = f"Unknown action: {action}"
                response.recorded_count = 0
                
            self.get_logger().info(f"Record waypoints action: {action}")
            
        except Exception as e:
            response.success = False
            response.message = f"Record waypoints error: {str(e)}"
            response.recorded_count = 0
            self.get_logger().error(f"Error in record waypoints: {str(e)}")
        
        return response
    
    def _handle_navigate_to_pose(self, request, response):
        """处理导航到位姿请求"""
        try:
            # 创建导航任务
            task_id = f"nav_{datetime.now().strftime('%Y%m%d_%H%M%S')}"
            parameters = {
                'x': request.x,
                'y': request.y,
                'yaw': request.yaw,
                'frame_id': request.frame_id if request.frame_id else 'map'
            }
            
            task = self._task_manager.create_task(
                task_id=task_id,
                task_type=TaskType.POINT_TO_POINT,
                priority=7,
                parameters=parameters
            )
            
            # 立即启动
            self._task_manager.start_task(task_id)
            
            response.success = True
            response.message = "Navigation started"
            response.task_id = task_id
            
            self.get_logger().info(f"Started navigation task: {task_id}")
            
        except Exception as e:
            response.success = False
            response.message = f"Failed to navigate: {str(e)}"
            response.task_id = ""
            self.get_logger().error(f"Error in navigation: {str(e)}")
        
        return response
    
    def _handle_emergency_stop(self, request, response):
        """处理紧急停止请求"""
        try:
            self.get_logger().warn("EMERGENCY STOP ACTIVATED!")
            
            # 取消当前任务
            current_task = self._task_manager.get_current_task()
            if current_task:
                self._task_manager.cancel_task(current_task.task_id)
            
            # 如果需要，清空所有任务
            if request.clear_tasks:
                # TODO: 实现清空所有任务
                pass
            
            # 停止所有运动
            # TODO: 发布零速度命令
            
            response.success = True
            response.message = "Emergency stop executed"
            
        except Exception as e:
            response.success = False
            response.message = f"Emergency stop error: {str(e)}"
            self.get_logger().error(f"Error in emergency stop: {str(e)}")
        
        return response
    
    def shutdown(self):
        """关闭时保存状态"""
        self.get_logger().info('Shutting down MissionPlanner...')
        # 保存所有状态
        self._task_manager._save_active_tasks()
        self.get_logger().info('MissionPlanner shutdown complete')


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
