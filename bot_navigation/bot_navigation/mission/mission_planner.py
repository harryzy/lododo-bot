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

import os
import subprocess
import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup

from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String, Bool

# 导入自定义服务消息
from bot_navigation_msgs.srv import (
    CreateTask, TaskControl, GetTaskStatus, ListTasks, ClearTasks,
    StartExploration, StartPatrol, WaypointControl,
    RecordWaypoints, NavigateToPose, EmergencyStop
)
from std_srvs.srv import Trigger

from .task_manager import TaskManager, TaskType, TaskState
from ..patrol.patrol_manager import PatrolManager
from ..waypoint.waypoint_recorder import WaypointRecorder
from .navigation_executor import NavigationExecutor, NavigationState

# 导入工具模块和任务处理器
from ..waypoint.waypoint_service import WaypointTools
from .service_handlers.handlers import (
    NavigationHandler,
    PatrolHandler,
    ExplorationHandler  # Phase 3
)

import math

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
                ('persistence_dir', ''),  # 持久化目录，空则使用默认值
            ]
        )
        
        self._update_rate = self.get_parameter('update_rate').value
        self._enable_auto_recovery = self.get_parameter('enable_auto_recovery').value
        self._task_timeout = self.get_parameter('task_timeout').value
        
        # 持久化目录配置
        persistence_dir = self.get_parameter('persistence_dir').value
        if not persistence_dir:
            # 默认使用工作空间下的mission目录
            import subprocess
            try:
                ws_path = subprocess.check_output(
                    ['ros2', 'pkg', 'prefix', 'bot_navigation'],
                    text=True
                ).strip()
                # 从install/bot_navigation返回到工作空间根目录
                ws_root = os.path.abspath(os.path.join(ws_path, '..', '..'))
                persistence_dir = os.path.join(ws_root, 'mission')
            except Exception as e:
                self.get_logger().warn(f'Failed to get workspace path: {e}, using fallback')
                persistence_dir = os.path.expanduser('~/lododo_bot/mission')
        else:
            persistence_dir = os.path.expanduser(persistence_dir)
        
        os.makedirs(persistence_dir, exist_ok=True)
        self.get_logger().info(f'Mission persistence directory: {persistence_dir}')
        
        # 创建子目录
        tasks_dir = os.path.join(persistence_dir, 'tasks')
        patrol_dir = os.path.join(persistence_dir, 'patrol_routes')
        waypoints_dir = os.path.join(persistence_dir, 'waypoints')
        
        # ========== 核心组件 ==========
        self._task_manager = TaskManager(persistence_dir=tasks_dir)
        
        # 先创建NavigationExecutor（PatrolManager需要依赖它）
        self._navigation_executor = NavigationExecutor(self)
        
        # 创建PatrolManager，传入node和navigation_executor
        self._patrol_manager = PatrolManager(
            node=self,
            navigation_executor=self._navigation_executor,
            persistence_dir=patrol_dir
        )
        
        self._waypoint_recorder = WaypointRecorder(persistence_dir=waypoints_dir)
        
        # 加载持久化数据
        self._task_manager.load_active_tasks()
        self._patrol_manager.load_all_routes()
        
        # ========== 工具模块 ==========
        # 路点工具（独立于任务系统的工具功能）
        self._waypoint_tools = WaypointTools(
            self,
            pose_topic='/rtabmap/localization_pose',
            backup_pose_topic='/wheel/odom',
            persistence_dir=waypoints_dir
        )
        
        # ========== 任务处理器 ==========
        # 创建统一的任务处理器（继承 TaskExecutionHandler 基类）
        self._navigation_handler = NavigationHandler(
            self, self._task_manager, self._navigation_executor
        )
        self._patrol_handler = PatrolHandler(
            self, self._task_manager, self._navigation_executor
        )
        self._exploration_handler = ExplorationHandler(
            self, self._task_manager, self._navigation_executor
        )
        
        self.get_logger().info('All task handlers initialized')
        
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
        self._clear_tasks_srv = self.create_service(
            ClearTasks, '/mission/clear_tasks',
            self._handle_clear_tasks, callback_group=self._callback_group
        )
        
        # 任务执行服务（委托给 TaskExecutionHandler）
        # 探索和巡航服务（暂时保留旧接口，Phase 3 重构）
        self._start_exploration_srv = self.create_service(
            StartExploration, '/mission/start_exploration',
            self._handle_start_exploration,
            callback_group=self._callback_group
        )
        self._start_patrol_srv = self.create_service(
            StartPatrol, '/mission/start_patrol',
            self._handle_start_patrol,
            callback_group=self._callback_group
        )
        
        # 路点服务（委托给 WaypointTools）
        self._waypoint_control_srv = self.create_service(
            WaypointControl, '/mission/waypoint_control',
            self._waypoint_tools.handle_waypoint_control,
            callback_group=self._callback_group
        )
        self._record_waypoints_srv = self.create_service(
            RecordWaypoints, '/mission/record_waypoints',
            self._waypoint_tools.handle_record_waypoints,
            callback_group=self._callback_group
        )
        
        # 导航服务（内置处理）
        self._navigate_to_pose_srv = self.create_service(
            NavigateToPose, '/mission/navigate_to_pose',
            self._handle_navigate_to_pose,
            callback_group=self._callback_group
        )
        self._emergency_stop_srv = self.create_service(
            EmergencyStop, '/mission/emergency_stop',
            self._handle_emergency_stop,
            callback_group=self._callback_group
        )
        
        self.get_logger().info('All service servers created')
    
    # ========== 任务管理服务处理（核心调度逻辑） ==========
    
    def _handle_create_task(self, request, response):
        """处理创建任务请求"""
        try:
            # 将参数键值对数组转换为字典
            parameters = {}
            if hasattr(request, 'parameters_keys') and hasattr(request, 'parameters_values'):
                for key, value in zip(request.parameters_keys, request.parameters_values):
                    parameters[key] = value
            
            task = self._task_manager.create_task(
                task_id=request.task_id,
                task_type=TaskType(request.task_type),
                priority=request.priority,
                parameters=parameters
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
                response.task_type = task.task_type.name  # 添加任务类型
                response.state = task.state.name
                response.progress = task.progress
                # 根据任务状态返回不同的消息
                if task.state == TaskState.FAILED:
                    response.message = f"Task failed: {task.error_message or 'Unknown error'}"
                elif task.state == TaskState.COMPLETED:
                    response.message = "Task completed successfully"
                elif task.state == TaskState.RUNNING:
                    response.message = "Task is running"
                elif task.state == TaskState.WAITING_EXECUTION:
                    response.message = "Task is waiting for NavigationExecutor"
                elif task.state == TaskState.PENDING:
                    response.message = "Task is pending"
                elif task.state == TaskState.PAUSED:
                    response.message = "Task is paused"
                elif task.state == TaskState.CANCELED:
                    response.message = "Task was canceled"
                else:
                    response.message = f"Task state: {task.state.name}"
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
            # 使用正确的字段名 'filter' 而不是 'filter_state'
            if request.filter and request.filter.lower() not in ['all', '']:
                filter_state = TaskState[request.filter.upper()]
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
    
    def _handle_clear_tasks(self, request, response):
        """处理清除任务请求"""
        try:
            cleared_count = 0
            cleared_ids = []
            
            # 如果指定了任务ID，按ID清除
            if request.task_ids:
                cleared_count, cleared_ids = self._task_manager.clear_tasks_by_ids(
                    request.task_ids,
                    request.clear_history
                )
                
            # 如果指定了状态，按状态清除
            elif request.states:
                # 检查是否是清除所有任务
                if len(request.states) == 1 and request.states[0].lower() == 'all':
                    cleared_count, cleared_ids = self._task_manager.clear_all_tasks(
                        request.clear_history
                    )
                else:
                    # 按指定状态清除
                    states_to_clear = []
                    for state_str in request.states:
                        try:
                            states_to_clear.append(TaskState[state_str.upper()])
                        except KeyError:
                            response.success = False
                            response.message = f"Invalid state: {state_str}"
                            response.cleared_count = 0
                            response.cleared_task_ids = []
                            return response
                    
                    cleared_count, cleared_ids = self._task_manager.clear_tasks_by_states(
                        states_to_clear,
                        request.clear_history
                    )
            else:
                # 没有指定任何条件
                response.success = False
                response.message = "Must specify either task_ids or states"
                response.cleared_count = 0
                response.cleared_task_ids = []
                return response
            
            response.success = True
            response.cleared_count = cleared_count
            response.cleared_task_ids = cleared_ids
            response.message = f"Successfully cleared {cleared_count} task(s)"
            
            self.get_logger().info(
                f"Cleared {cleared_count} task(s): {', '.join(cleared_ids) if cleared_ids else 'none'}"
            )
            
        except Exception as e:
            response.success = False
            response.message = f"Error clearing tasks: {str(e)}"
            response.cleared_count = 0
            response.cleared_task_ids = []
            self.get_logger().error(f"Error clearing tasks: {str(e)}")
        
        return response
    
    # ========== 导航服务处理 ==========
    
    def _handle_navigate_to_pose(self, request, response):
        """
        处理导航到位姿请求
        
        服务: /mission/navigate_to_pose
        创建点对点导航任务并启动
        """
        try:
            # 创建导航任务
            task_id = f"nav_{datetime.now().strftime('%Y%m%d_%H%M%S_%f')[:21]}"
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
            import traceback
            response.success = False
            error_str = str(e) if str(e) else f"{type(e).__name__}"
            response.message = f"Failed to start navigation: {error_str}"
            response.task_id = ""
            self.get_logger().error(f"Error starting navigation: {error_str}")
            self.get_logger().error(f"Traceback:\n{traceback.format_exc()}")
        
        return response
    
    def _handle_emergency_stop(self, request, response):
        """
        处理紧急停止请求
        
        服务: /mission/emergency_stop
        """
        try:
            # 立即取消当前导航（如果有）
            nav_state = self._navigation_executor.get_state()
            if nav_state == NavigationState.EXECUTING or nav_state == NavigationState.CANCELING:
                self._navigation_executor.cancel_navigation()
                self.get_logger().warn("Emergency stop: cancelled active navigation")
            
            # 取消所有正在运行的任务
            active_tasks = self._task_manager.get_tasks_by_state(TaskState.RUNNING)
            cancelled_count = 0
            
            for task in active_tasks:
                try:
                    self._task_manager.cancel_task(task.task_id)
                    cancelled_count += 1
                except Exception as e:
                    self.get_logger().error(f"Failed to cancel task {task.task_id}: {str(e)}")
            
            # 如果请求清空任务队列
            if request.clear_tasks:
                self._task_manager.clear_all_tasks()
            
            response.success = True
            response.message = f"Emergency stop executed. Cancelled {cancelled_count} tasks"
            
            self.get_logger().warn(f"Emergency stop: cancelled {cancelled_count} tasks")
            
        except Exception as e:
            response.success = False
            response.message = f"Emergency stop error: {str(e)}"
            self.get_logger().error(f"Error in emergency stop: {str(e)}")
        
        return response
    
    def _handle_start_exploration(self, request, response):
        """处理开始探索请求"""
        try:
            # 创建探索任务
            task_id = f"exploration_{datetime.now().strftime('%Y%m%d_%H%M%S_%f')[:28]}"
            parameters = {
                'map_name': request.map_name,
                'save_map': request.save_map,
                'max_duration': request.max_duration if hasattr(request, 'max_duration') else 0,
                'coverage_threshold': request.coverage_threshold if hasattr(request, 'coverage_threshold') else 0.9
            }
            
            task = self._task_manager.create_task(
                task_id=task_id,
                task_type=TaskType.FRONTIER_EXPLORATION,
                priority=6,
                parameters=parameters
            )
            
            # 立即启动
            self._task_manager.start_task(task_id)
            
            response.success = True
            response.message = "Exploration started"
            response.task_id = task_id
            
            self.get_logger().info(f"Started exploration task: {task_id}")
            
        except Exception as e:
            import traceback
            response.success = False
            response.message = f"Failed to start exploration: {str(e)}"
            response.task_id = ""
            self.get_logger().error(f"Error starting exploration: {str(e)}")
            self.get_logger().error(f"Traceback:\n{traceback.format_exc()}")
        
        return response
    
    def _handle_start_patrol(self, request, response):
        """处理开始巡航请求（暂时保留，Phase 3 重构）"""
        # TODO: Phase 3 - 使用 PatrolHandler
        response.success = False
        response.message = "Patrol not yet implemented in new architecture"
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
            # 如果有当前任务，标记为失败
            current_task = self._task_manager.get_current_task()
            if current_task:
                self._task_manager.fail_task(current_task.task_id, f"Update callback error: {str(e)}")
                self.get_logger().error(f"Marked task {current_task.task_id} as failed due to error")
    
    def _execute_pending_tasks(self):
        """执行待执行的任务（使用新的 Handler 架构）"""
        # 获取RUNNING、WAITING_EXECUTION、PAUSED和CANCELED状态的任务
        # WAITING_EXECUTION: 等待获取 NavigationExecutor
        # RUNNING: 正常执行
        # PAUSED: 需要让handler有机会响应暂停状态（取消Nav2导航）
        # CANCELED: 需要让handler有机会清理资源
        waiting_tasks = self._task_manager.get_tasks_by_state(TaskState.WAITING_EXECUTION)
        running_tasks = self._task_manager.get_tasks_by_state(TaskState.RUNNING)
        paused_tasks = self._task_manager.get_tasks_by_state(TaskState.PAUSED)
        canceled_tasks = self._task_manager.get_tasks_by_state(TaskState.CANCELED)
        
        # 合并所有需要处理的任务，优先执行 RUNNING，然后 WAITING_EXECUTION
        tasks_to_process = waiting_tasks + running_tasks + paused_tasks + canceled_tasks
        
        for task in tasks_to_process:
            try:
                # 任务类型路由 - 使用新的 Handler 架构
                handler = self._get_handler_for_task(task)
                if handler:
                    handler.execute(task)
                else:
                    # 未知任务类型
                    error_msg = f"No handler for task type: {task.task_type}"
                    self.get_logger().error(error_msg)
                    self._task_manager.fail_task(task.task_id, error_msg)
                    
            except Exception as e:
                # 捕获任务执行中的异常并标记任务失败
                error_msg = f"Task execution error: {str(e)}"
                self.get_logger().error(f"Task {task.task_id} failed: {error_msg}")
                import traceback
                self.get_logger().error(f"Traceback:\n{traceback.format_exc()}")
                self._task_manager.fail_task(task.task_id, error_msg)
    
    def _get_handler_for_task(self, task):
        """根据任务类型获取对应的 Handler"""
        if task.task_type == TaskType.POINT_TO_POINT:
            return self._navigation_handler
        elif task.task_type == TaskType.PATH_PATROL:
            return self._patrol_handler
        elif task.task_type == TaskType.FRONTIER_EXPLORATION:
            return self._exploration_handler
        else:
            return None
    
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
        # 保存活动任务
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
