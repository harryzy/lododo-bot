#!/usr/bin/env python3
"""
task_execution_handler.py - 任务执行处理器

处理探索、巡航等任务的执行逻辑
"""

from rclpy.node import Node
from bot_navigation_msgs.srv import StartExploration, StartPatrol
from datetime import datetime

from ..task_manager import TaskManager, TaskType
from ...patrol.patrol_manager import PatrolManager


class TaskExecutionHandler:
    """任务执行处理器"""
    
    def __init__(self, node: Node, task_manager: TaskManager, patrol_manager: PatrolManager):
        """
        初始化任务执行处理器
        
        Args:
            node: ROS2 节点实例
            task_manager: 任务管理器
            patrol_manager: 巡航管理器
        """
        self.node = node
        self._task_manager = task_manager
        self._patrol_manager = patrol_manager
        self._active_nodes = {}  # 存储活动的子节点进程
    
    def handle_start_exploration(self, request, response):
        """
        处理开始探索请求
        
        服务: /mission/start_exploration
        """
        try:
            # 创建探索任务（使用微秒级精度避免ID冲突）
            task_id = f"exploration_{datetime.now().strftime('%Y%m%d_%H%M%S_%f')[:28]}"  # 截断到毫秒
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
            
            self.node.get_logger().info(f"Started exploration task: {task_id}")
            
        except Exception as e:
            import traceback
            response.success = False
            error_str = str(e) if str(e) else f"{type(e).__name__}"
            response.message = f"Failed to start exploration: {error_str}"
            response.task_id = ""
            self.node.get_logger().error(f"Error starting exploration: {error_str}")
            self.node.get_logger().error(f"Exception type: {type(e).__name__}")
            self.node.get_logger().error(f"Traceback:\n{traceback.format_exc()}")
        
        return response
    
    def handle_start_patrol(self, request, response):
        """
        处理开始巡航请求
        
        服务: /mission/start_patrol
        """
        try:
            # 创建巡航任务（使用微秒级精度避免ID冲突）
            task_id = f"patrol_{datetime.now().strftime('%Y%m%d_%H%M%S_%f')[:24]}"  # 截断到毫秒
            parameters = {
                'waypoint_file': request.waypoint_file,
                'patrol_mode': request.patrol_mode,
                'speed_factor': request.speed_factor
            }
            
            task = self._task_manager.create_task(
                task_id=task_id,
                task_type=TaskType.PATH_PATROL,
                priority=6,
                parameters=parameters
            )
            
            # 立即启动
            self._task_manager.start_task(task_id)
            
            response.success = True
            response.message = "Patrol started"
            response.task_id = task_id
            
            self.node.get_logger().info(f"Started patrol task: {task_id}")
            
        except Exception as e:
            import traceback
            response.success = False
            error_str = str(e) if str(e) else f"{type(e).__name__}"
            response.message = f"Failed to start patrol: {error_str}"
            response.task_id = ""
            self.node.get_logger().error(f"Error starting patrol: {error_str}")
            self.node.get_logger().error(f"Exception type: {type(e).__name__}")
            self.node.get_logger().error(f"Traceback:\n{traceback.format_exc()}")
        
        return response
    
    def execute_exploration_task(self, task):
        """
        执行探索任务
        
        由 MissionPlanner 的任务调度循环调用
        """
        from ..task_manager import TaskState
        
        params = task.parameters
        
        # 检查任务状态 - 如果任务被暂停，停止探索
        if task.state == TaskState.PAUSED:
            if task.task_id in self._active_nodes.get('exploration', []):
                self.node.get_logger().info(f"Exploration paused for task {task.task_id}")
                # TODO: 实际暂停探索节点的执行
                # 目前探索功能未完全实现，此处仅记录日志
            return
        
        # 检查任务状态 - 如果任务被取消，停止并清理探索
        if task.state == TaskState.CANCELED:
            if task.task_id in self._active_nodes.get('exploration', []):
                self._active_nodes['exploration'].remove(task.task_id)
                self.node.get_logger().info(f"Exploration canceled for task {task.task_id}")
                # TODO: 实际停止探索节点
            return
        
        # 检查是否已经在执行
        if task.task_id in self._active_nodes.get('exploration', []):
            # 任务已在执行中，检查是否完成
            # TODO: 检查exploration_mapper的状态
            # 临时：不做任何操作，等待手动完成或超时
            return
        
        # 首次执行：启动探索节点
        if 'exploration' not in self._active_nodes:
            self._active_nodes['exploration'] = []
        self._active_nodes['exploration'].append(task.task_id)
        
        self.node.get_logger().info(
            f"Exploration task started: map_name={params.get('map_name')}, "
            f"max_duration={params.get('max_duration')}"
        )
        
        # TODO: 实际启动 exploration_mapper 节点
        # 可以通过 launch API 或 subprocess
        # 并设置完成回调: self._on_exploration_complete(task.task_id)
        
        # 临时：由于exploration是长时间运行任务，需要外部接口来完成
        # 不自动调用 complete_task，等待用户通过服务完成
    
    def execute_patrol_task(self, task):
        """
        执行巡航任务
        
        由 MissionPlanner 的任务调度循环调用
        """
        from ..task_manager import TaskState
        from ...patrol.patrol_manager import PatrolState
        
        params = task.parameters
        
        # 检查任务状态 - 如果任务被暂停，停止巡航
        if task.state == TaskState.PAUSED:
            if task.task_id in self._active_nodes.get('patrol', []):
                # 暂停巡航
                progress = self._patrol_manager.get_current_progress()
                if progress['patrol_state'] != PatrolState.PAUSED.value:
                    self._patrol_manager.pause_patrol()
                    self.node.get_logger().info(f"Patrol paused for task {task.task_id}")
            return
        
        # 检查任务状态 - 如果任务被取消，停止并清理巡航
        if task.state == TaskState.CANCELED:
            if task.task_id in self._active_nodes.get('patrol', []):
                # 停止巡航
                progress = self._patrol_manager.get_current_progress()
                if progress['patrol_state'] not in (PatrolState.IDLE.value, PatrolState.COMPLETED.value):
                    self._patrol_manager.stop_patrol()
                self._active_nodes['patrol'].remove(task.task_id)
                self.node.get_logger().info(f"Patrol canceled for task {task.task_id}")
            return
        
        # 检查是否已经在执行
        if task.task_id in self._active_nodes.get('patrol', []):
            # 如果任务状态是RUNNING但patrol_manager处于暂停状态，恢复巡航
            progress = self._patrol_manager.get_current_progress()
            
            if task.state == TaskState.RUNNING and progress['patrol_state'] == PatrolState.PAUSED.value:
                self._patrol_manager.resume_patrol()
                self.node.get_logger().info(f"Patrol resumed for task {task.task_id}")
            
            # 执行巡航逻辑（导航控制）
            self._patrol_manager.execute_patrol()
            
            # 检查巡航状态
            progress = self._patrol_manager.get_current_progress()
            patrol_state = progress.get('patrol_state')
            
            if patrol_state == PatrolState.COMPLETED.value:
                # 巡航完成
                self._active_nodes['patrol'].remove(task.task_id)
                self._task_manager.complete_task(task.task_id)
                self.node.get_logger().info(f"Patrol task {task.task_id} completed successfully")
            
            elif patrol_state == PatrolState.FAILED.value:
                # 巡航失败
                self._active_nodes['patrol'].remove(task.task_id)
                self._task_manager.fail_task(task.task_id, "Patrol failed during execution")
                self.node.get_logger().error(f"Patrol task {task.task_id} failed")
            
            return
        
        # 首次执行：启动巡航
        if 'patrol' not in self._active_nodes:
            self._active_nodes['patrol'] = []
        
        # 加载路点文件
        waypoint_file = params.get('waypoint_file')
        patrol_mode = params.get('patrol_mode', 'loop')
        
        self.node.get_logger().info(
            f"Loading patrol route from: {waypoint_file}, mode={patrol_mode}"
        )
        
        # 使用PatrolManager加载waypoint文件（转换格式）
        route_id = self._patrol_manager.load_route_from_waypoints_file(
            waypoint_file,
            route_name=f"patrol_task_{task.task_id}"
        )
        
        if route_id is None:
            # 加载失败
            self._task_manager.fail_task(task.task_id, f"Failed to load waypoints from {waypoint_file}")
            self.node.get_logger().error(f"Failed to load patrol waypoints for task {task.task_id}")
            return
        
        # 启动巡航（使用加载的路线）
        success = self._patrol_manager.start_patrol(route_id)
        
        if not success:
            self._task_manager.fail_task(task.task_id, "Failed to start patrol")
            self.node.get_logger().error(f"Failed to start patrol for task {task.task_id}")
            return
        
        self._active_nodes['patrol'].append(task.task_id)
        
        self.node.get_logger().info(
            f"Patrol task started: route_id={route_id}, waypoint_file={waypoint_file}, "
            f"mode={patrol_mode}"
        )
    
    def stop_exploration(self):
        """停止探索任务"""
        if 'exploration' in self._active_nodes:
            task_id = self._active_nodes.pop('exploration')
            self.node.get_logger().info(f"Stopped exploration task: {task_id}")
            # TODO: 实际停止探索节点
    
    def stop_patrol(self):
        """停止巡航任务"""
        if 'patrol' in self._active_nodes:
            task_id = self._active_nodes.pop('patrol')
            self.node.get_logger().info(f"Stopped patrol task: {task_id}")
            # TODO: 实际停止巡航节点
    
    def stop_all_tasks(self):
        """停止所有任务"""
        self.stop_exploration()
        self.stop_patrol()
    
    def _on_exploration_complete(self, task_id: str, success: bool = True, message: str = ""):
        """
        探索任务完成回调
        
        Args:
            task_id: 任务ID
            success: 是否成功
            message: 结果消息
        """
        if 'exploration' in self._active_nodes and task_id in self._active_nodes['exploration']:
            self._active_nodes['exploration'].remove(task_id)
            
            if success:
                self._task_manager.complete_task(task_id)
                self.node.get_logger().info(f"Exploration task {task_id} completed: {message}")
            else:
                self._task_manager.fail_task(task_id, message)
                self.node.get_logger().error(f"Exploration task {task_id} failed: {message}")
    
    def _on_patrol_complete(self, task_id: str, success: bool = True, message: str = ""):
        """
        巡航任务完成回调
        
        Args:
            task_id: 任务ID
            success: 是否成功
            message: 结果消息
        """
        if 'patrol' in self._active_nodes and task_id in self._active_nodes['patrol']:
            self._active_nodes['patrol'].remove(task_id)
            
            if success:
                self._task_manager.complete_task(task_id)
                self.node.get_logger().info(f"Patrol task {task_id} completed: {message}")
            else:
                self._task_manager.fail_task(task_id, message)
                self.node.get_logger().error(f"Patrol task {task_id} failed: {message}")
