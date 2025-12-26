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
            
            self.node.get_logger().info(f"Started exploration task: {task_id}")
            
        except Exception as e:
            response.success = False
            response.message = f"Failed to start exploration: {str(e)}"
            response.task_id = ""
            self.node.get_logger().error(f"Error starting exploration: {str(e)}")
        
        return response
    
    def handle_start_patrol(self, request, response):
        """
        处理开始巡航请求
        
        服务: /mission/start_patrol
        """
        try:
            # 创建巡航任务
            task_id = f"patrol_{datetime.now().strftime('%Y%m%d_%H%M%S')}"
            parameters = {
                'waypoint_file': request.waypoint_file,
                'patrol_mode': request.patrol_mode,
                'loop_count': request.loop_count
            }
            
            task = self._task_manager.create_task(
                task_id=task_id,
                task_type=TaskType.WAYPOINT_PATROL,
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
            response.success = False
            response.message = f"Failed to start patrol: {str(e)}"
            response.task_id = ""
            self.node.get_logger().error(f"Error starting patrol: {str(e)}")
        
        return response
    
    def execute_exploration_task(self, task):
        """
        执行探索任务
        
        由 MissionPlanner 的任务调度循环调用
        """
        params = task.parameters
        
        # 检查是否已经在探索
        if 'exploration' in self._active_nodes:
            self.node.get_logger().info(f"Exploration task {task.task_id} already running")
            return
        
        # 启动探索节点
        self._active_nodes['exploration'] = task.task_id
        
        self.node.get_logger().info(
            f"Exploration task started: map_name={params.get('map_name')}, "
            f"max_duration={params.get('max_duration')}"
        )
        
        # TODO: 实际启动 exploration_mapper 节点
        # 可以通过 launch API 或 subprocess
        
        # 临时：模拟任务完成
        # self._task_manager.complete_task(task.task_id)
    
    def execute_patrol_task(self, task):
        """
        执行巡航任务
        
        由 MissionPlanner 的任务调度循环调用
        """
        params = task.parameters
        
        # 检查是否已经在巡航
        if 'patrol' in self._active_nodes:
            self.node.get_logger().info(f"Patrol task {task.task_id} already running")
            return
        
        # 启动巡航节点
        self._active_nodes['patrol'] = task.task_id
        
        self.node.get_logger().info(
            f"Patrol task started: waypoint_file={params.get('waypoint_file')}, "
            f"mode={params.get('patrol_mode')}"
        )
        
        # TODO: 实际启动 patrol_node
        # 可以通过 launch API 或 subprocess
        
        # 临时：模拟任务完成
        # self._task_manager.complete_task(task.task_id)
    
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
