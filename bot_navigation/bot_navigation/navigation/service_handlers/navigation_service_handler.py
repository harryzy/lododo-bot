#!/usr/bin/env python3
"""
navigation_service_handler.py - 导航相关服务处理器

处理点对点导航、紧急停止等服务请求
"""

import math
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from bot_navigation_msgs.srv import NavigateToPose, EmergencyStop
from datetime import datetime

from ..task_manager import TaskManager, TaskType
from ..navigation_executor import NavigationExecutor


class NavigationServiceHandler:
    """导航服务处理器"""
    
    def __init__(self, node: Node, task_manager: TaskManager, navigation_executor: NavigationExecutor):
        """
        初始化导航服务处理器
        
        Args:
            node: ROS2 节点实例
            task_manager: 任务管理器
            navigation_executor: 导航执行器
        """
        self.node = node
        self._task_manager = task_manager
        self._navigation_executor = navigation_executor
    
    def handle_navigate_to_pose(self, request, response):
        """
        处理导航到位姿请求
        
        服务: /mission/navigate_to_pose
        """
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
            
            self.node.get_logger().info(f"Started navigation task: {task_id}")
            
        except Exception as e:
            response.success = False
            response.message = f"Failed to start navigation: {str(e)}"
            response.task_id = ""
            self.node.get_logger().error(f"Error starting navigation: {str(e)}")
        
        return response
    
    def handle_emergency_stop(self, request, response):
        """
        处理紧急停止请求
        
        服务: /mission/emergency_stop
        """
        try:
            # 取消所有正在运行的任务
            active_tasks = self._task_manager.get_active_tasks()
            cancelled_count = 0
            
            for task in active_tasks:
                try:
                    self._task_manager.cancel_task(task.task_id)
                    cancelled_count += 1
                except Exception as e:
                    self.node.get_logger().error(f"Failed to cancel task {task.task_id}: {str(e)}")
            
            # 如果请求清空任务队列
            if request.clear_queue:
                self._task_manager.clear_all_tasks()
            
            response.success = True
            response.message = f"Emergency stop executed. Cancelled {cancelled_count} tasks"
            response.stopped_tasks = [task.task_id for task in active_tasks]
            
            self.node.get_logger().warn(f"Emergency stop: cancelled {cancelled_count} tasks")
            
        except Exception as e:
            response.success = False
            response.message = f"Emergency stop error: {str(e)}"
            response.stopped_tasks = []
            self.node.get_logger().error(f"Error in emergency stop: {str(e)}")
        
        return response
    
    def execute_navigation_task(self, task):
        """
        执行点对点导航任务
        
        由 TaskExecutionHandler 调用
        """
        params = task.parameters
        
        try:
            # 构造目标位姿
            goal_pose = PoseStamped()
            goal_pose.header.frame_id = params.get('frame_id', 'map')
            goal_pose.header.stamp = self.node.get_clock().now().to_msg()
            goal_pose.pose.position.x = float(params.get('x', 0.0))
            goal_pose.pose.position.y = float(params.get('y', 0.0))
            goal_pose.pose.position.z = 0.0
            
            # 设置方向（yaw转四元数）
            yaw = float(params.get('yaw', 0.0))
            goal_pose.pose.orientation.z = math.sin(yaw / 2.0)
            goal_pose.pose.orientation.w = math.cos(yaw / 2.0)
            
            self.node.get_logger().info(
                f"Navigation task started: target=({goal_pose.pose.position.x:.2f}, "
                f"{goal_pose.pose.position.y:.2f}, yaw={yaw:.2f})"
            )
            
            # 使用 NavigationExecutor 执行导航
            self._navigation_executor.navigate_to_pose(goal_pose)
            
            # TODO: 监听导航结果并更新任务状态
            # 现在暂时手动标记为完成（实际应该在回调中处理）
            
        except Exception as e:
            self.node.get_logger().error(f"Navigation task failed: {str(e)}")
            self._task_manager.fail_task(task.task_id, str(e))
