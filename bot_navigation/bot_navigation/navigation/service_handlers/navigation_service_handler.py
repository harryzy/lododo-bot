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
        self._current_nav_task_id = None  # 当前正在执行的导航任务ID
    
    def handle_navigate_to_pose(self, request, response):
        """
        处理导航到位姿请求
        
        服务: /mission/navigate_to_pose
        """
        try:
            # 创建导航任务（使用微秒级精度避免ID冲突）
            task_id = f"nav_{datetime.now().strftime('%Y%m%d_%H%M%S_%f')[:21]}"  # 截断到毫秒
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
            import traceback
            response.success = False
            error_str = str(e) if str(e) else f"{type(e).__name__}"
            response.message = f"Failed to start navigation: {error_str}"
            response.task_id = ""
            self.node.get_logger().error(f"Error starting navigation: {error_str}")
            self.node.get_logger().error(f"Exception type: {type(e).__name__}")
            self.node.get_logger().error(f"Traceback:\n{traceback.format_exc()}")
        
        return response
    
    def handle_emergency_stop(self, request, response):
        """
        处理紧急停止请求
        
        服务: /mission/emergency_stop
        """
        try:
            # 取消所有正在运行的任务
            from ..task_manager import TaskState
            active_tasks = self._task_manager.get_tasks_by_state(TaskState.RUNNING)
            cancelled_count = 0
            
            for task in active_tasks:
                try:
                    self._task_manager.cancel_task(task.task_id)
                    cancelled_count += 1
                except Exception as e:
                    self.node.get_logger().error(f"Failed to cancel task {task.task_id}: {str(e)}")
            
            # 如果请求清空任务队列
            if request.clear_tasks:
                self._task_manager.clear_all_tasks()
            
            response.success = True
            response.message = f"Emergency stop executed. Cancelled {cancelled_count} tasks"
            
            self.node.get_logger().warn(f"Emergency stop: cancelled {cancelled_count} tasks")
            
        except Exception as e:
            response.success = False
            response.message = f"Emergency stop error: {str(e)}"
            self.node.get_logger().error(f"Error in emergency stop: {str(e)}")
        
        return response
    
    def execute_navigation_task(self, task):
        """
        执行点对点导航任务
        
        由 TaskExecutionHandler 调用
        注意：此方法会被定时器反复调用，需要检查是否已经启动过导航
        """
        from ..navigation_executor import NavigationState
        from ..task_manager import TaskState
        
        # 检查任务状态 - 如果任务被暂停，取消导航
        if task.state == TaskState.PAUSED:
            if self._current_nav_task_id == task.task_id:
                nav_state = self._navigation_executor.get_state()
                self.node.get_logger().info(f"Task {task.task_id} is PAUSED, nav_state: {nav_state.name}")
                if nav_state != NavigationState.IDLE:
                    # 取消当前导航
                    self._navigation_executor.cancel_navigation()
                    self.node.get_logger().info(f"Navigation paused for task {task.task_id}")
                # 保留 _current_nav_task_id，以便恢复时可以继续
            return
        
        # 检查任务状态 - 如果任务被取消，取消导航并清理
        if task.state == TaskState.CANCELED:
            if self._current_nav_task_id == task.task_id:
                nav_state = self._navigation_executor.get_state()
                if nav_state != NavigationState.IDLE:
                    # 取消当前导航
                    self._navigation_executor.cancel_navigation()
                    self.node.get_logger().info(f"Navigation canceled for task {task.task_id}")
                # 清除当前任务ID
                self._current_nav_task_id = None
            return
        
        # 检查导航执行器状态，避免重复发送导航目标
        nav_state = self._navigation_executor.get_state()
        
        self.node.get_logger().debug(
            f"execute_navigation_task: task={task.task_id}, task_state={task.state.name}, "
            f"nav_state={nav_state.name}, current_nav_task_id={self._current_nav_task_id}"
        )
        
        # 如果这是当前正在执行的任务，检查其状态
        if self._current_nav_task_id == task.task_id:
            if nav_state == NavigationState.SUCCESS:
                self._task_manager.complete_task(task.task_id)
                self.node.get_logger().info(f"Navigation task {task.task_id} completed successfully")
                self._current_nav_task_id = None  # 清除当前任务ID
                return
            elif (nav_state == NavigationState.IDLE or 
                  nav_state == NavigationState.CANCELED or
                  nav_state == NavigationState.FAILED):
                # 导航已经结束（可能是被暂停后取消）
                # 检查任务状态：如果任务是RUNNING，说明是恢复操作，需要重新发送导航目标
                # 如果任务不是RUNNING（比如被取消/暂停），则不应该继续执行
                if task.state == TaskState.RUNNING:
                    self.node.get_logger().info(
                        f"Resuming navigation for task {task.task_id} "
                        f"(nav_state={nav_state.name}, task_state={task.state.name})"
                    )
                    # 重置导航状态（如果是FAILED/CANCELED，必须重置才能重新发送目标）
                    if nav_state != NavigationState.IDLE:
                        self._navigation_executor.reset_state()
                        self.node.get_logger().info(f"Reset navigation state before resume")
                    # 清除_current_nav_task_id，让后面的代码重新发送
                    self._current_nav_task_id = None
                    # 继续执行下面的代码发送导航目标
                elif nav_state == NavigationState.FAILED:
                    # 导航失败且任务不是RUNNING状态（说明不是恢复），标记任务为失败
                    error_msg = self._navigation_executor.get_last_error() or "Navigation failed"
                    self._task_manager.fail_task(task.task_id, error_msg)
                    self.node.get_logger().warn(f"Navigation task {task.task_id} failed: {error_msg}")
                    self._current_nav_task_id = None
                    return
                else:
                    # IDLE或CANCELED状态，任务已经停止，什么也不做
                    return
            else:
                # 导航正在进行中，不需要重复发送
                self.node.get_logger().debug(f"Navigation in progress for task {task.task_id}, nav_state: {nav_state.name}")
                return
        
        # 如果有其他任务正在执行，等待
        if self._current_nav_task_id is not None and nav_state != NavigationState.IDLE:
            # 有其他导航任务正在执行，当前任务需要等待
            return
        
        # 如果导航器状态不是IDLE，但没有关联的任务（可能是上次任务遗留），重置状态
        if nav_state != NavigationState.IDLE and self._current_nav_task_id is None:
            self.node.get_logger().warn(
                f"Navigation executor in state {nav_state.name} without active task, resetting state"
            )
            if nav_state == NavigationState.EXECUTING or nav_state == NavigationState.CANCELING:
                # 如果有正在执行的导航，先取消
                self._navigation_executor.cancel_navigation()
            else:
                # FAILED/CANCELED等终止状态，直接重置
                self._navigation_executor.reset_state()
            # 不return，继续发送新的导航目标
        
        # 只有在IDLE状态时才发送新的导航目标
        params = task.parameters
        
        self.node.get_logger().info(
            f"Sending navigation goal for task {task.task_id}: "
            f"x={params.get('x')}, y={params.get('y')}, yaw={params.get('yaw')}"
        )
        
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
                f"Starting navigation task {task.task_id}: target=({goal_pose.pose.position.x:.2f}, "
                f"{goal_pose.pose.position.y:.2f}, yaw={yaw:.2f})"
            )
            
            # 使用 NavigationExecutor 执行导航
            success = self._navigation_executor.navigate_to_pose(goal_pose)
            
            if not success:
                # 导航启动失败
                error_msg = self._navigation_executor.get_last_error() or "Failed to start navigation"
                self._task_manager.fail_task(task.task_id, error_msg)
                self.node.get_logger().error(f"Failed to start navigation for task {task.task_id}: {error_msg}")
            else:
                # 导航启动成功，记录当前任务ID
                self._current_nav_task_id = task.task_id
            
        except Exception as e:
            self.node.get_logger().error(f"Navigation task failed: {str(e)}")
            self._task_manager.fail_task(task.task_id, str(e))
