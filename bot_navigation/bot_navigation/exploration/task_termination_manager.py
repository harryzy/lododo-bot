#!/usr/bin/env python3
"""
TaskTerminationManager - 任务终止管理器

功能 / Features:
  - 统一的任务终止流程
  - 确保停止所有运动（旋转+导航）
  - 安全的资源清理
  
Purpose:
  消除 exploration_handler.py 中4处重复的任务终止代码

Author: LeKiwi Bot Development Team
Date: 2026-01-05
"""

from typing import Callable, Optional
from ..mission.task_manager import Task, TaskState
from ..mission.navigation_executor import NavigationState


class TaskTerminationManager:
    """
    任务终止管理器
    
    Manages safe task termination with consistent cleanup procedures:
    1. Stop all motion (rotation + navigation)
    2. Update task state
    3. Release resources
    4. Log termination reason
    """
    
    def __init__(self, node, task_manager, nav_executor, rotation_controller):
        """
        初始化终止管理器
        
        Args:
            node: ROS2 节点
            task_manager: 任务管理器
            nav_executor: 导航执行器
            rotation_controller: 旋转控制器
        """
        self._node = node
        self._task_manager = task_manager
        self._nav_executor = nav_executor
        self._rotation_controller = rotation_controller
        
    def terminate_task(self, 
                      task: Task, 
                      final_state: TaskState,
                      reason: str = "",
                      release_callback: Optional[Callable[[str], None]] = None) -> None:
        """
        安全终止任务
        
        执行完整的终止流程：
        1. 停止所有运动（旋转+导航）
        2. 更新任务状态
        3. 记录日志
        4. 释放资源（通过回调）
        
        Args:
            task: 要终止的任务
            final_state: 最终状态 (COMPLETED 或 FAILED)
            reason: 终止原因（用于日志）
            release_callback: 资源释放回调函数，接收 task_id 参数
        """
        # 1. 停止所有运动
        self._stop_all_motion()
        
        # 2. 更新任务状态
        self._task_manager.update_task_state(task.task_id, final_state)
        
        # 3. 记录日志
        state_emoji = "✅" if final_state == TaskState.COMPLETED else "❌"
        log_msg = f"{state_emoji} [TaskTermination] Task {task.task_id} terminated as {final_state.value}."
        if reason:
            log_msg += f" Reason: {reason}"
        
        if final_state == TaskState.COMPLETED:
            self._node.get_logger().info(log_msg)
        else:
            self._node.get_logger().warn(log_msg)
        
        # 4. 释放资源（通过回调）
        if release_callback:
            try:
                release_callback(task.task_id)
                self._node.get_logger().debug(f"[TaskTermination] Released resources for task {task.task_id}")
            except Exception as e:
                self._node.get_logger().error(f"[TaskTermination] Failed to release resources: {e}")
    
    def _stop_all_motion(self) -> None:
        """
        停止所有运动（旋转 + 导航）
        
        确保机器人完全停止，防止任务终止后继续运动
        """
        motion_stopped = False
        
        # 1. 停止旋转
        if self._rotation_controller.is_rotating:
            self._rotation_controller.stop_rotation()
            self._node.get_logger().debug("[TaskTermination] Stopped rotation")
            motion_stopped = True
        
        # 2. 取消导航
        nav_state = self._nav_executor.get_state()
        if nav_state == NavigationState.EXECUTING:
            self._nav_executor.cancel_navigation()
            self._node.get_logger().debug("[TaskTermination] Canceled navigation")
            motion_stopped = True
        
        if motion_stopped:
            self._node.get_logger().info("[TaskTermination] All motion stopped successfully")
    
    def stop_motion_only(self) -> bool:
        """
        仅停止运动，不终止任务
        
        用于暂停或状态转换时需要停止运动但保持任务活跃的场景
        
        Returns:
            bool: 是否有运动被停止
        """
        motion_stopped = False
        
        if self._rotation_controller.is_rotating:
            self._rotation_controller.stop_rotation()
            motion_stopped = True
        
        if self._nav_executor.get_state() == NavigationState.EXECUTING:
            self._nav_executor.cancel_navigation()
            motion_stopped = True
        
        return motion_stopped
    
    def is_any_motion_active(self) -> bool:
        """
        检查是否有任何运动正在进行
        
        Returns:
            bool: 是否有旋转或导航正在进行
        """
        return (self._rotation_controller.is_rotating or 
                self._nav_executor.get_state() == NavigationState.EXECUTING)
