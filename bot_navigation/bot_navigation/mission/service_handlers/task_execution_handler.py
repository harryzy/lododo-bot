#!/usr/bin/env python3
"""
TaskExecutionHandler - 任务执行处理器基类

功能 / Features:
  - 定义统一的任务执行接口
  - 管理 NavigationExecutor 资源的获取和释放
  - 提供任务执行、暂停、恢复、取消的抽象方法

Author: LeKiwi Bot Development Team
Date: 2025-12-30
"""

from abc import ABC, abstractmethod
from typing import Optional
from rclpy.node import Node
from ..task_manager import Task, TaskManager, TaskState
from ..navigation_executor import NavigationExecutor, NavigationState


class TaskExecutionHandler(ABC):
    """
    任务执行处理器基类
    
    所有具体的任务处理器（ExplorationHandler、NavigationHandler、PatrolHandler）
    都应该继承这个基类，并实现抽象方法。
    
    职责:
    1. 管理 NavigationExecutor 的获取和释放（资源互斥）
    2. 定义统一的任务执行接口
    3. 提供任务生命周期管理的钩子方法
    """
    
    def __init__(self, 
                 node: Node, 
                 task_manager: TaskManager, 
                 navigation_executor: NavigationExecutor):
        """
        初始化任务执行处理器
        
        Args:
            node: ROS2 节点实例
            task_manager: 任务管理器
            navigation_executor: 导航执行器（共享资源）
        """
        self._node = node
        self._task_manager = task_manager
        self._nav_executor = navigation_executor
        self._current_task_id: Optional[str] = None  # 当前持有执行器的任务ID
    
    @abstractmethod
    def execute(self, task: Task) -> None:
        """
        执行任务（每个周期调用）
        
        此方法会被 MissionPlanner 的定时器反复调用。
        具体实现需要处理:
        1. WAITING_EXECUTION 状态 - 尝试获取 NavigationExecutor
        2. RUNNING 状态 - 执行任务逻辑
        3. PAUSED 状态 - 暂停处理
        4. CANCELED 状态 - 取消处理
        
        Args:
            task: 要执行的任务
        """
        pass
    
    @abstractmethod
    def pause(self, task: Task) -> bool:
        """
        暂停任务
        
        Args:
            task: 要暂停的任务
            
        Returns:
            bool: 暂停成功返回 True
        """
        pass
    
    @abstractmethod
    def resume(self, task: Task) -> bool:
        """
        恢复任务
        
        Args:
            task: 要恢复的任务
            
        Returns:
            bool: 恢复成功返回 True
        """
        pass
    
    @abstractmethod
    def cancel(self, task: Task) -> bool:
        """
        取消任务
        
        Args:
            task: 要取消的任务
            
        Returns:
            bool: 取消成功返回 True
        """
        pass
    
    def acquire_executor(self, task_id: str) -> bool:
        """
        尝试获取 NavigationExecutor
        
        此方法实现了资源互斥逻辑：
        - 只有一个任务可以持有 NavigationExecutor
        - 必须在 NavigationExecutor 为 IDLE 状态时才能获取
        
        Args:
            task_id: 请求执行器的任务ID
            
        Returns:
            bool: 获取成功返回 True
        """
        # 如果当前没有任务持有执行器
        if self._current_task_id is None:
            nav_state = self._nav_executor.get_state()
            
            # 检查执行器是否空闲
            if nav_state == NavigationState.IDLE:
                self._current_task_id = task_id
                self._node.get_logger().info(
                    f"[TaskExecutionHandler] Task {task_id} acquired NavigationExecutor"
                )
                return True
            else:
                self._node.get_logger().debug(
                    f"[TaskExecutionHandler] NavigationExecutor busy (state: {nav_state.name}), "
                    f"task {task_id} waiting..."
                )
        else:
            # 已经有其他任务持有执行器
            self._node.get_logger().debug(
                f"[TaskExecutionHandler] NavigationExecutor held by task {self._current_task_id}, "
                f"task {task_id} waiting..."
            )
        
        return False
    
    def release_executor(self, task_id: Optional[str] = None):
        """
        释放 NavigationExecutor
        
        Args:
            task_id: 释放执行器的任务ID（可选，用于验证）
        """
        if self._current_task_id is not None:
            # 如果指定了 task_id，验证是否匹配
            if task_id is not None and task_id != self._current_task_id:
                self._node.get_logger().warn(
                    f"[TaskExecutionHandler] Task {task_id} trying to release executor, "
                    f"but it's held by {self._current_task_id}"
                )
                return
            
            self._node.get_logger().info(
                f"[TaskExecutionHandler] Task {self._current_task_id} released NavigationExecutor"
            )
            self._current_task_id = None
    
    def is_executor_owner(self, task_id: str) -> bool:
        """
        检查指定任务是否持有执行器
        
        Args:
            task_id: 任务ID
            
        Returns:
            bool: 如果该任务持有执行器返回 True
        """
        return self._current_task_id == task_id
    
    def get_current_task_id(self) -> Optional[str]:
        """
        获取当前持有执行器的任务ID
        
        Returns:
            Optional[str]: 任务ID，如果没有任务持有则返回 None
        """
        return self._current_task_id
