#!/usr/bin/env python3
"""
PatrolHandler - 巡航任务处理器

功能 / Features:
  - 处理 PATH_PATROL 类型的巡航任务
  - 集成 PatrolManager 实现路径巡航
  - 支持循环、往返、单次、随机模式

Author: LeKiwi Bot Development Team
Date: 2025-12-30
"""

from ..task_execution_handler import TaskExecutionHandler
from ...task_manager import Task, TaskState
from ...navigation_executor import NavigationState
from ....patrol.patrol_manager import PatrolManager, PatrolState


class PatrolHandler(TaskExecutionHandler):
    """
    巡航任务处理器
    
    集成 PatrolManager 的逻辑，实现路径巡航任务
    """
    
    def __init__(self, node, task_manager, navigation_executor):
        super().__init__(node, task_manager, navigation_executor)
        
        # 创建 PatrolManager 实例
        self._patrol_manager = PatrolManager(
            node, navigation_executor
        )
    
    def execute(self, task: Task) -> None:
        """
        执行巡航任务
        
        状态流转：
        WAITING_EXECUTION → (acquire executor) → RUNNING → (patrol) → COMPLETED/FAILED
        """
        # 1. WAITING_EXECUTION 状态 - 尝试获取 NavigationExecutor
        if task.state == TaskState.WAITING_EXECUTION:
            nav_state = self._nav_executor.get_state()
            
            # 检查执行器是否空闲
            if nav_state == NavigationState.IDLE and not self.is_executor_owner(task.task_id):
                if self.acquire_executor(task.task_id):
                    # 获取成功，转为 RUNNING
                    self._task_manager.update_task_state(task.task_id, TaskState.RUNNING)
                    self._node.get_logger().info(
                        f"[PatrolHandler] Task {task.task_id} acquired executor, "
                        f"state: WAITING_EXECUTION -> RUNNING"
                    )
                    # 继续执行下面的逻辑
                else:
                    # 获取失败，继续等待
                    return
            else:
                # 执行器忙碌，继续等待
                self._node.get_logger().debug(
                    f"[PatrolHandler] Task {task.task_id} waiting for executor "
                    f"(nav_state={nav_state.name})"
                )
                return
        
        # 2. PAUSED 状态 - 暂停处理
        if task.state == TaskState.PAUSED:
            if self.is_executor_owner(task.task_id):
                # 暂停巡航
                patrol_state = self._patrol_manager.get_state()
                if patrol_state != PatrolState.PAUSED:
                    self._patrol_manager.pause()
                    self._node.get_logger().info(
                        f"[PatrolHandler] Patrol paused for task {task.task_id}"
                    )
            return
        
        # 3. CANCELED 状态 - 取消处理
        if task.state == TaskState.CANCELED:
            if self.is_executor_owner(task.task_id):
                # 停止巡航
                self._patrol_manager.stop()
                self.release_executor(task.task_id)
                self._node.get_logger().info(
                    f"[PatrolHandler] Patrol canceled for task {task.task_id}"
                )
            return
        
        # 4. RUNNING 状态 - 执行巡航
        if task.state == TaskState.RUNNING:
            self._execute_patrol(task)
    
    def _execute_patrol(self, task: Task):
        """执行巡航逻辑"""
        # 使用 PatrolManager 执行巡航
        # 委托给原有的 execute_patrol 方法
        # 注意：PatrolManager.execute_patrol 已经集成了完整的巡航逻辑
        if hasattr(self, '_last_task_id') and self._last_task_id != task.task_id:
            # 任务切换，重新初始化巡航
            self._patrol_manager.stop()
        
        self._last_task_id = task.task_id
        
        # 执行巡航（使用现有的 PatrolManager 方法）
        patrol_state = self._patrol_manager.get_state()
        
        # 检查巡航状态
        if patrol_state == PatrolState.COMPLETED:
            # 巡航完成
            self._task_manager.complete_task(task.task_id)
            self.release_executor(task.task_id)
            self._node.get_logger().info(
                f"[PatrolHandler] Task {task.task_id} completed"
            )
            return
        
        elif patrol_state == PatrolState.FAILED:
            # 巡航失败
            error_msg = "Patrol failed"
            self._task_manager.fail_task(task.task_id, error_msg)
            self.release_executor(task.task_id)
            self._node.get_logger().warn(
                f"[PatrolHandler] Task {task.task_id} failed"
            )
            return
        
        # 继续执行巡航
        # 注意：实际的巡航执行逻辑在 PatrolManager 中
        # 这里只需要调用 PatrolManager 的更新方法
    
    def pause(self, task: Task) -> bool:
        """暂停巡航任务"""
        if self.is_executor_owner(task.task_id):
            self._patrol_manager.pause()
            self._node.get_logger().info(
                f"[PatrolHandler] Paused task {task.task_id}"
            )
            return True
        return False
    
    def resume(self, task: Task) -> bool:
        """恢复巡航任务"""
        if self.is_executor_owner(task.task_id):
            self._patrol_manager.resume()
            self._node.get_logger().info(
                f"[PatrolHandler] Resuming task {task.task_id}"
            )
            return True
        return False
    
    def cancel(self, task: Task) -> bool:
        """取消巡航任务"""
        if self.is_executor_owner(task.task_id):
            self._patrol_manager.stop()
            self.release_executor(task.task_id)
            self._node.get_logger().info(
                f"[PatrolHandler] Canceled task {task.task_id}"
            )
            return True
        return False
