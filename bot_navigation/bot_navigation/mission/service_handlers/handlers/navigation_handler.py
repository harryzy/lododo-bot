#!/usr/bin/env python3
"""
NavigationHandler - 点对点导航任务处理器

功能 / Features:
  - 处理 POINT_TO_POINT 类型的导航任务
  - 管理导航目标的发送和状态监控
  - 实现暂停、恢复、取消功能

Author: LeKiwi Bot Development Team
Date: 2025-12-30
"""

import math
from geometry_msgs.msg import PoseStamped
from ..task_execution_handler import TaskExecutionHandler
from ...task_manager import Task, TaskState
from ...navigation_executor import NavigationState


class NavigationHandler(TaskExecutionHandler):
    """
    点对点导航任务处理器
    
    处理导航到指定坐标点的任务，集成原 navigation_service_handler 的逻辑
    """
    
    def execute(self, task: Task) -> None:
        """
        执行导航任务
        
        状态流转：
        WAITING_EXECUTION → (acquire executor) → RUNNING → (navigate) → COMPLETED/FAILED
        """
        self._node.get_logger().debug(
            f"[NavigationHandler.execute] task_id={task.task_id}, "
            f"state={task.state.name}, is_owner={self.is_executor_owner(task.task_id)}"
        )
        
        # 1. WAITING_EXECUTION 状态 - 尝试获取 NavigationExecutor
        if task.state == TaskState.WAITING_EXECUTION:
            nav_state = self._nav_executor.get_state()
            current_owner = self._current_task_id
            is_owner = self.is_executor_owner(task.task_id)
            
            # 诊断：详细状态
            self._node.get_logger().info(
                f"[DIAG] Task {task.task_id} trying to acquire executor - "
                f"nav_state={nav_state.name}, current_owner={current_owner}, is_owner={is_owner}"
            )
            
            # 检查执行器是否可用（IDLE 或 FAILED 都可以）
            # FAILED 状态可能是上次任务取消后 Nav2 的异步回调导致
            if nav_state in [NavigationState.IDLE, NavigationState.FAILED] and not self.is_executor_owner(task.task_id):
                # 如果是 FAILED 状态，先重置为 IDLE
                if nav_state == NavigationState.FAILED:
                    self._nav_executor.reset_state()
                    self._node.get_logger().info(
                        f"[NavigationHandler] Reset executor from FAILED to IDLE before task {task.task_id}"
                    )
                
                if self.acquire_executor(task.task_id):
                    # 获取成功，转为 RUNNING
                    self._task_manager.update_task_state(task.task_id, TaskState.RUNNING)
                    self._node.get_logger().info(
                        f"[NavigationHandler] Task {task.task_id} acquired executor, "
                        f"state: WAITING_EXECUTION -> RUNNING"
                    )
                    # 继续执行下面的逻辑
                else:
                    # 获取失败，继续等待
                    return
            else:
                # 执行器忙碌，继续等待
                self._node.get_logger().debug(
                    f"[NavigationHandler] Task {task.task_id} waiting for executor "
                    f"(nav_state={nav_state.name})"
                )
                return
        
        # 2. PAUSED 状态 - 暂停处理
        if task.state == TaskState.PAUSED:
            if self.is_executor_owner(task.task_id):
                nav_state = self._nav_executor.get_state()
                if nav_state not in [NavigationState.IDLE, NavigationState.CANCELED]:
                    # 取消当前导航（只在首次取消时成功）
                    cancel_success = self._nav_executor.cancel_navigation()
                    if cancel_success:
                        self._node.get_logger().info(
                            f"[NavigationHandler] Navigation paused for task {task.task_id}"
                        )
                    else:
                        self._node.get_logger().debug(
                            f"[NavigationHandler] Task {task.task_id} already paused (no active goal)"
                        )
            return
        
        # 3. CANCELED 状态 - 取消处理
        if task.state == TaskState.CANCELED:
            if self.is_executor_owner(task.task_id):
                nav_state = self._nav_executor.get_state()
                if nav_state not in [NavigationState.IDLE, NavigationState.CANCELED]:
                    # 取消当前导航
                    self._nav_executor.cancel_navigation()
                    self._node.get_logger().info(
                        f"[NavigationHandler] Navigation canceled for task {task.task_id}"
                    )
                # 释放执行器
                self.release_executor(task.task_id)
            
            # 清理完毕，从活动任务中删除
            self._task_manager.remove_task(task.task_id)
            return
        
        # 4. RUNNING 状态 - 执行导航
        if task.state == TaskState.RUNNING:
            self._execute_navigation(task)
    
    def _execute_navigation(self, task: Task):
        """执行导航逻辑"""
        nav_state = self._nav_executor.get_state()
        
        # 检查是否已经在导航
        if self.is_executor_owner(task.task_id):
            # 检查导航状态
            if nav_state == NavigationState.SUCCESS:
                # 导航成功
                self._task_manager.complete_task(task.task_id)
                self.release_executor(task.task_id)
                self._node.get_logger().info(
                    f"[NavigationHandler] Task {task.task_id} completed successfully"
                )
                # 清理完毕，删除任务
                self._task_manager.remove_task(task.task_id)
                return
            
            elif nav_state == NavigationState.FAILED:
                # 导航失败 - 但可能是从PAUSED恢复后的旧状态
                # 检查任务是否是刚从PAUSED恢复（task之前是PAUSED）
                # 这种情况下应该重置状态并重新发送导航
                self._node.get_logger().info(
                    f"[NavigationHandler] Detected FAILED state for task {task.task_id}, will reset and resend goal"
                )
                # 重置执行器状态
                self._nav_executor.reset_state()
                new_state = self._nav_executor.get_state()
                self._node.get_logger().info(
                    f"[NavigationHandler] Executor state after reset: {new_state.name}"
                )
                # 继续到发送导航目标
                # 不return，让代码继续执行到_send_navigation_goal
            
            elif nav_state in [NavigationState.IDLE, NavigationState.CANCELED]:
                # 导航已经结束（可能是恢复操作）
                self._node.get_logger().info(
                    f"[NavigationHandler] Detected resume condition: nav_state={nav_state.value}, task_id={task.task_id}"
                )
                # 重置状态并重新发送目标
                if nav_state != NavigationState.IDLE:
                    self._node.get_logger().info(
                        f"[NavigationHandler] Resetting executor state from {nav_state.value} to IDLE"
                    )
                    self._nav_executor.reset_state()
                    # 确认状态已重置
                    new_state = self._nav_executor.get_state()
                    self._node.get_logger().info(
                        f"[NavigationHandler] Executor state after reset: {new_state.name}"
                    )
                self._node.get_logger().info(
                    f"[NavigationHandler] Will resend navigation goal..."
                )
                # 继续发送导航目标
            elif nav_state == NavigationState.EXECUTING:
                # 导航正在进行中
                self._node.get_logger().debug(
                    f"[NavigationHandler] Navigation in progress for task {task.task_id}"
                )
                return
            else:
                # 其他状态
                return
        
        # 发送导航目标
        self._send_navigation_goal(task)
    
    def _send_navigation_goal(self, task: Task):
        """发送导航目标"""
        params = task.parameters
        
        # 记录当前executor状态
        current_nav_state = self._nav_executor.get_state()
        self._node.get_logger().info(
            f"[NavigationHandler._send_navigation_goal] task_id={task.task_id}, "
            f"executor_state={current_nav_state.name}, is_owner={self.is_executor_owner(task.task_id)}"
        )
        
        try:
            # 构造目标位姿
            goal_pose = PoseStamped()
            goal_pose.header.frame_id = params.get('frame_id', 'map')
            goal_pose.header.stamp = self._node.get_clock().now().to_msg()
            goal_pose.pose.position.x = float(params.get('x', 0.0))
            goal_pose.pose.position.y = float(params.get('y', 0.0))
            goal_pose.pose.position.z = 0.0
            
            # 设置方向（yaw转四元数）
            yaw = float(params.get('yaw', 0.0))
            goal_pose.pose.orientation.z = math.sin(yaw / 2.0)
            goal_pose.pose.orientation.w = math.cos(yaw / 2.0)
            
            self._node.get_logger().info(
                f"[NavigationHandler] Sending navigation goal for task {task.task_id}: "
                f"target=({goal_pose.pose.position.x:.2f}, {goal_pose.pose.position.y:.2f}, yaw={yaw:.2f})"
            )
            
            # 使用 NavigationExecutor 执行导航
            success = self._nav_executor.navigate_to_pose(goal_pose)
            
            if not success:
                # 导航启动失败
                error_msg = self._nav_executor.get_last_error() or "Failed to start navigation"
                self._task_manager.fail_task(task.task_id, error_msg)
                self.release_executor(task.task_id)
                self._node.get_logger().error(
                    f"[NavigationHandler] Failed to start navigation for task {task.task_id}: {error_msg}"
                )
                # 清理完毕，删除任务
                self._task_manager.remove_task(task.task_id)
        
        except Exception as e:
            self._node.get_logger().error(
                f"[NavigationHandler] Navigation task failed: {str(e)}"
            )
            self._task_manager.fail_task(task.task_id, str(e))
            self.release_executor(task.task_id)
            # 清理完毕，删除任务
            self._task_manager.remove_task(task.task_id)
    
    def pause(self, task: Task) -> bool:
        """暂停导航任务"""
        if self.is_executor_owner(task.task_id):
            nav_state = self._nav_executor.get_state()
            if nav_state == NavigationState.EXECUTING:
                self._nav_executor.cancel_navigation()
                self._node.get_logger().info(
                    f"[NavigationHandler] Paused task {task.task_id}"
                )
                return True
        return False
    
    def resume(self, task: Task) -> bool:
        """
        恢复导航任务
        
        注意：恢复操作只是将任务状态改回 RUNNING，
        实际的导航重启会在 execute() 方法中处理
        """
        if self.is_executor_owner(task.task_id):
            self._node.get_logger().info(
                f"[NavigationHandler] Resuming task {task.task_id}"
            )
            # 导航重启会在 execute() 中处理
            return True
        return False
    
    def cancel(self, task: Task) -> bool:
        """取消导航任务"""
        if self.is_executor_owner(task.task_id):
            nav_state = self._nav_executor.get_state()
            if nav_state not in [NavigationState.IDLE, NavigationState.CANCELED]:
                self._nav_executor.cancel_navigation()
            
            self.release_executor(task.task_id)
            self._node.get_logger().info(
                f"[NavigationHandler] Canceled task {task.task_id}"
            )
            return True
        return False
