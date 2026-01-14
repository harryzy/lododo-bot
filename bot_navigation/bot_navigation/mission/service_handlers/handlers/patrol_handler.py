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
        self._node.get_logger().debug(f"[PatrolHandler] execute() called for task {task.task_id}, state={task.state.value}")
        
        # 1. WAITING_EXECUTION 状态 - 尝试获取 NavigationExecutor
        if task.state == TaskState.WAITING_EXECUTION:
            nav_state = self._nav_executor.get_state()
            is_owner = self.is_executor_owner(task.task_id)
            
            self._node.get_logger().debug(
                f"[PatrolHandler] Task {task.task_id} in WAITING_EXECUTION: "
                f"nav_state={nav_state.name}, is_owner={is_owner}"
            )
            
            # 如果执行器处于FAILED状态，先重置它
            if nav_state == NavigationState.FAILED and not is_owner:
                self._node.get_logger().info(
                    f"[PatrolHandler] Resetting NavigationExecutor from FAILED state for task {task.task_id}"
                )
                self._nav_executor.reset_state()
                nav_state = NavigationState.IDLE
            
            # 检查执行器是否空闲
            if nav_state == NavigationState.IDLE and not is_owner:
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
                patrol_state = self._patrol_manager._patrol_state
                if patrol_state != PatrolState.PAUSED:
                    self._patrol_manager.pause_patrol()
                    self._node.get_logger().info(
                        f"[PatrolHandler] Patrol paused for task {task.task_id}"
                    )
            return
        
        # 3. CANCELED 状态 - 取消处理
        if task.state == TaskState.CANCELED:
            if self.is_executor_owner(task.task_id):
                # 停止巡航
                self._patrol_manager.stop_patrol()
                self.release_executor(task.task_id)
                self._node.get_logger().info(
                    f"[PatrolHandler] Patrol canceled for task {task.task_id}"
                )
            # 删除任务
            self._task_manager.remove_task(task.task_id)
            return
        
        # 4. RUNNING 状态 - 执行巡航
        if task.state == TaskState.RUNNING:
            self._execute_patrol(task)
    
    def _execute_patrol(self, task: Task):
        """执行巡航逻辑"""
        # 使用 PatrolManager 执行巡航
        # 委托给原有的 execute_patrol 方法
        # 注意：PatrolManager.execute_patrol 已绋集成了完整的巡航逻辑
        if not hasattr(self, '_last_task_id') or self._last_task_id != task.task_id:
            # 任务切换或首次启动，加载路点并初始化巡航
            if hasattr(self, '_last_task_id'):
                self._patrol_manager.stop_patrol()
            
            # 从任务参数中获取路点文件
            waypoint_file = task.parameters.get('waypoint_file', '')
            patrol_mode = task.parameters.get('patrol_mode', 'loop')
            
            if not waypoint_file:
                error_msg = "No waypoint file specified"
                self._task_manager.fail_task(task.task_id, error_msg, permanent_failure=True)
                self.release_executor(task.task_id)
                self._node.get_logger().error(f"[PatrolHandler] {error_msg}")
                return
            
            # 加载路点文件
            try:
                import os
                
                # 如果传入的是文件名（不包含路径），则从PatrolManager的persistence_dir拼接
                if '/' not in waypoint_file and not waypoint_file.startswith('~'):
                    # 获取PatrolManager的waypoints目录（实际是patrol_routes目录）
                    # 但路点文件应该从独立的waypoints目录加载
                    # 从ament_index获取workspace路径
                    try:
                        from ament_index_python.packages import get_package_share_directory
                        import pathlib
                        pkg_share = get_package_share_directory('bot_navigation')
                        workspace_root = pathlib.Path(pkg_share).parent.parent.parent.parent
                        waypoints_base_dir = workspace_root / 'waypoints'
                    except Exception:
                        # 降级方案：使用环境变量或默认路径
                        waypoints_base_dir = os.path.expanduser('~/lododo_bot/waypoints')
                    
                    # 自动添加.yaml扩展名（如果没有）
                    if not waypoint_file.endswith('.yaml'):
                        waypoint_file = f"{waypoint_file}.yaml"
                    
                    expanded_path = os.path.join(str(waypoints_base_dir), waypoint_file)
                else:
                    # 如果已经是完整路径或相对路径，直接展开
                    expanded_path = os.path.expanduser(waypoint_file)
                
                self._node.get_logger().info(f"[PatrolHandler] Loading waypoints from: {expanded_path}")
                
                # 使用 PatrolManager 的路点加载方法
                route_id = self._patrol_manager.load_route_from_waypoints_file(
                    expanded_path,
                    route_name=f"route_{task.task_id}"
                )
                
                if not route_id:
                    error_msg = f"Failed to load waypoints from {waypoint_file}"
                    self._task_manager.fail_task(task.task_id, error_msg, permanent_failure=True)
                    self.release_executor(task.task_id)
                    self._node.get_logger().error(f"[PatrolHandler] {error_msg}")
                    return
                
                # 获取加载的路线
                route = self._patrol_manager._routes.get(route_id)
                if not route:
                    error_msg = "Route not found after loading"
                    self._task_manager.fail_task(task.task_id, error_msg, permanent_failure=True)
                    self.release_executor(task.task_id)
                    self._node.get_logger().error(f"[PatrolHandler] {error_msg}")
                    return
                
                # 设置巡航模式
                route.loop = (patrol_mode == 'loop')
                
                # 启动巡航
                if not self._patrol_manager.start_patrol(route_id):
                    error_msg = "Failed to start patrol"
                    self._task_manager.fail_task(task.task_id, error_msg, permanent_failure=True)
                    self.release_executor(task.task_id)
                    self._node.get_logger().error(f"[PatrolHandler] {error_msg}")
                    return
                
                self._node.get_logger().info(
                    f"[PatrolHandler] Started patrol with {len(route.waypoints)} waypoints, mode={patrol_mode}"
                )
                
            except Exception as e:
                error_msg = f"Error loading waypoints: {str(e)}"
                self._task_manager.fail_task(task.task_id, error_msg)
                self.release_executor(task.task_id)
                self._node.get_logger().error(f"[PatrolHandler] {error_msg}")
                import traceback
                self._node.get_logger().error(traceback.format_exc())
                return
        
        self._last_task_id = task.task_id
        
        # 执行巡航（使用现有的 PatrolManager 方法）
        patrol_state = self._patrol_manager._patrol_state
        self._node.get_logger().debug(
            f"[DEBUG] PatrolHandler._execute_patrol: patrol_state={patrol_state.value}, task_id={task.task_id}"
        )
        
        # 检查巡航状态
        if patrol_state == PatrolState.COMPLETED:
            # 巡航完成
            self._task_manager.complete_task(task.task_id)
            self.release_executor(task.task_id)
            self._node.get_logger().info(
                f"[PatrolHandler] Task {task.task_id} completed"
            )
            # 不删除任务，complete_task已将其移入历史
            # self._task_manager.remove_task(task.task_id)
            return
        
        elif patrol_state == PatrolState.FAILED:
            # 巡航失败（导航失败或取消）
            error_msg = self._patrol_manager.get_current_progress().get('error_message', 'Patrol failed')
            self._task_manager.fail_task(task.task_id, error_msg, permanent_failure=True)
            self.release_executor(task.task_id)
            self._node.get_logger().warn(
                f"[PatrolHandler] Task {task.task_id} failed: {error_msg}"
            )
            # 不删除任务，fail_task已将其移入历史
            # self._task_manager.remove_task(task.task_id)
            return
        
        # 继续执行巡航 - 调用 PatrolManager 的执行循环
        self._patrol_manager.execute_patrol()
    
    def pause(self, task: Task) -> bool:
        """暂停巡航任务"""
        if self.is_executor_owner(task.task_id):
            self._patrol_manager.pause_patrol()
            self._node.get_logger().info(
                f"[PatrolHandler] Paused task {task.task_id}"
            )
            return True
        return False
    
    def resume(self, task: Task) -> bool:
        """恢复巡航任务"""
        if self.is_executor_owner(task.task_id):
            self._patrol_manager.resume_patrol()
            self._node.get_logger().info(
                f"[PatrolHandler] Resuming task {task.task_id}"
            )
            return True
        return False
    
    def cancel(self, task: Task) -> bool:
        """取消巡航任务"""
        if self.is_executor_owner(task.task_id):
            self._patrol_manager.stop_patrol()
            self.release_executor(task.task_id)
            self._node.get_logger().info(
                f"[PatrolHandler] Canceled task {task.task_id}"
            )
            return True
        return False
