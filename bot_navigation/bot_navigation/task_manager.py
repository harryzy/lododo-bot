#!/usr/bin/env python3
"""
TaskManager - 任务管理器

功能 / Features:
  - 任务状态机管理（8种状态）
  - 任务队列管理
  - 任务持久化接口
  - 任务优先级调度

Author: LeKiwi Bot Development Team
Date: 2025-12-22
"""

from enum import Enum
from typing import Dict, List, Optional, Any
from dataclasses import dataclass, field, asdict
from datetime import datetime
import json
import os


class TaskType(Enum):
    """任务类型枚举"""
    # 探索类
    FRONTIER_EXPLORATION = 'frontier_exploration'
    AREA_EXPLORATION = 'area_exploration'
    SMART_EXPLORATION = 'smart_exploration'
    
    # 巡航类
    PATH_PATROL = 'path_patrol'
    AREA_PATROL = 'area_patrol'
    DYNAMIC_PATROL = 'dynamic_patrol'
    
    # 导航类
    POINT_TO_POINT = 'point_to_point'
    PATH_FOLLOWING = 'path_following'
    DOCK_NAVIGATION = 'dock_navigation'
    
    # 复合类
    SEQUENTIAL_TASKS = 'sequential_tasks'
    PARALLEL_TASKS = 'parallel_tasks'
    CONDITIONAL_TASK = 'conditional_task'
    LOOP_TASK = 'loop_task'


class TaskState(Enum):
    """任务状态枚举"""
    PENDING = 'pending'      # 等待中
    RUNNING = 'running'      # 执行中
    PAUSED = 'paused'        # 已暂停
    COMPLETED = 'completed'  # 已完成
    FAILED = 'failed'        # 失败
    CANCELED = 'canceled'    # 已取消
    TIMEOUT = 'timeout'      # 超时
    BLOCKED = 'blocked'      # 阻塞（等待条件）


@dataclass
class Task:
    """任务数据结构"""
    task_id: str
    task_type: TaskType
    parameters: Dict[str, Any] = field(default_factory=dict)
    priority: int = 5  # 优先级 1-10，数字越小优先级越高
    state: TaskState = TaskState.PENDING
    progress: float = 0.0  # 进度 0.0-1.0
    created_at: str = field(default_factory=lambda: datetime.now().isoformat())
    started_at: Optional[str] = None
    completed_at: Optional[str] = None
    error_message: Optional[str] = None
    retry_count: int = 0
    max_retries: int = 3
    timeout: Optional[float] = None  # 超时时间(秒)
    
    def to_dict(self) -> Dict:
        """转换为字典"""
        data = asdict(self)
        data['task_type'] = self.task_type.value
        data['state'] = self.state.value
        return data
    
    @classmethod
    def from_dict(cls, data: Dict) -> 'Task':
        """从字典创建任务"""
        data['task_type'] = TaskType(data['task_type'])
        data['state'] = TaskState(data['state'])
        return cls(**data)


class TaskManager:
    """
    任务管理器
    
    负责任务的创建、调度、执行、监控和持久化
    """
    
    def __init__(self, persistence_dir: str = None):
        """
        初始化任务管理器
        
        Args:
            persistence_dir: 任务持久化目录
        """
        self._tasks: Dict[str, Task] = {}  # 所有任务
        self._task_queue: List[str] = []   # 任务队列（task_id列表）
        self._current_task_id: Optional[str] = None  # 当前执行的任务
        
        # 持久化目录
        if persistence_dir is None:
            home = os.path.expanduser('~')
            persistence_dir = os.path.join(home, '.ros', 'lekiwi_bot', 'navigation', 'tasks')
        
        self._persistence_dir = persistence_dir
        os.makedirs(self._persistence_dir, exist_ok=True)
        
        self._active_tasks_file = os.path.join(self._persistence_dir, 'active_tasks.json')
        self._history_dir = os.path.join(self._persistence_dir, 'history')
        os.makedirs(self._history_dir, exist_ok=True)
    
    def create_task(self, 
                   task_type: TaskType,
                   parameters: Dict[str, Any],
                   priority: int = 5,
                   task_id: Optional[str] = None) -> Task:
        """
        创建新任务
        
        Args:
            task_type: 任务类型
            parameters: 任务参数
            priority: 优先级
            task_id: 任务ID（可选，默认自动生成）
            
        Returns:
            Task: 创建的任务
        """
        if task_id is None:
            task_id = f"task_{datetime.now().strftime('%Y%m%d_%H%M%S_%f')}"
        
        task = Task(
            task_id=task_id,
            task_type=task_type,
            parameters=parameters,
            priority=priority
        )
        
        self._tasks[task_id] = task
        self._add_to_queue(task_id)
        self._save_active_tasks()
        
        return task
    
    def _add_to_queue(self, task_id: str):
        """将任务添加到队列（按优先级排序）"""
        task = self._tasks[task_id]
        
        # 找到合适的插入位置
        insert_index = len(self._task_queue)
        for i, tid in enumerate(self._task_queue):
            if self._tasks[tid].priority > task.priority:
                insert_index = i
                break
        
        self._task_queue.insert(insert_index, task_id)
    
    def get_task(self, task_id: str) -> Optional[Task]:
        """获取任务"""
        return self._tasks.get(task_id)
    
    def get_all_tasks(self) -> List[Task]:
        """获取所有任务"""
        return list(self._tasks.values())
    
    def get_tasks_by_state(self, state: TaskState) -> List[Task]:
        """获取指定状态的任务"""
        return [task for task in self._tasks.values() if task.state == state]
    
    def get_next_task(self) -> Optional[Task]:
        """获取下一个待执行的任务"""
        for task_id in self._task_queue:
            task = self._tasks[task_id]
            if task.state == TaskState.PENDING:
                return task
        return None
    
    def start_task(self, task_id: str) -> bool:
        """
        开始执行任务
        
        Args:
            task_id: 任务ID
            
        Returns:
            bool: 成功返回True
        """
        task = self._tasks.get(task_id)
        if task is None:
            return False
        
        if task.state != TaskState.PENDING:
            return False
        
        task.state = TaskState.RUNNING
        task.started_at = datetime.now().isoformat()
        self._current_task_id = task_id
        self._save_active_tasks()
        
        return True
    
    def pause_task(self, task_id: str) -> bool:
        """暂停任务"""
        task = self._tasks.get(task_id)
        if task is None or task.state != TaskState.RUNNING:
            return False
        
        task.state = TaskState.PAUSED
        self._save_active_tasks()
        return True
    
    def resume_task(self, task_id: str) -> bool:
        """恢复任务"""
        task = self._tasks.get(task_id)
        if task is None or task.state != TaskState.PAUSED:
            return False
        
        task.state = TaskState.RUNNING
        self._save_active_tasks()
        return True
    
    def complete_task(self, task_id: str) -> bool:
        """完成任务"""
        task = self._tasks.get(task_id)
        if task is None:
            return False
        
        task.state = TaskState.COMPLETED
        task.completed_at = datetime.now().isoformat()
        task.progress = 1.0
        
        if task_id == self._current_task_id:
            self._current_task_id = None
        
        self._remove_from_queue(task_id)
        self._save_to_history(task)
        self._save_active_tasks()
        
        return True
    
    def fail_task(self, task_id: str, error_message: str) -> bool:
        """任务失败"""
        task = self._tasks.get(task_id)
        if task is None:
            return False
        
        task.state = TaskState.FAILED
        task.completed_at = datetime.now().isoformat()
        task.error_message = error_message
        task.retry_count += 1
        
        # 检查是否可以重试
        if task.retry_count < task.max_retries:
            task.state = TaskState.PENDING
            task.error_message = f"Retry {task.retry_count}/{task.max_retries}: {error_message}"
        else:
            if task_id == self._current_task_id:
                self._current_task_id = None
            self._remove_from_queue(task_id)
            self._save_to_history(task)
        
        self._save_active_tasks()
        return True
    
    def cancel_task(self, task_id: str) -> bool:
        """取消任务"""
        task = self._tasks.get(task_id)
        if task is None:
            return False
        
        task.state = TaskState.CANCELED
        task.completed_at = datetime.now().isoformat()
        
        if task_id == self._current_task_id:
            self._current_task_id = None
        
        self._remove_from_queue(task_id)
        self._save_to_history(task)
        self._save_active_tasks()
        
        return True
    
    def update_progress(self, task_id: str, progress: float) -> bool:
        """更新任务进度"""
        task = self._tasks.get(task_id)
        if task is None:
            return False
        
        task.progress = max(0.0, min(1.0, progress))
        self._save_active_tasks()
        return True
    
    def _remove_from_queue(self, task_id: str):
        """从队列中移除任务"""
        if task_id in self._task_queue:
            self._task_queue.remove(task_id)
    
    def get_current_task(self) -> Optional[Task]:
        """获取当前执行的任务"""
        if self._current_task_id is None:
            return None
        return self._tasks.get(self._current_task_id)
    
    def clear_completed_tasks(self):
        """清理已完成的任务"""
        completed_ids = [
            tid for tid, task in self._tasks.items()
            if task.state in [TaskState.COMPLETED, TaskState.FAILED, TaskState.CANCELED]
        ]
        
        for tid in completed_ids:
            del self._tasks[tid]
        
        self._save_active_tasks()
    
    def _save_active_tasks(self):
        """保存活动任务到文件"""
        active_tasks = {
            'tasks': {tid: task.to_dict() for tid, task in self._tasks.items()},
            'queue': self._task_queue,
            'current_task_id': self._current_task_id
        }
        
        with open(self._active_tasks_file, 'w') as f:
            json.dump(active_tasks, f, indent=2)
    
    def load_active_tasks(self):
        """从文件加载活动任务"""
        if not os.path.exists(self._active_tasks_file):
            return
        
        try:
            with open(self._active_tasks_file, 'r') as f:
                data = json.load(f)
            
            self._tasks = {
                tid: Task.from_dict(task_data)
                for tid, task_data in data['tasks'].items()
            }
            self._task_queue = data['queue']
            self._current_task_id = data.get('current_task_id')
            
        except Exception as e:
            print(f"Failed to load active tasks: {e}")
    
    def _save_to_history(self, task: Task):
        """保存任务到历史记录"""
        date_str = datetime.now().strftime('%Y%m%d')
        history_file = os.path.join(self._history_dir, f'tasks_{date_str}.json')
        
        # 读取现有历史
        history = []
        if os.path.exists(history_file):
            try:
                with open(history_file, 'r') as f:
                    history = json.load(f)
            except:
                pass
        
        # 添加新任务
        history.append(task.to_dict())
        
        # 保存
        with open(history_file, 'w') as f:
            json.dump(history, f, indent=2)
