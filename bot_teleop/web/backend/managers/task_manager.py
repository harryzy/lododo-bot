#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Task Manager - Core task management component
Responsible for task creation, updating, persistence, and querying

Architecture: Passively receive CMD responses, no active polling
- Receive all /cmd/response messages through WebTerminalNode's _response_callback
- Intelligently match task_id or request_id, update cached tasks
- Ignore unrelated responses (not in cache)
- JSON file persistence for task history
"""

import json
import os
from datetime import datetime
from typing import Dict, List, Optional
from dataclasses import dataclass, asdict
import logging
from pathlib import Path

logger = logging.getLogger(__name__)


@dataclass
class Task:
    """Task data model"""
    task_id: Optional[str] = None  # Task ID returned by MissionPlanner (string like nav_xxx)
    request_id: str = ""  # CMD request ID
    action: str = ""  # Task type (navigate_to_pose, start_exploration, etc.)
    params: Dict = None  # Task parameters
    status: str = "queued"  # queued, executing, paused, completed, failed, cancelled
    progress: float = 0.0  # Progress 0-100
    message: str = ""  # Status message
    created_at: str = ""  # Creation time
    updated_at: str = ""  # Update time
    completed_at: Optional[str] = None  # Completion time

    def __post_init__(self):
        if self.params is None:
            self.params = {}
        if not self.created_at:
            self.created_at = datetime.now().isoformat()
        if not self.updated_at:
            self.updated_at = self.created_at

    def to_dict(self) -> Dict:
        """Convert to dictionary"""
        return asdict(self)

    @classmethod
    def from_dict(cls, data: Dict) -> 'Task':
        """Create task from dictionary"""
        return cls(**data)


class TaskManager:
    """
    Task Manager - Singleton pattern
    
    Core responsibilities:
    1. Create tasks (when user initiates operation)
    2. Update tasks (when receiving CMD responses)
    3. Query tasks (when user requests status)
    4. Persist task history
    """

    _instance = None

    def __new__(cls):
        if cls._instance is None:
            cls._instance = super(TaskManager, cls).__new__(cls)
            cls._instance._initialized = False
        return cls._instance

    def __init__(self):
        if self._initialized:
            return

        self._initialized = True
        self.active_tasks: Dict[str, Task] = {}  # request_id -> Task
        self.task_id_map: Dict[int, str] = {}  # task_id -> request_id
        self.history_file = Path.home() / "lododo_bot" / "data" / "task_history.json"
        self.history_file.parent.mkdir(parents=True, exist_ok=True)
        
        # Load task history (for display only, not part of logic)
        self.task_history: List[Task] = self._load_history()
        
        logger.info(f"✓ TaskManager initialized. History file: {self.history_file}")

    def create_task(self, request_id: str, action: str, params: Dict) -> Task:
        """
        Create new task (called when user initiates operation)
        
        Args:
            request_id: CMD request ID (unique identifier)
            action: Task type
            params: Task parameters
        
        Returns:
            Task: Created task object
        """
        task = Task(
            request_id=request_id,
            action=action,
            params=params,
            status="queued",
            message="Waiting for CMD response"
        )
        
        self.active_tasks[request_id] = task
        logger.info(f"✓ Task created: {request_id} ({action})")
        return task

    def update_task(self, response: Dict) -> Optional[Task]:
        """
        Update task (called when receiving CMD response)
        
        Intelligent matching logic:
        1. Try to match using response.data.task_id (task_id returned by navigation task)
        2. Try to match using response.request_id (request ID)
        3. Ignore if not in cache (not our task)
        
        Args:
            response: Dictionary representation of CommandResponse
        
        Returns:
            Optional[Task]: Updated task, or None if not related
        """
        request_id = response.get('header', {}).get('request_id')
        data = response.get('body', {}).get('data', {})
        status_code = response.get('body', {}).get('status', '')
        message = response.get('body', {}).get('message', '')
        
        # Try to match task_id (returned by MissionPlanner)
        task_id = data.get('task_id')
        matched_request_id = None
        
        if task_id and task_id in self.task_id_map:
            matched_request_id = self.task_id_map[task_id]
        elif request_id and request_id in self.active_tasks:
            matched_request_id = request_id
        else:
            # Not in cache, ignore
            logger.debug(f"Ignoring response (not in cache): request_id={request_id}, task_id={task_id}")
            return None
        
        task = self.active_tasks[matched_request_id]
        
        # Update task status
        task.updated_at = datetime.now().isoformat()
        
        # 映射 CMD 状态到任务状态
        if status_code == 'completed':
            # 任务创建成功，获取 task_id
            if task_id and not task.task_id:
                task.task_id = task_id
                self.task_id_map[task_id] = matched_request_id
                task.status = 'executing'
                task.message = message or "Task created"
                logger.info(f"✓ Task upgraded: {matched_request_id} -> task_id={task_id}")
            else:
                # 查询状态返回，解析具体状态
                task_state = data.get('state', data.get('task_status', ''))
                task.message = message
                
                # 映射 MissionPlanner 状态
                if task_state in ['RUNNING', 'EXECUTING']:
                    task.status = 'executing'
                    task.progress = data.get('progress', task.progress)
                elif task_state in ['SUCCESS', 'COMPLETED']:
                    task.status = 'completed'
                    task.progress = 100.0
                    task.completed_at = task.updated_at
                    self._archive_task(matched_request_id)
                elif task_state == 'PAUSED':
                    task.status = 'paused'
                elif task_state in ['FAILED', 'ERROR']:
                    task.status = 'failed'
                    task.completed_at = task.updated_at
                    self._archive_task(matched_request_id)
                elif task_state in ['CANCELLED', 'CANCELED']:  # 支持英式和美式拼写
                    task.status = 'cancelled'
                    task.completed_at = task.updated_at
                    self._archive_task(matched_request_id)
                
        elif status_code == 'failed':
            task.status = 'failed'
            task.message = message
            task.completed_at = task.updated_at
            self._archive_task(matched_request_id)
        
        logger.debug(f"✓ Task updated: {matched_request_id} -> {task.status} ({task.message})")
        return task

    def get_active_tasks(self) -> List[Dict]:
        """
        Get all active tasks
        
        Returns:
            List[Dict]: Task list (dictionary format)
        """
        return [task.to_dict() for task in self.active_tasks.values()]

    def get_task_history(self, limit: int = 50) -> List[Dict]:
        """
        Get task history
        
        Args:
            limit: Maximum number to return
        
        Returns:
            List[Dict]: Historical task list
        """
        return [task.to_dict() for task in self.task_history[-limit:]]

    def get_task_by_request_id(self, request_id: str) -> Optional[Task]:
        """Get task by request ID"""
        return self.active_tasks.get(request_id)

    def get_task_by_task_id(self, task_id: str) -> Optional[Task]:
        """Get task by task ID (task_id is string like nav_xxx)"""
        # 遍历active_tasks，匹配task.task_id
        for task in self.active_tasks.values():
            if task.task_id == task_id:
                return task
        
        # 也检查历史任务
        for task in self.task_history:
            if task.task_id == task_id:
                return task
        
        return None

    def _archive_task(self, request_id: str):
        """
        归档已完成的任务
        
        Args:
            request_id: 请求 ID
        """
        if request_id not in self.active_tasks:
            return
        
        task = self.active_tasks.pop(request_id)
        
        # Remove from task_id_map
        if task.task_id and task.task_id in self.task_id_map:
            del self.task_id_map[task.task_id]
        
        # Add to history
        self.task_history.append(task)
        
        # Persist (async write to avoid blocking)
        self._save_history_async()
        
        logger.info(f"✓ Task archived: {request_id} ({task.action}) -> {task.status}")

    def _save_history_async(self):
        """Asynchronously save task history to file"""
        try:
            # Only keep latest 100 records
            history_to_save = self.task_history[-100:]
            
            with open(self.history_file, 'w', encoding='utf-8') as f:
                json.dump(
                    [task.to_dict() for task in history_to_save],
                    f,
                    indent=2,
                    ensure_ascii=False
                )
            
            logger.debug(f"✓ Task history saved: {len(history_to_save)} tasks")
        
        except Exception as e:
            logger.error(f"Failed to save task history: {e}")

    def _load_history(self) -> List[Task]:
        """Load task history from file"""
        if not self.history_file.exists():
            return []
        
        try:
            with open(self.history_file, 'r', encoding='utf-8') as f:
                data = json.load(f)
                return [Task.from_dict(task_dict) for task_dict in data]
        
        except Exception as e:
            logger.warning(f"Failed to load task history: {e}")
            return []


# Global instance
task_manager = TaskManager()
