#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Task Manager - 任务管理核心组件
负责任务的创建、更新、持久化和查询

Architecture: 被动接收 CMD 响应，不主动轮询
- 通过 WebTerminalNode 的 _response_callback 接收所有 /cmd/response 消息
- 智能匹配 task_id 或 request_id，更新缓存中的任务
- 忽略不相关的响应（not in cache）
- JSON 文件持久化任务历史
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
    """任务数据模型"""
    task_id: Optional[str] = None  # MissionPlanner 返回的任务 ID（字符串如nav_xxx）
    request_id: str = ""  # CMD 请求 ID
    action: str = ""  # 任务类型 (navigate_to_pose, start_exploration, etc.)
    params: Dict = None  # 任务参数
    status: str = "queued"  # queued, executing, paused, completed, failed, cancelled
    progress: float = 0.0  # 进度 0-100
    message: str = ""  # 状态消息
    created_at: str = ""  # 创建时间
    updated_at: str = ""  # 更新时间
    completed_at: Optional[str] = None  # 完成时间

    def __post_init__(self):
        if self.params is None:
            self.params = {}
        if not self.created_at:
            self.created_at = datetime.now().isoformat()
        if not self.updated_at:
            self.updated_at = self.created_at

    def to_dict(self) -> Dict:
        """转换为字典"""
        return asdict(self)

    @classmethod
    def from_dict(cls, data: Dict) -> 'Task':
        """从字典创建任务"""
        return cls(**data)


class TaskManager:
    """
    任务管理器 - 单例模式
    
    核心职责：
    1. 创建任务（用户发起操作时）
    2. 更新任务（接收 CMD 响应时）
    3. 查询任务（用户请求状态时）
    4. 持久化任务历史
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
        
        # 加载历史任务（仅用于展示，不参与逻辑）
        self.task_history: List[Task] = self._load_history()
        
        logger.info(f"✓ TaskManager initialized. History file: {self.history_file}")

    def create_task(self, request_id: str, action: str, params: Dict) -> Task:
        """
        创建新任务（用户发起操作时调用）
        
        Args:
            request_id: CMD 请求 ID（唯一标识）
            action: 任务类型
            params: 任务参数
        
        Returns:
            Task: 创建的任务对象
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
        更新任务（接收 CMD 响应时调用）
        
        智能匹配逻辑：
        1. 尝试用 response.data.task_id 匹配（导航任务返回的 task_id）
        2. 尝试用 response.request_id 匹配（请求 ID）
        3. 如果都不在缓存中，忽略（not our task）
        
        Args:
            response: CommandResponse 的字典表示
        
        Returns:
            Optional[Task]: 更新的任务，如果不相关则返回 None
        """
        request_id = response.get('header', {}).get('request_id')
        data = response.get('body', {}).get('data', {})
        status_code = response.get('body', {}).get('status', '')
        message = response.get('body', {}).get('message', '')
        
        # 尝试匹配 task_id（MissionPlanner 返回的）
        task_id = data.get('task_id')
        matched_request_id = None
        
        if task_id and task_id in self.task_id_map:
            matched_request_id = self.task_id_map[task_id]
        elif request_id and request_id in self.active_tasks:
            matched_request_id = request_id
        else:
            # 不在缓存中，忽略
            logger.debug(f"Ignoring response (not in cache): request_id={request_id}, task_id={task_id}")
            return None
        
        task = self.active_tasks[matched_request_id]
        
        # 更新任务状态
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
        获取所有活跃任务
        
        Returns:
            List[Dict]: 任务列表（字典格式）
        """
        return [task.to_dict() for task in self.active_tasks.values()]

    def get_task_history(self, limit: int = 50) -> List[Dict]:
        """
        获取任务历史记录
        
        Args:
            limit: 返回数量限制
        
        Returns:
            List[Dict]: 历史任务列表
        """
        return [task.to_dict() for task in self.task_history[-limit:]]

    def get_task_by_request_id(self, request_id: str) -> Optional[Task]:
        """根据请求 ID 获取任务"""
        return self.active_tasks.get(request_id)

    def get_task_by_task_id(self, task_id: str) -> Optional[Task]:
        """根据任务 ID 获取任务（task_id为字符串如nav_xxx）"""
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
        
        # 从 task_id_map 中移除
        if task.task_id and task.task_id in self.task_id_map:
            del self.task_id_map[task.task_id]
        
        # 添加到历史记录
        self.task_history.append(task)
        
        # 持久化（异步写入，避免阻塞）
        self._save_history_async()
        
        logger.info(f"✓ Task archived: {request_id} ({task.action}) -> {task.status}")

    def _save_history_async(self):
        """异步保存历史记录到文件"""
        try:
            # 只保留最近 100 条记录
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
        """从文件加载历史记录"""
        if not self.history_file.exists():
            return []
        
        try:
            with open(self.history_file, 'r', encoding='utf-8') as f:
                data = json.load(f)
                return [Task.from_dict(task_dict) for task_dict in data]
        
        except Exception as e:
            logger.warning(f"Failed to load task history: {e}")
            return []


# 全局实例
task_manager = TaskManager()
