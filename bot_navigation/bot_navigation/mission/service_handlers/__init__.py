#!/usr/bin/env python3
"""
__init__.py - 服务处理器模块初始化文件

重构说明:
- waypoint_service_handler → waypoint_tools (工具模块)
- navigation_service_handler → 废弃（功能移到 MissionPlanner 和 NavigationHandler）
- deprecated_task_handler → 旧版本备份
- task_execution_handler → 新的基类
- handlers/ → 具体的任务处理器（NavigationHandler, PatrolHandler, ExplorationHandler）
"""

from .waypoint_tools import WaypointTools
from .task_execution_handler import TaskExecutionHandler

# 导入具体的任务处理器
from .handlers import NavigationHandler, PatrolHandler, ExplorationHandler

__all__ = [
    'WaypointTools',
    'TaskExecutionHandler',
    'NavigationHandler',
    'PatrolHandler',
    'ExplorationHandler',
]
