"""
waypoint - 路点工具模块 / Waypoint Tools Module

提供路点录制、保存、加载等功能
独立工具模块，不依赖任务管理系统

Modules:
  - waypoint_recorder: 路点记录核心逻辑
  - waypoint_recorder_node: 交互式CLI节点
  - waypoint_service: 服务接口封装
"""

from .waypoint_recorder import WaypointRecorder
from .waypoint_service import WaypointTools

__all__ = ['WaypointRecorder', 'WaypointTools']
