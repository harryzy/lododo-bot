"""
Deprecated exploration_mapper module

原独立的 exploration_mapper 节点已废弃，功能已迁移到统一的任务执行架构。

新架构:
- ExplorationHandler: 探索任务处理器（继承 TaskExecutionHandler）
- 通过 MissionPlanner 统一管理，而不是作为独立节点运行

备份文件:
- exploration_mapper_backup.py: 原始实现的完整备份

迁移日期: 2025-12-30
"""

# 此文件仅用于标记废弃状态
__all__ = []
