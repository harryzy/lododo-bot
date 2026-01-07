"""
bot_cmd_interface核心组件模块 / Core components module

包含请求队列、服务适配器、响应发布器等核心组件 /
Contains core components including request queue, service adapter, and response publisher
"""

from .request_queue import RequestQueue
# ServiceAdapter和ResponsePublisher将在Phase 2.2和2.3实现
# ServiceAdapter and ResponsePublisher will be implemented in Phase 2.2 and 2.3

__all__ = [
    'RequestQueue',
]

