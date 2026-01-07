"""
bot_cmd_interface SDK - 统一命令消息构造工具 / Unified command message construction toolkit

Version: 1.0.0
Author: LeKiwi Robot Team
License: MIT

This SDK provides a standardized way to construct command requests and parse responses
for the LeKiwi robot unified command interface.

统一命令接口SDK，为LeKiwi机器人提供标准化的命令请求构造和响应解析工具。
"""

# 核心消息类 / Core message classes
from .message import (
    CommandRequest,
    CommandResponse,
    ResponseStatus,
    ErrorCode,
    create_navigate_request,
    create_patrol_request,
    create_exploration_request,
    create_emergency_stop_request,
    create_get_status_request,
    create_cancel_task_request,
)

# 动作定义 / Action definitions
from .action_types import ActionType

# 验证器 / Validators
from .validators import validate_request, validate_response

__version__ = '1.0.0'

__all__ = [
    # 消息类 / Message classes
    'CommandRequest',
    'CommandResponse',
    'ResponseStatus',
    'ErrorCode',
    
    # 便捷构造函数 / Convenience constructors
    'create_navigate_request',
    'create_patrol_request',
    'create_exploration_request',
    'create_emergency_stop_request',
    'create_get_status_request',
    'create_cancel_task_request',
    
    # 动作类型 / Action types
    'ActionType',
    
    # 验证器 / Validators
    'validate_request',
    'validate_response',
]
