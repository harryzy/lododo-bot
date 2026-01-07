"""
命令请求和响应消息类 / Command request and response message classes

定义统一的JSON消息格式，用于终端与CommandAdapter之间的通信 /
Defines unified JSON message format for communication between terminals and CommandAdapter
"""

import json
import uuid
from datetime import datetime
from typing import Optional, Tuple, Dict, Any, List

from .action_types import ActionType


class ResponseStatus:
    """
    响应状态常量 / Response status constants
    """
    QUEUED = "queued"
    """请求已入队 / Request queued"""
    
    EXECUTING = "executing"
    """请求正在执行 / Request executing"""
    
    COMPLETED = "completed"
    """请求已完成 / Request completed"""
    
    FAILED = "failed"
    """请求失败 / Request failed"""
    
    CANCELLED = "cancelled"
    """请求已取消 / Request cancelled"""
    
    ALL_STATUSES = [QUEUED, EXECUTING, COMPLETED, FAILED, CANCELLED]


class ErrorCode:
    """
    统一错误码定义 / Unified error code definitions
    
    基于HTTP状态码规范，提供统一的错误码体系 /
    Based on HTTP status codes, provides unified error code system
    """
    
    SUCCESS = 0
    """成功 / Success"""
    
    BAD_REQUEST = 400
    """请求格式错误或缺少必需参数 / Bad request format or missing required parameters"""
    
    NOT_FOUND = 404
    """资源未找到（如task_id不存在）/ Resource not found (e.g., task_id does not exist)"""
    
    CONFLICT = 409
    """状态冲突（如重复请求、队列满）/ State conflict (e.g., duplicate request, queue full)"""
    
    CLIENT_CLOSED = 499
    """客户端关闭连接 / Client closed connection"""
    
    INTERNAL_ERROR = 500
    """服务内部错误 / Internal server error"""
    
    SERVICE_UNAVAILABLE = 503
    """服务不可用（可重试）/ Service unavailable (retryable)"""
    
    GATEWAY_TIMEOUT = 504
    """网关超时（可重试）/ Gateway timeout (retryable)"""
    
    @staticmethod
    def get_message(code: int) -> str:
        """
        获取错误码对应的描述信息 / Get error message for error code
        
        Args:
            code: 错误码 / Error code
            
        Returns:
            str: 错误描述 / Error description
        """
        messages = {
            0: "Success",
            400: "Bad request format or missing required parameters",
            404: "Resource not found",
            409: "State conflict (duplicate request or queue full)",
            499: "Client closed connection",
            500: "Internal server error",
            503: "Service unavailable",
            504: "Gateway timeout",
        }
        return messages.get(code, "Unknown error")
    
    @staticmethod
    def is_retryable(code: int) -> bool:
        """
        判断错误码是否可重试 / Check if error code is retryable
        
        Args:
            code: 错误码 / Error code
            
        Returns:
            bool: 是否可重试 / Whether retryable
        """
        return code in [ErrorCode.SERVICE_UNAVAILABLE, ErrorCode.GATEWAY_TIMEOUT]


class CommandRequest:
    """
    请求消息封装类 / Request message wrapper class
    
    用于构造发送到/cmd/request的JSON消息 /
    Used to construct JSON messages sent to /cmd/request
    
    消息格式 / Message format:
    {
        "header": {
            "request_id": "uuid-string",
            "timestamp": "ISO8601",
            "priority": 3
        },
        "body": {
            "action": "navigate_to_pose",
            "params": {...},
            "timeout": 300.0
        }
    }
    """
    
    def __init__(
        self,
        action: str,
        params: dict,
        request_id: Optional[str] = None,
        priority: int = 3,
        timeout: float = 300.0
    ):
        """
        初始化命令请求 / Initialize command request
        
        Args:
            action: 动作类型，必须是ActionType中定义的值 / 
                   Action type, must be from ActionType
            params: 动作参数字典 / Action parameters dictionary
            request_id: 请求ID，默认自动生成UUID / 
                       Request ID, auto-generated UUID by default
            priority: 优先级 (1=最高, 5=最低)，默认3 / 
                     Priority (1=highest, 5=lowest), default 3
            timeout: 超时时间（秒），默认300秒 / 
                    Timeout in seconds, default 300s
        """
        self.request_id = request_id or str(uuid.uuid4())
        self.timestamp = datetime.now().isoformat()
        self.action = action
        self.params = params
        self.priority = priority
        self.timeout = timeout
    
    def to_json(self) -> str:
        """
        转换为JSON字符串 / Convert to JSON string
        
        Returns:
            str: JSON格式的请求消息 / JSON formatted request message
        """
        message = {
            "header": {
                "request_id": self.request_id,
                "timestamp": self.timestamp,
                "priority": self.priority
            },
            "body": {
                "action": self.action,
                "params": self.params,
                "timeout": self.timeout
            }
        }
        return json.dumps(message, ensure_ascii=False)
    
    @classmethod
    def from_json(cls, json_str: str) -> 'CommandRequest':
        """
        从JSON字符串解析 / Parse from JSON string
        
        Args:
            json_str: JSON格式的请求消息 / JSON formatted request message
            
        Returns:
            CommandRequest: 解析后的请求对象 / Parsed request object
            
        Raises:
            json.JSONDecodeError: JSON格式错误 / Invalid JSON format
            KeyError: 缺少必需字段 / Missing required fields
        """
        data = json.loads(json_str)
        header = data.get('header', {})
        body = data.get('body', {})
        
        return cls(
            action=body['action'],
            params=body.get('params', {}),
            request_id=header.get('request_id'),
            priority=header.get('priority', 3),
            timeout=body.get('timeout', 300.0)
        )
    
    def validate(self) -> Tuple[bool, str]:
        """
        验证请求消息合法性 / Validate request message
        
        Returns:
            Tuple[bool, str]: (是否合法, 错误信息) / (is_valid, error_message)
        """
        # 验证action / Validate action
        if not self.action:
            return False, "Missing action field"
        
        if not ActionType.is_valid_action(self.action):
            return False, f"Invalid action: {self.action}"
        
        # 验证params / Validate params
        if not isinstance(self.params, dict):
            return False, "Params must be a dictionary"
        
        # 验证优先级 / Validate priority
        if not isinstance(self.priority, int) or not (1 <= self.priority <= 5):
            return False, "Priority must be an integer between 1 and 5"
        
        # 验证超时 / Validate timeout
        if not isinstance(self.timeout, (int, float)) or self.timeout <= 0:
            return False, "Timeout must be a positive number"
        
        return True, "OK"
    
    def __repr__(self) -> str:
        return (f"CommandRequest(request_id='{self.request_id}', "
                f"action='{self.action}', priority={self.priority})")


class CommandResponse:
    """
    响应消息封装类 / Response message wrapper class
    
    用于解析/cmd/response的JSON消息 /
    Used to parse JSON messages from /cmd/response
    
    ⚠️ 重要提示 / IMPORTANT NOTE:
    - progress参数仅用于get_task_status查询响应 /
      progress is only for get_task_status query responses
    - 请求处理响应（queued/executing/completed）不应包含progress /
      Request lifecycle responses should NOT include progress
    
    消息格式 / Message format:
    {
        "header": {
            "request_id": "uuid-string",
            "timestamp": "ISO8601",
            "status": "completed"
        },
        "body": {
            "message": "Success",
            "code": 0,
            "result": {...},
            "progress": 0.75,  // ⚠️ 仅用于get_task_status / Only for get_task_status
            "warnings": [...]
        }
    }
    """
    
    def __init__(
        self,
        request_id: str,
        status: str,
        message: str,
        code: int = 0,
        result: Optional[Dict[str, Any]] = None,
        progress: Optional[float] = None,
        warnings: Optional[List[str]] = None
    ):
        """
        初始化命令响应 / Initialize command response
        
        Args:
            request_id: 对应的请求ID / Corresponding request ID
            status: 响应状态，必须是ResponseStatus中定义的值 / 
                   Response status, must be from ResponseStatus
            message: 响应消息 / Response message
            code: 错误码，0表示成功 / Error code, 0 for success
            result: 结果数据字典（可选）/ Result data dictionary (optional)
            progress: 任务进度 0.0-1.0（仅用于get_task_status）/ 
                     Task progress 0.0-1.0 (only for get_task_status)
            warnings: 警告信息列表（可选）/ Warning messages list (optional)
        """
        self.request_id = request_id
        self.timestamp = datetime.now().isoformat()
        self.status = status
        self.message = message
        self.code = code
        self.result = result or {}
        self.progress = progress
        self.warnings = warnings or []
    
    def to_json(self) -> str:
        """
        转换为JSON字符串 / Convert to JSON string
        
        Returns:
            str: JSON格式的响应消息 / JSON formatted response message
        """
        body = {
            "message": self.message,
            "code": self.code
        }
        
        # 只在有值时添加可选字段 / Only add optional fields when present
        if self.result:
            body["result"] = self.result
        if self.progress is not None:
            body["progress"] = self.progress
        if self.warnings:
            body["warnings"] = self.warnings
        
        message = {
            "header": {
                "request_id": self.request_id,
                "timestamp": self.timestamp,
                "status": self.status
            },
            "body": body
        }
        return json.dumps(message, ensure_ascii=False)
    
    @classmethod
    def from_json(cls, json_str: str) -> 'CommandResponse':
        """
        从JSON字符串解析 / Parse from JSON string
        
        Args:
            json_str: JSON格式的响应消息 / JSON formatted response message
            
        Returns:
            CommandResponse: 解析后的响应对象 / Parsed response object
            
        Raises:
            json.JSONDecodeError: JSON格式错误 / Invalid JSON format
            KeyError: 缺少必需字段 / Missing required fields
        """
        data = json.loads(json_str)
        header = data.get('header', {})
        body = data.get('body', {})
        
        return cls(
            request_id=header['request_id'],
            status=header['status'],
            message=body['message'],
            code=body.get('code', 0),
            result=body.get('result'),
            progress=body.get('progress'),
            warnings=body.get('warnings')
        )
    
    def is_success(self) -> bool:
        """
        判断响应是否成功 / Check if response is successful
        
        Returns:
            bool: 是否成功 / Whether successful
        """
        return self.code == ErrorCode.SUCCESS and self.status == ResponseStatus.COMPLETED
    
    def is_final(self) -> bool:
        """
        判断是否为最终响应（不再有后续响应）/ Check if this is a final response
        
        Returns:
            bool: 是否为最终响应 / Whether this is final
        """
        return self.status in [ResponseStatus.COMPLETED, ResponseStatus.FAILED, ResponseStatus.CANCELLED]
    
    def __repr__(self) -> str:
        return (f"CommandResponse(request_id='{self.request_id}', "
                f"status='{self.status}', code={self.code})")


# ========== 便捷构造函数 / Convenience constructors ==========

def create_navigate_request(
    x: float,
    y: float,
    yaw: Optional[float] = None,
    timeout: Optional[float] = None,
    priority: int = 3
) -> CommandRequest:
    """
    创建导航到目标点请求 / Create navigate to pose request
    
    Args:
        x: 目标X坐标 (米) / Target X coordinate (meters)
        y: 目标Y坐标 (米) / Target Y coordinate (meters)
        yaw: 目标朝向 (弧度)，可选 / Target yaw (radians), optional
        timeout: 超时时间（秒），可选 / Timeout (seconds), optional
        priority: 优先级，默认3 / Priority, default 3
        
    Returns:
        CommandRequest: 导航请求对象 / Navigation request object
        
    Example:
        >>> request = create_navigate_request(1.5, 2.0, yaw=0.785)
        >>> print(request.to_json())
    """
    params = {
        "goal_pose": {
            "position": {"x": x, "y": y}
        }
    }
    
    if yaw is not None:
        params["goal_pose"]["orientation"] = {"yaw": yaw}
    
    kwargs = {
        "action": ActionType.NAVIGATE_TO_POSE,
        "params": params,
        "priority": priority
    }
    
    if timeout is not None:
        kwargs["timeout"] = timeout
    
    return CommandRequest(**kwargs)


def create_patrol_request(
    waypoint_file: str,
    mode: str = "loop",
    timeout: Optional[float] = None,
    priority: int = 3
) -> CommandRequest:
    """
    创建巡航任务请求 / Create patrol task request
    
    Args:
        waypoint_file: 航点文件路径 / Waypoint file path
        mode: 巡航模式，"loop"(循环) 或 "once"(单次) / 
              Patrol mode, "loop" or "once"
        timeout: 超时时间（秒），可选 / Timeout (seconds), optional
        priority: 优先级，默认3 / Priority, default 3
        
    Returns:
        CommandRequest: 巡航请求对象 / Patrol request object
        
    Example:
        >>> request = create_patrol_request("route1.yaml", mode="loop")
    """
    params = {
        "waypoint_file": waypoint_file,
        "mode": mode
    }
    
    kwargs = {
        "action": ActionType.START_PATROL,
        "params": params,
        "priority": priority
    }
    
    if timeout is not None:
        kwargs["timeout"] = timeout
    else:
        kwargs["timeout"] = 3600.0  # 巡航默认1小时超时 / Patrol default 1 hour timeout
    
    return CommandRequest(**kwargs)


def create_exploration_request(
    map_name: Optional[str] = None,
    save_on_completion: bool = True,
    timeout: Optional[float] = None,
    priority: int = 3
) -> CommandRequest:
    """
    创建自主探索建图请求 / Create autonomous exploration request
    
    Args:
        map_name: 地图名称，可选 / Map name, optional
        save_on_completion: 完成后是否保存地图，默认True / 
                           Whether to save map on completion, default True
        timeout: 超时时间（秒），可选 / Timeout (seconds), optional
        priority: 优先级，默认3 / Priority, default 3
        
    Returns:
        CommandRequest: 探索请求对象 / Exploration request object
        
    Example:
        >>> request = create_exploration_request(map_name="office_floor1")
    """
    params = {
        "save_on_completion": save_on_completion
    }
    
    if map_name:
        params["map_name"] = map_name
    
    kwargs = {
        "action": ActionType.START_EXPLORATION,
        "params": params,
        "priority": priority
    }
    
    if timeout is not None:
        kwargs["timeout"] = timeout
    else:
        kwargs["timeout"] = 3600.0  # 探索默认1小时超时 / Exploration default 1 hour timeout
    
    return CommandRequest(**kwargs)


def create_emergency_stop_request(
    force_immediate: bool = False,
    priority: int = 1
) -> CommandRequest:
    """
    创建紧急停止请求 / Create emergency stop request
    
    ⚠️ 紧急停止具有最高优先级，会抢占队列 /
    Emergency stop has highest priority and will preempt the queue
    
    Args:
        force_immediate: 是否强制立即停止，默认False / 
                        Whether to force immediate stop, default False
        priority: 优先级，默认1（最高）/ Priority, default 1 (highest)
        
    Returns:
        CommandRequest: 紧急停止请求对象 / Emergency stop request object
        
    Example:
        >>> request = create_emergency_stop_request(force_immediate=True)
    """
    params = {
        "force_immediate": force_immediate
    }
    
    return CommandRequest(
        action=ActionType.EMERGENCY_STOP,
        params=params,
        priority=priority,
        timeout=5.0  # 紧急停止5秒超时 / Emergency stop 5s timeout
    )


def create_get_status_request(
    task_id: Optional[str] = None,
    request_id: Optional[str] = None,
    timeout: Optional[float] = None,
    priority: int = 3
) -> CommandRequest:
    """
    创建任务状态查询请求 / Create task status query request
    
    至少提供task_id或request_id之一 / At least one of task_id or request_id must be provided
    
    Args:
        task_id: 任务ID / Task ID
        request_id: 原始请求ID / Original request ID
        timeout: 超时时间（秒），可选 / Timeout (seconds), optional
        priority: 优先级，默认3 / Priority, default 3
        
    Returns:
        CommandRequest: 状态查询请求对象 / Status query request object
        
    Example:
        >>> request = create_get_status_request(task_id="task-001")
        >>> # 或者 / Or
        >>> request = create_get_status_request(request_id="req-uuid")
    """
    params = {}
    
    if task_id:
        params["task_id"] = task_id
    if request_id:
        params["request_id"] = request_id
    
    if not params:
        raise ValueError("At least one of task_id or request_id must be provided")
    
    kwargs = {
        "action": ActionType.GET_TASK_STATUS,
        "params": params,
        "priority": priority
    }
    
    if timeout is not None:
        kwargs["timeout"] = timeout
    else:
        kwargs["timeout"] = 5.0  # 查询默认5秒超时 / Query default 5s timeout
    
    return CommandRequest(**kwargs)


def create_cancel_task_request(
    task_id: Optional[str] = None,
    request_id: Optional[str] = None,
    timeout: Optional[float] = None,
    priority: int = 2
) -> CommandRequest:
    """
    创建取消任务请求 / Create cancel task request
    
    至少提供task_id或request_id之一 / At least one of task_id or request_id must be provided
    
    Args:
        task_id: 任务ID / Task ID
        request_id: 原始请求ID / Original request ID
        timeout: 超时时间（秒），可选 / Timeout (seconds), optional
        priority: 优先级，默认2 / Priority, default 2
        
    Returns:
        CommandRequest: 取消任务请求对象 / Cancel task request object
        
    Example:
        >>> request = create_cancel_task_request(task_id="task-001")
    """
    params = {}
    
    if task_id:
        params["task_id"] = task_id
    if request_id:
        params["request_id"] = request_id
    
    if not params:
        raise ValueError("At least one of task_id or request_id must be provided")
    
    kwargs = {
        "action": ActionType.CANCEL_TASK,
        "params": params,
        "priority": priority
    }
    
    if timeout is not None:
        kwargs["timeout"] = timeout
    else:
        kwargs["timeout"] = 10.0  # 取消默认10秒超时 / Cancel default 10s timeout
    
    return CommandRequest(**kwargs)
