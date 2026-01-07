"""
请求和响应验证器 / Request and response validators

提供请求消息和响应消息的合法性验证功能 /
Provides validation functionality for request and response messages
"""

from typing import Tuple, Dict, Any
from .message import CommandRequest, CommandResponse, ResponseStatus, ErrorCode
from .action_types import ActionType


# ========== 必需参数定义 / Required parameters definition ==========

REQUIRED_PARAMS = {
    ActionType.NAVIGATE_TO_POSE: ['goal_pose'],
    ActionType.NAVIGATE_TO_LOCATION: ['location'],
    ActionType.START_EXPLORATION: [],  # 所有参数可选 / All parameters optional
    ActionType.START_PATROL: ['waypoint_file'],
    ActionType.STOP_PATROL: [],  # 无参数要求 / No parameters required
    ActionType.PAUSE_TASK: [],  # 无参数要求 / No parameters required
    ActionType.RESUME_TASK: [],  # 无参数要求 / No parameters required
    ActionType.CANCEL_TASK: [],  # task_id或request_id至少一个 / task_id or request_id required
    ActionType.GET_TASK_STATUS: [],  # task_id或request_id至少一个 / task_id or request_id required
    ActionType.GET_ROBOT_STATUS: [],  # 无参数要求 / No parameters required
    ActionType.EMERGENCY_STOP: [],  # 所有参数可选 / All parameters optional
    ActionType.SAVE_MAP: [],  # map_name可选 / map_name optional
    ActionType.LOAD_MAP: ['map_name'],
}


def validate_request(request: CommandRequest) -> Tuple[bool, str]:
    """
    验证请求消息合法性 / Validate request message
    
    检查项 / Checks:
    1. action是否合法 / Is action valid
    2. request_id是否存在 / request_id exists
    3. params是否为dict / params is dict
    4. 必需参数是否齐全 / Required params present
    5. 参数值类型是否正确 / Parameter value types correct
    
    Args:
        request: 待验证的请求对象 / Request object to validate
        
    Returns:
        Tuple[bool, str]: (是否合法, 错误信息) / (is_valid, error_message)
        
    Example:
        >>> request = create_navigate_request(1.0, 2.0)
        >>> valid, msg = validate_request(request)
        >>> if not valid:
        ...     print(f"Validation failed: {msg}")
    """
    # 基础验证 / Basic validation
    valid, msg = request.validate()
    if not valid:
        return False, msg
    
    # 检查动作是否有必需参数定义 / Check if action has required params definition
    if request.action not in REQUIRED_PARAMS:
        return False, f"Unknown action: {request.action}"
    
    required_params = REQUIRED_PARAMS[request.action]
    
    # 检查必需参数 / Check required parameters
    for param in required_params:
        if param not in request.params:
            return False, f"Missing required parameter: {param}"
    
    # 特定动作的参数验证 / Action-specific parameter validation
    if request.action == ActionType.NAVIGATE_TO_POSE:
        return _validate_navigate_params(request.params)
    
    elif request.action == ActionType.NAVIGATE_TO_LOCATION:
        if not isinstance(request.params.get('location'), str):
            return False, "Parameter 'location' must be a string"
    
    elif request.action == ActionType.START_PATROL:
        if not isinstance(request.params.get('waypoint_file'), str):
            return False, "Parameter 'waypoint_file' must be a string"
        mode = request.params.get('mode', 'loop')
        if mode not in ['loop', 'once']:
            return False, "Parameter 'mode' must be 'loop' or 'once'"
    
    elif request.action in [ActionType.GET_TASK_STATUS, ActionType.CANCEL_TASK]:
        # 至少需要task_id或request_id之一 / At least task_id or request_id required
        if 'task_id' not in request.params and 'request_id' not in request.params:
            return False, "Either 'task_id' or 'request_id' must be provided"
    
    elif request.action == ActionType.LOAD_MAP:
        if not isinstance(request.params.get('map_name'), str):
            return False, "Parameter 'map_name' must be a string"
    
    return True, "OK"


def _validate_navigate_params(params: Dict[str, Any]) -> Tuple[bool, str]:
    """
    验证导航参数 / Validate navigation parameters
    
    Args:
        params: 参数字典 / Parameters dictionary
        
    Returns:
        Tuple[bool, str]: (是否合法, 错误信息) / (is_valid, error_message)
    """
    goal_pose = params.get('goal_pose')
    if not isinstance(goal_pose, dict):
        return False, "Parameter 'goal_pose' must be a dictionary"
    
    # 检查position / Check position
    position = goal_pose.get('position')
    if not isinstance(position, dict):
        return False, "Parameter 'goal_pose.position' must be a dictionary"
    
    if 'x' not in position or 'y' not in position:
        return False, "Missing 'x' or 'y' in 'goal_pose.position'"
    
    try:
        float(position['x'])
        float(position['y'])
    except (TypeError, ValueError):
        return False, "'x' and 'y' must be numeric"
    
    # 检查orientation（可选）/ Check orientation (optional)
    orientation = goal_pose.get('orientation')
    if orientation is not None:
        if not isinstance(orientation, dict):
            return False, "Parameter 'goal_pose.orientation' must be a dictionary"
        
        if 'yaw' in orientation:
            try:
                float(orientation['yaw'])
            except (TypeError, ValueError):
                return False, "'yaw' must be numeric"
    
    return True, "OK"


def validate_response(response: CommandResponse) -> Tuple[bool, str]:
    """
    验证响应消息合法性 / Validate response message
    
    检查项 / Checks:
    1. request_id是否存在 / request_id exists
    2. status是否合法 / status is valid
    3. message是否存在 / message exists
    4. code是否为整数 / code is integer
    5. progress字段使用是否正确 / progress field usage is correct
    
    Args:
        response: 待验证的响应对象 / Response object to validate
        
    Returns:
        Tuple[bool, str]: (是否合法, 错误信息) / (is_valid, error_message)
        
    Example:
        >>> response = CommandResponse(
        ...     request_id="req-001",
        ...     status="completed",
        ...     message="Success",
        ...     code=0
        ... )
        >>> valid, msg = validate_response(response)
    """
    # 检查request_id / Check request_id
    if not response.request_id:
        return False, "Missing request_id"
    
    # 检查status / Check status
    if response.status not in ResponseStatus.ALL_STATUSES:
        return False, f"Invalid status: {response.status}"
    
    # 检查message / Check message
    if not response.message:
        return False, "Missing message"
    
    # 检查code / Check code
    if not isinstance(response.code, int):
        return False, "Code must be an integer"
    
    # 检查progress字段 / Check progress field
    if response.progress is not None:
        if not isinstance(response.progress, (int, float)):
            return False, "Progress must be numeric"
        
        if not (0.0 <= response.progress <= 1.0):
            return False, "Progress must be between 0.0 and 1.0"
        
        # ⚠️ 警告：progress应仅在get_task_status响应中使用 /
        # WARNING: progress should only be used in get_task_status responses
        if response.status in [ResponseStatus.QUEUED, ResponseStatus.EXECUTING]:
            # 这是一个软警告，不阻止验证通过 / Soft warning, doesn't fail validation
            # 但应该记录日志提醒开发者 / But should log warning to alert developers
            pass
    
    # 检查result / Check result
    if response.result is not None and not isinstance(response.result, dict):
        return False, "Result must be a dictionary"
    
    # 检查warnings / Check warnings
    if response.warnings is not None:
        if not isinstance(response.warnings, list):
            return False, "Warnings must be a list"
        for warning in response.warnings:
            if not isinstance(warning, str):
                return False, "Each warning must be a string"
    
    return True, "OK"


def validate_action_params(action: str, params: Dict[str, Any]) -> Tuple[bool, str]:
    """
    验证指定动作的参数 / Validate parameters for specific action
    
    这是一个便捷函数，用于在构造请求前验证参数 /
    This is a convenience function for validating params before constructing request
    
    Args:
        action: 动作类型 / Action type
        params: 参数字典 / Parameters dictionary
        
    Returns:
        Tuple[bool, str]: (是否合法, 错误信息) / (is_valid, error_message)
        
    Example:
        >>> params = {"goal_pose": {"position": {"x": 1.0, "y": 2.0}}}
        >>> valid, msg = validate_action_params(ActionType.NAVIGATE_TO_POSE, params)
    """
    # 检查动作是否合法 / Check if action is valid
    if not ActionType.is_valid_action(action):
        return False, f"Invalid action: {action}"
    
    # 创建临时请求对象进行验证 / Create temporary request for validation
    try:
        temp_request = CommandRequest(action=action, params=params)
        return validate_request(temp_request)
    except Exception as e:
        return False, f"Failed to create request: {str(e)}"
