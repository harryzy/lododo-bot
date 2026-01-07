"""
命令动作类型常量定义 / Command action type constants

定义了所有支持的命令动作类型，用于统一命令接口 /
Defines all supported command action types for unified command interface
"""


class ActionType:
    """
    命令动作类型常量 / Command action type constants
    
    包含导航、任务管理、查询、控制、地图等类型的动作 /
    Contains actions for navigation, task management, query, control, and map operations
    """
    
    # ========== 导航类 / Navigation ==========
    NAVIGATE_TO_POSE = "navigate_to_pose"
    """导航到目标点 (x, y, yaw) / Navigate to target pose (x, y, yaw)"""
    
    NAVIGATE_TO_LOCATION = "navigate_to_location"
    """导航到命名地点 / Navigate to named location"""
    
    # ========== 任务类 / Task ==========
    START_EXPLORATION = "start_exploration"
    """开始自主探索建图 / Start autonomous exploration and mapping"""
    
    START_PATROL = "start_patrol"
    """开始巡航任务 / Start patrol task"""
    
    STOP_PATROL = "stop_patrol"
    """停止巡航任务 / Stop patrol task"""
    
    PAUSE_TASK = "pause_task"
    """暂停当前任务 / Pause current task"""
    
    RESUME_TASK = "resume_task"
    """恢复暂停的任务 / Resume paused task"""
    
    CANCEL_TASK = "cancel_task"
    """取消任务 / Cancel task"""
    
    # ========== 查询类 / Query ==========
    GET_TASK_STATUS = "get_task_status"
    """查询任务状态 / Get task status"""
    
    GET_ROBOT_STATUS = "get_robot_status"
    """查询机器人状态 / Get robot status"""
    
    # ========== 控制类 / Control ==========
    EMERGENCY_STOP = "emergency_stop"
    """紧急停止 / Emergency stop"""
    
    # ========== 地图类 / Map ==========
    SAVE_MAP = "save_map"
    """保存地图 / Save map"""
    
    LOAD_MAP = "load_map"
    """加载地图 / Load map"""
    
    # ========== 所有动作列表 / All actions list ==========
    ALL_ACTIONS = [
        NAVIGATE_TO_POSE,
        NAVIGATE_TO_LOCATION,
        START_EXPLORATION,
        START_PATROL,
        STOP_PATROL,
        PAUSE_TASK,
        RESUME_TASK,
        CANCEL_TASK,
        GET_TASK_STATUS,
        GET_ROBOT_STATUS,
        EMERGENCY_STOP,
        SAVE_MAP,
        LOAD_MAP,
    ]
    
    @staticmethod
    def is_valid_action(action: str) -> bool:
        """
        验证动作类型是否合法 / Validate if action type is valid
        
        Args:
            action: 动作类型字符串 / Action type string
            
        Returns:
            bool: 是否为合法动作类型 / Whether it's a valid action type
        """
        return action in ActionType.ALL_ACTIONS
    
    @staticmethod
    def get_category(action: str) -> str:
        """
        获取动作类型所属分类 / Get category of action type
        
        Args:
            action: 动作类型字符串 / Action type string
            
        Returns:
            str: 分类名称 (navigation/task/query/control/map) /
                 Category name (navigation/task/query/control/map)
        """
        if action in [ActionType.NAVIGATE_TO_POSE, ActionType.NAVIGATE_TO_LOCATION]:
            return "navigation"
        elif action in [
            ActionType.START_EXPLORATION,
            ActionType.START_PATROL,
            ActionType.STOP_PATROL,
            ActionType.PAUSE_TASK,
            ActionType.RESUME_TASK,
            ActionType.CANCEL_TASK,
        ]:
            return "task"
        elif action in [ActionType.GET_TASK_STATUS, ActionType.GET_ROBOT_STATUS]:
            return "query"
        elif action == ActionType.EMERGENCY_STOP:
            return "control"
        elif action in [ActionType.SAVE_MAP, ActionType.LOAD_MAP]:
            return "map"
        else:
            return "unknown"
