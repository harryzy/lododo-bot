"""
任务管理 API
提供导航、探索、巡逻等任务的创建和管理接口

架构重构后：
- 使用 TaskManager 管理任务状态
- 所有任务创建通过 TaskManager.create_task()
- 查询接口直接从 TaskManager 获取缓存数据
"""

from fastapi import APIRouter, HTTPException, Depends
from pydantic import BaseModel
from typing import Optional, List

router = APIRouter()


# ============================================
# 数据模型
# ============================================

class NavigateRequest(BaseModel):
    """导航请求"""
    x: float
    y: float
    yaw: float = 0.0


class ExplorationRequest(BaseModel):
    """探索请求"""
    map_name: Optional[str] = None
    save_on_completion: bool = True


class PatrolRequest(BaseModel):
    """巡逻请求"""
    waypoint_file: str
    mode: str = "loop"  # loop, once, bounce


class TaskResponse(BaseModel):
    """任务响应"""
    success: bool
    message: str
    request_id: str
    task_id: Optional[str] = None
    task_data: Optional[dict] = None  # 任务完整数据（从缓存）


class TaskStatusResponse(BaseModel):
    """任务状态响应"""
    success: bool
    request_id: str
    tasks: Optional[List[dict]] = None


class TaskCancelRequest(BaseModel):
    """任务取消请求"""
    task_id: str


class TaskQueryRequest(BaseModel):
    """任务查询请求（用户手动查询）"""
    task_id: str


# ============================================
# 依赖注入
# ============================================

def get_node():
    """获取 WebTerminalNode 实例"""
    from ..web_server import get_web_terminal_node
    return get_web_terminal_node()


def get_task_manager():
    """获取 TaskManager 实例"""
    from ..web_server import get_task_manager
    return get_task_manager()


# ============================================
# API 端点
# ============================================

@router.post("/tasks/navigate", response_model=TaskResponse)
async def create_navigation_task(
    req: NavigateRequest, 
    node = Depends(get_node),
    task_manager = Depends(get_task_manager)
):
    """
    创建导航任务
    
    发送导航命令到指定位姿
    """
    try:
        # 发送导航命令
        request_id = node.navigate_to_pose(x=req.x, y=req.y, yaw=req.yaw)
        
        # 在 TaskManager 中创建任务记录
        task_manager.create_task(
            request_id=request_id,
            action="navigate_to_pose",
            params={"x": req.x, "y": req.y, "yaw": req.yaw}
        )
        
        return TaskResponse(
            success=True,
            message=f"Navigation command sent to ({req.x:.2f}, {req.y:.2f})",
            request_id=request_id
        )
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))


@router.post("/tasks/exploration", response_model=TaskResponse)
async def create_exploration_task(
    req: ExplorationRequest, 
    node = Depends(get_node),
    task_manager = Depends(get_task_manager)
):
    """
    创建探索任务
    
    启动自主探索建图
    """
    try:
        request_id = node.start_exploration(
            map_name=req.map_name,
            save_on_completion=req.save_on_completion
        )
        
        # 在 TaskManager 中创建任务记录
        task_manager.create_task(
            request_id=request_id,
            action="start_exploration",
            params={"map_name": req.map_name, "save_on_completion": req.save_on_completion}
        )
        
        map_info = f" (map: {req.map_name})" if req.map_name else ""
        return TaskResponse(
            success=True,
            message=f"Exploration command sent{map_info}",
            request_id=request_id
        )
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))


@router.post("/tasks/patrol", response_model=TaskResponse)
async def create_patrol_task(
    req: PatrolRequest, 
    node = Depends(get_node),
    task_manager = Depends(get_task_manager)
):
    """
    创建巡逻任务
    
    按照指定路点文件开始巡逻
    """
    try:
        request_id = node.start_patrol(
            waypoint_file=req.waypoint_file,
            mode=req.mode
        )
        
        # 在 TaskManager 中创建任务记录
        task_manager.create_task(
            request_id=request_id,
            action="start_patrol",
            params={"waypoint_file": req.waypoint_file, "mode": req.mode}
        )
        
        return TaskResponse(
            success=True,
            message=f"Patrol command sent (mode: {req.mode})",
            request_id=request_id
        )
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))


@router.get("/tasks/status", response_model=TaskStatusResponse)
async def get_task_status(task_id: Optional[str] = None, node = Depends(get_node)):
    """
    查询任务状态
    
    Args:
        task_id: 任务 ID（可选，为空时查询所有任务）
    """
    try:
        request_id = node.query_task_status(task_id=task_id)
        
        return TaskStatusResponse(
            success=True,
            request_id=request_id,
            tasks=None  # 实际状态通过 WebSocket 推送
        )
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))


@router.post("/tasks/cancel", response_model=TaskResponse)
async def cancel_task(req: TaskCancelRequest, node = Depends(get_node)):
    """
    取消任务
    
    取消指定的任务
    """
    try:
        request_id = node.cancel_task(task_id=req.task_id)
        
        return TaskResponse(
            success=True,
            message=f"Cancel command sent for task {req.task_id}",
            request_id=request_id,
            task_id=req.task_id
        )
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))


@router.post("/tasks/{task_id}/pause", response_model=TaskResponse)
async def pause_task(
    task_id: str, 
    node = Depends(get_node),
    task_manager = Depends(get_task_manager)
):
    """
    暂停任务
    
    1. 返回缓存中的任务数据
    2. 发送暂停命令
    3. 最新状态通过 WebSocket 推送
    """
    try:
        # 检查任务是否存在
        task = task_manager.get_task_by_task_id(task_id)
        if not task:
            raise HTTPException(status_code=404, detail=f"Task {task_id} not found")
        
        request_id = node.pause_task(task_id=task_id)
        
        return TaskResponse(
            success=True,
            message=f"Pause command sent for task {task_id}",
            request_id=request_id,
            task_id=str(task_id),
            task_data=task.to_dict()
        )
    except HTTPException:
        raise
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))


@router.post("/tasks/{task_id}/resume", response_model=TaskResponse)
async def resume_task(
    task_id: str,  # 修复：使用str类型以匹配字符串格式的task_id
    node = Depends(get_node),
    task_manager = Depends(get_task_manager)
):
    """
    恢复任务
    
    1. 返回缓存中的任务数据
    2. 发送恢复命令
    3. 最新状态通过 WebSocket 推送
    """
    try:
        # 检查任务是否存在
        task = task_manager.get_task_by_task_id(task_id)
        if not task:
            raise HTTPException(status_code=404, detail=f"Task {task_id} not found")
        
        request_id = node.resume_task(task_id=task_id)
        
        return TaskResponse(
            success=True,
            message=f"Resume command sent for task {task_id}",
            request_id=request_id,
            task_id=str(task_id),
            task_data=task.to_dict()
        )
    except HTTPException:
        raise
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))


@router.post("/tasks/emergency_stop", response_model=TaskResponse)
async def emergency_stop(node = Depends(get_node)):
    """
    紧急停止
    
    立即停止所有任务和机器人运动
    """
    try:
        request_id = node.emergency_stop()
        
        return TaskResponse(
            success=True,
            message="Emergency stop command sent",
            request_id=request_id
        )
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))


@router.get("/tasks/list")
async def list_tasks(task_manager = Depends(get_task_manager)):
    """
    获取任务列表（活跃任务 + 历史记录）
    
    返回所有缓存的任务，不发起 ROS2 查询
    """
    try:
        active_tasks = task_manager.get_active_tasks()
        history_tasks = task_manager.get_task_history(limit=20)
        
        return {
            "success": True,
            "message": "Task list retrieved from cache",
            "active_tasks": active_tasks,
            "history_tasks": history_tasks,
            "total_active": len(active_tasks),
            "total_history": len(history_tasks)
        }
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))


@router.get("/tasks/active")
async def get_active_tasks(task_manager = Depends(get_task_manager)):
    """
    获取所有活跃任务（从缓存）
    
    直接返回 TaskManager 缓存的活跃任务，不查询 ROS2
    """
    try:
        active_tasks = task_manager.get_active_tasks()
        
        return {
            "success": True,
            "message": "Active tasks retrieved from cache",
            "tasks": active_tasks,
            "count": len(active_tasks)
        }
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))


@router.get("/tasks/history")
async def get_task_history(
    limit: int = 50,
    task_manager = Depends(get_task_manager)
):
    """
    获取任务历史记录
    
    从 TaskManager 获取已完成/失败/取消的任务历史
    """
    try:
        history_tasks = task_manager.get_task_history(limit=limit)
        
        return {
            "success": True,
            "message": "Task history retrieved",
            "tasks": history_tasks,
            "count": len(history_tasks)
        }
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))


@router.post("/tasks/{task_id}/query", response_model=TaskResponse)
async def query_single_task(
    task_id: str, 
    node = Depends(get_node),
    task_manager = Depends(get_task_manager)
):
    """
    手动查询单个任务状态（用户点击按钮触发）
    
    1. 立即返回缓存中的任务数据（快速响应）
    2. 发送 ROS2 查询请求到 MissionPlanner（后台更新）
    3. 最新状态通过 WebSocket 推送到前端
    """
    try:
        # 先从缓存获取当前任务数据（包括历史任务）
        task = task_manager.get_task_by_task_id(task_id)
        if not task:
            raise HTTPException(status_code=404, detail=f"Task {task_id} not found in cache")
        
        # 如果任务已完成/失败/取消，不发起ROS查询（避免500错误）
        if task.status in ['completed', 'failed', 'cancelled']:
            return TaskResponse(
                success=True,
                message=f"Task {task_id} already finished (status: {task.status})",
                request_id="",  # 未发起新查询
                task_id=task_id,
                task_data=task.to_dict()
            )
        
        # 任务仍在执行中，发起 ROS2 查询请求
        request_id = node.query_task_status(task_id=task_id)
        
        # 立即返回缓存数据 + 查询请求ID
        return TaskResponse(
            success=True,
            message=f"Query sent for task {task_id}",
            request_id=request_id,
            task_id=task_id,
            task_data=task.to_dict()
        )
    except HTTPException:
        raise
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))
