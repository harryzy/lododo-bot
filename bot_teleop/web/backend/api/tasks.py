"""
任务管理 API
提供导航、探索、巡逻等任务的创建和管理接口
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


class TaskStatusResponse(BaseModel):
    """任务状态响应"""
    success: bool
    request_id: str
    tasks: Optional[List[dict]] = None


class TaskCancelRequest(BaseModel):
    """任务取消请求"""
    task_id: str


# ============================================
# 依赖注入
# ============================================

def get_node():
    """获取 WebTerminalNode 实例"""
    from ..web_server import get_web_terminal_node
    return get_web_terminal_node()


# ============================================
# API 端点
# ============================================

@router.post("/tasks/navigate", response_model=TaskResponse)
async def create_navigation_task(req: NavigateRequest, node = Depends(get_node)):
    """
    创建导航任务
    
    发送导航命令到指定位姿
    """
    try:
        request_id = node.navigate_to_pose(x=req.x, y=req.y, yaw=req.yaw)
        
        return TaskResponse(
            success=True,
            message=f"Navigation command sent to ({req.x:.2f}, {req.y:.2f})",
            request_id=request_id
        )
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))


@router.post("/tasks/exploration", response_model=TaskResponse)
async def create_exploration_task(req: ExplorationRequest, node = Depends(get_node)):
    """
    创建探索任务
    
    启动自主探索建图
    """
    try:
        request_id = node.start_exploration(
            map_name=req.map_name,
            save_on_completion=req.save_on_completion
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
async def create_patrol_task(req: PatrolRequest, node = Depends(get_node)):
    """
    创建巡逻任务
    
    按照指定路点文件开始巡逻
    """
    try:
        request_id = node.start_patrol(
            waypoint_file=req.waypoint_file,
            mode=req.mode
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
async def list_tasks():
    """
    获取任务列表
    
    返回所有任务的概览（通过查询状态实现）
    """
    # 这个接口可以缓存最近收到的任务状态响应
    # 暂时返回占位数据
    return {
        "success": True,
        "message": "Please use /tasks/status to query task status",
        "tasks": []
    }
