"""
路点管理 API
提供路点录制、保存、加载、编辑等功能
"""

from fastapi import APIRouter, HTTPException
from pydantic import BaseModel
from typing import List, Optional
import yaml
from pathlib import Path

router = APIRouter()


# ============================================
# 数据模型
# ============================================

class Waypoint(BaseModel):
    """路点"""
    name: str
    x: float
    y: float
    yaw: float
    dwell_time: float = 2.0


class WaypointRoute(BaseModel):
    """路点路线"""
    name: str
    waypoints: List[Waypoint]
    description: Optional[str] = None


class WaypointListResponse(BaseModel):
    """路点列表响应"""
    success: bool
    routes: List[dict]


class WaypointSaveRequest(BaseModel):
    """路点保存请求"""
    route_name: str
    waypoints: List[Waypoint]
    description: Optional[str] = None


# ============================================
# 工具函数
# ============================================

def get_waypoints_directory() -> Path:
    """获取路点目录"""
    waypoints_dir = Path.home() / "lododo_bot" / "waypoints"
    if not waypoints_dir.exists():
        waypoints_dir.mkdir(parents=True, exist_ok=True)
    return waypoints_dir


def load_waypoint_file(filepath: Path) -> dict:
    """加载路点文件"""
    try:
        with open(filepath, 'r', encoding='utf-8') as f:
            data = yaml.safe_load(f)
            return data
    except Exception as e:
        raise ValueError(f"Failed to load waypoint file: {e}")


def save_waypoint_file(filepath: Path, route: WaypointRoute):
    """保存路点文件"""
    try:
        # 构造 YAML 数据（保持与 waypoint_recorder 一致的格式）
        data = {
            "waypoints": [
                {
                    "name": wp.name,
                    "x": wp.x,
                    "y": wp.y,
                    "yaw": wp.yaw,
                    "dwell_time": wp.dwell_time
                }
                for wp in route.waypoints
            ]
        }
        
        if route.description:
            data["description"] = route.description
        
        with open(filepath, 'w', encoding='utf-8') as f:
            yaml.dump(data, f, allow_unicode=True, sort_keys=False)
            
    except Exception as e:
        raise ValueError(f"Failed to save waypoint file: {e}")


# ============================================
# API 端点
# ============================================

@router.get("/waypoints", response_model=WaypointListResponse)
async def list_waypoint_routes():
    """
    获取所有路点路线列表
    
    扫描路点目录，返回所有可用路线
    """
    try:
        waypoints_dir = get_waypoints_directory()
        routes = []
        
        for file_path in waypoints_dir.glob("*.yaml"):
            try:
                data = load_waypoint_file(file_path)
                routes.append({
                    "name": file_path.stem,
                    "path": str(file_path),
                    "waypoint_count": len(data.get("waypoints", [])),
                    "description": data.get("description")
                })
            except Exception:
                continue
        
        return WaypointListResponse(
            success=True,
            routes=routes
        )
        
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))


@router.get("/waypoints/{route_name}")
async def get_waypoint_route(route_name: str):
    """
    获取指定路点路线的详细信息
    
    Args:
        route_name: 路线名称
    """
    try:
        waypoints_dir = get_waypoints_directory()
        file_path = waypoints_dir / f"{route_name}.yaml"
        
        if not file_path.exists():
            raise HTTPException(status_code=404, detail=f"Route '{route_name}' not found")
        
        data = load_waypoint_file(file_path)
        
        return {
            "success": True,
            "name": route_name,
            "path": str(file_path),
            "waypoints": data.get("waypoints", []),
            "description": data.get("description")
        }
        
    except HTTPException:
        raise
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))


@router.post("/waypoints")
async def save_waypoint_route(req: WaypointSaveRequest):
    """
    保存路点路线
    
    创建或更新路点路线文件
    """
    try:
        waypoints_dir = get_waypoints_directory()
        file_path = waypoints_dir / f"{req.route_name}.yaml"
        
        # 构造路线对象
        route = WaypointRoute(
            name=req.route_name,
            waypoints=req.waypoints,
            description=req.description
        )
        
        # 保存到文件
        save_waypoint_file(file_path, route)
        
        return {
            "success": True,
            "message": f"Route '{req.route_name}' saved successfully",
            "path": str(file_path),
            "waypoint_count": len(req.waypoints)
        }
        
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))


@router.put("/waypoints/{route_name}")
async def update_waypoint_route(route_name: str, req: WaypointSaveRequest):
    """
    更新路点路线
    
    修改已存在的路点路线
    """
    try:
        waypoints_dir = get_waypoints_directory()
        file_path = waypoints_dir / f"{route_name}.yaml"
        
        if not file_path.exists():
            raise HTTPException(status_code=404, detail=f"Route '{route_name}' not found")
        
        # 构造路线对象
        route = WaypointRoute(
            name=route_name,
            waypoints=req.waypoints,
            description=req.description
        )
        
        # 保存到文件
        save_waypoint_file(file_path, route)
        
        return {
            "success": True,
            "message": f"Route '{route_name}' updated successfully",
            "waypoint_count": len(req.waypoints)
        }
        
    except HTTPException:
        raise
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))


@router.delete("/waypoints/{route_name}")
async def delete_waypoint_route(route_name: str):
    """
    删除路点路线
    
    从路点目录删除指定路线文件
    """
    try:
        waypoints_dir = get_waypoints_directory()
        file_path = waypoints_dir / f"{route_name}.yaml"
        
        if not file_path.exists():
            raise HTTPException(status_code=404, detail=f"Route '{route_name}' not found")
        
        # 删除文件
        file_path.unlink()
        
        return {
            "success": True,
            "message": f"Route '{route_name}' deleted successfully"
        }
        
    except HTTPException:
        raise
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))


@router.post("/waypoints/{route_name}/add")
async def add_waypoint_to_route(route_name: str, waypoint: Waypoint):
    """
    向路线添加路点
    
    在现有路线末尾添加新路点
    """
    try:
        waypoints_dir = get_waypoints_directory()
        file_path = waypoints_dir / f"{route_name}.yaml"
        
        if not file_path.exists():
            raise HTTPException(status_code=404, detail=f"Route '{route_name}' not found")
        
        # 加载现有路线
        data = load_waypoint_file(file_path)
        waypoints = data.get("waypoints", [])
        
        # 添加新路点
        waypoints.append({
            "name": waypoint.name,
            "x": waypoint.x,
            "y": waypoint.y,
            "yaw": waypoint.yaw,
            "dwell_time": waypoint.dwell_time
        })
        
        data["waypoints"] = waypoints
        
        # 保存回文件
        with open(file_path, 'w', encoding='utf-8') as f:
            yaml.dump(data, f, allow_unicode=True, sort_keys=False)
        
        return {
            "success": True,
            "message": f"Waypoint '{waypoint.name}' added to route '{route_name}'",
            "waypoint_count": len(waypoints)
        }
        
    except HTTPException:
        raise
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))
