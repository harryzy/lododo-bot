"""
路点管理 API
提供路点录制、保存、加载、编辑等功能
"""

from fastapi import APIRouter, HTTPException
from pydantic import BaseModel
from typing import List, Optional
import yaml
import os
from pathlib import Path

# ROS2 集成 - 用于路点录制
import rclpy
from rclpy.node import Node
from bot_navigation_msgs.srv import WaypointControl
import threading

router = APIRouter()

# 全局 ROS2 节点用于服务调用
_ros_node: Optional[Node] = None
_ros_executor = None
_ros_thread = None


# ============================================
# ROS2 初始化
# ============================================

def init_ros_node():
    """初始化 ROS2 节点（用于服务调用）"""
    global _ros_node, _ros_executor, _ros_thread
    
    if _ros_node is not None:
        return  # 已初始化
    
    try:
        # 检查 ROS2 是否已初始化
        if not rclpy.ok():
            rclpy.init()
        
        # 创建节点
        _ros_node = Node('waypoint_web_api_node')
        
        # 在单独线程中运行 executor
        from rclpy.executors import SingleThreadedExecutor
        _ros_executor = SingleThreadedExecutor()
        _ros_executor.add_node(_ros_node)
        
        _ros_thread = threading.Thread(target=_ros_executor.spin, daemon=True)
        _ros_thread.start()
        
        _ros_node.get_logger().info('Waypoint Web API ROS2 node initialized')
        
    except Exception as e:
        print(f"Warning: Failed to initialize ROS2 node for waypoint recording: {e}")


def shutdown_ros_node():
    """关闭 ROS2 节点"""
    global _ros_node, _ros_executor, _ros_thread
    
    if _ros_executor is not None:
        _ros_executor.shutdown()
    
    if _ros_node is not None:
        _ros_node.destroy_node()
        _ros_node = None
    
    if rclpy.ok():
        rclpy.shutdown()


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


class RecordControlRequest(BaseModel):
    """录制控制请求"""
    command: str  # 'start', 'mark', 'stop', 'save'
    route_name: Optional[str] = None
    waypoint_name: Optional[str] = None


# ============================================
# 工具函数
# ============================================

def get_waypoints_directory() -> Path:
    """获取路点目录（从web_config.yaml读取）"""
    try:
        # 从web_config.yaml加载路径
        from ..api.config import load_config
        config = load_config()
        waypoints_path = config.get('paths', {}).get('waypoints_dir', '')
        
        if waypoints_path:
            # 展开用户目录符号 ~
            waypoints_dir = Path(os.path.expanduser(waypoints_path))
            print(f"[WaypointAPI] Using waypoints_dir from config: {waypoints_dir}")
        else:
            # 配置中未指定，使用默认路径
            waypoints_dir = Path.home() / "lododo_bot" / "waypoints"
            print(f"[WaypointAPI] Using default waypoints_dir: {waypoints_dir}")
        
        # 确保目录存在
        if not waypoints_dir.exists():
            waypoints_dir.mkdir(parents=True, exist_ok=True)
            
        return waypoints_dir
    except Exception as e:
        # 配置加载失败时使用默认路径
        print(f"[WaypointAPI] Failed to load config, using default path. Error: {e}")
        waypoints_dir = Path.home() / "lododo_bot" / "waypoints"
        if not waypoints_dir.exists():
            waypoints_dir.mkdir(parents=True, exist_ok=True)
        return waypoints_dir
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

# ============================================
# 路点录制 API
# ============================================

@router.post("/waypoints/record/start")
async def start_recording():
    """
    开始路点录制
    调用 waypoint_recorder 节点的 start 命令
    """
    global _ros_node
    
    # 确保 ROS2 节点已初始化
    if _ros_node is None:
        init_ros_node()
    
    if _ros_node is None:
        raise HTTPException(status_code=503, detail="ROS2 node not available")
    
    try:
        # 创建服务客户端
        client = _ros_node.create_client(WaypointControl, '/waypoint_recorder/control')
        
        # 等待服务可用（最多3秒）
        if not client.wait_for_service(timeout_sec=3.0):
            raise HTTPException(
                status_code=503,
                detail="Waypoint recorder service not available. Is the waypoint_recorder node running?"
            )
        
        # 创建请求
        request = WaypointControl.Request()
        request.command = 'start'
        
        # 同步调用服务
        from rclpy.task import Future
        future = client.call_async(request)
        
        # 等待响应（最多5秒）
        import time
        timeout = 5.0
        start_time = time.time()
        while not future.done():
            if time.time() - start_time > timeout:
                raise HTTPException(status_code=504, detail="Service call timeout")
            time.sleep(0.1)
        
        response = future.result()
        
        if response.success:
            return {
                "success": True,
                "message": response.message
            }
        else:
            raise HTTPException(status_code=400, detail=response.message)
            
    except HTTPException:
        raise
    except Exception as e:
        _ros_node.get_logger().error(f"Start recording failed: {e}")
        raise HTTPException(status_code=500, detail=str(e))


@router.post("/waypoints/record/mark")
async def mark_waypoint(waypoint_name: Optional[str] = None):
    """
    标记当前位置为路点
    
    Args:
        waypoint_name: 路点名称（可选，由后端自动生成）
    """
    global _ros_node
    
    if _ros_node is None:
        init_ros_node()
    
    if _ros_node is None:
        raise HTTPException(status_code=503, detail="ROS2 node not available")
    
    try:
        # 创建服务客户端
        client = _ros_node.create_client(WaypointControl, '/waypoint_recorder/control')
        
        if not client.wait_for_service(timeout_sec=3.0):
            raise HTTPException(status_code=503, detail="Waypoint recorder service not available")
        
        # 创建请求
        request = WaypointControl.Request()
        request.command = 'mark'
        if waypoint_name:
            request.filename = waypoint_name  # 使用 filename 字段传递路点名称
        
        # 调用服务
        future = client.call_async(request)
        
        import time
        timeout = 5.0
        start_time = time.time()
        while not future.done():
            if time.time() - start_time > timeout:
                raise HTTPException(status_code=504, detail="Service call timeout")
            time.sleep(0.1)
        
        response = future.result()
        
        if response.success:
            return {
                "success": True,
                "message": response.message
            }
        else:
            raise HTTPException(status_code=400, detail=response.message)
            
    except HTTPException:
        raise
    except Exception as e:
        _ros_node.get_logger().error(f"Mark waypoint failed: {e}")
        raise HTTPException(status_code=500, detail=str(e))


@router.post("/waypoints/record/stop")
async def stop_recording():
    """
    停止录制（不保存）
    """
    global _ros_node
    
    if _ros_node is None:
        raise HTTPException(status_code=503, detail="ROS2 node not available")
    
    try:
        client = _ros_node.create_client(WaypointControl, '/waypoint_recorder/control')
        
        if not client.wait_for_service(timeout_sec=3.0):
            raise HTTPException(status_code=503, detail="Waypoint recorder service not available")
        
        request = WaypointControl.Request()
        request.command = 'stop'
        
        future = client.call_async(request)
        
        import time
        timeout = 5.0
        start_time = time.time()
        while not future.done():
            if time.time() - start_time > timeout:
                raise HTTPException(status_code=504, detail="Service call timeout")
            time.sleep(0.1)
        
        response = future.result()
        
        if response.success:
            return {
                "success": True,
                "message": response.message
            }
        else:
            raise HTTPException(status_code=400, detail=response.message)
            
    except HTTPException:
        raise
    except Exception as e:
        _ros_node.get_logger().error(f"Stop recording failed: {e}")
        raise HTTPException(status_code=500, detail=str(e))


@router.post("/waypoints/record/save")
async def save_recording(route_name: str, description: Optional[str] = None):
    """
    保存录制的路点
    
    Args:
        route_name: 路线名称
        description: 路线描述（可选）
    """
    global _ros_node
    
    if _ros_node is None:
        raise HTTPException(status_code=503, detail="ROS2 node not available")
    
    try:
        client = _ros_node.create_client(WaypointControl, '/waypoint_recorder/control')
        
        if not client.wait_for_service(timeout_sec=3.0):
            raise HTTPException(status_code=503, detail="Waypoint recorder service not available")
        
        request = WaypointControl.Request()
        request.command = 'save'
        request.filename = route_name
        
        future = client.call_async(request)
        
        import time
        timeout = 5.0
        start_time = time.time()
        while not future.done():
            if time.time() - start_time > timeout:
                raise HTTPException(status_code=504, detail="Service call timeout")
            time.sleep(0.1)
        
        response = future.result()
        
        if response.success:
            # 如果提供了描述，更新文件元数据
            if description:
                try:
                    waypoints_dir = get_waypoints_directory()
                    file_path = waypoints_dir / f"{route_name}.yaml"
                    
                    if file_path.exists():
                        data = load_waypoint_file(file_path)
                        data["description"] = description
                        
                        with open(file_path, 'w', encoding='utf-8') as f:
                            yaml.dump(data, f, allow_unicode=True, sort_keys=False)
                except Exception as e:
                    _ros_node.get_logger().warning(f"Failed to add description: {e}")
            
            return {
                "success": True,
                "message": response.message,
                "route_name": route_name
            }
        else:
            raise HTTPException(status_code=400, detail=response.message)
            
    except HTTPException:
        raise
    except Exception as e:
        _ros_node.get_logger().error(f"Save recording failed: {e}")
        raise HTTPException(status_code=500, detail=str(e))


@router.get("/waypoints/record/status")
async def get_recording_status():
    """
    获取录制状态
    """
    global _ros_node
    
    if _ros_node is None:
        return {
            "recording": False,
            "waypoint_count": 0,
            "message": "ROS2 node not initialized"
        }
    
    try:
        client = _ros_node.create_client(WaypointControl, '/waypoint_recorder/control')
        
        if not client.wait_for_service(timeout_sec=1.0):
            return {
                "recording": False,
                "waypoint_count": 0,
                "message": "Waypoint recorder service not available"
            }
        
        request = WaypointControl.Request()
        request.command = 'status'
        
        future = client.call_async(request)
        
        import time
        timeout = 3.0
        start_time = time.time()
        while not future.done():
            if time.time() - start_time > timeout:
                return {
                    "recording": False,
                    "waypoint_count": 0,
                    "message": "Service call timeout"
                }
            time.sleep(0.1)
        
        response = future.result()
        
        return {
            "success": response.success,
            "message": response.message,
            "recording": "recording" in response.message.lower()
        }
        
    except Exception as e:
        _ros_node.get_logger().error(f"Get recording status failed: {e}")
        return {
            "recording": False,
            "waypoint_count": 0,
            "message": str(e)
        }