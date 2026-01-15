"""
Waypoint Management API
Provides waypoint recording, saving, loading, and editing functionality
"""

from fastapi import APIRouter, HTTPException
from pydantic import BaseModel
from typing import List, Optional
import yaml
import os
from pathlib import Path

# ROS2 Integration - for waypoint recording
import rclpy
from rclpy.node import Node
from bot_navigation_msgs.srv import WaypointControl
import threading

router = APIRouter()

# Global ROS2 node for service calls
_ros_node: Optional[Node] = None
_ros_executor = None
_ros_thread = None


# ============================================
# ROS2 Initialization
# ============================================

def init_ros_node():
    """Initialize ROS2 node (for service calls)"""
    global _ros_node, _ros_executor, _ros_thread
    
    if _ros_node is not None:
        return  # Already initialized
    
    try:
        # Check if ROS2 is already initialized
        if not rclpy.ok():
            rclpy.init()
        
        # Create node
        _ros_node = Node('waypoint_web_api_node')
        
        # Run executor in separate thread
        from rclpy.executors import SingleThreadedExecutor
        _ros_executor = SingleThreadedExecutor()
        _ros_executor.add_node(_ros_node)
        
        _ros_thread = threading.Thread(target=_ros_executor.spin, daemon=True)
        _ros_thread.start()
        
        _ros_node.get_logger().info('Waypoint Web API ROS2 node initialized')
        
    except Exception as e:
        print(f"Warning: Failed to initialize ROS2 node for waypoint recording: {e}")


def shutdown_ros_node():
    """Shutdown ROS2 node"""
    global _ros_node, _ros_executor, _ros_thread
    
    if _ros_executor is not None:
        _ros_executor.shutdown()
    
    if _ros_node is not None:
        _ros_node.destroy_node()
        _ros_node = None
    
    if rclpy.ok():
        rclpy.shutdown()


# ============================================
# Data Models
# ============================================

class Waypoint(BaseModel):
    """Waypoint"""
    name: str
    x: float
    y: float
    yaw: float
    dwell_time: float = 2.0


class WaypointRoute(BaseModel):
    """Waypoint route"""
    name: str
    waypoints: List[Waypoint]
    description: Optional[str] = None


class WaypointListResponse(BaseModel):
    """Waypoint list response"""
    success: bool
    routes: List[dict]


class WaypointSaveRequest(BaseModel):
    """Waypoint save request"""
    route_name: str
    waypoints: List[Waypoint]
    description: Optional[str] = None


class RecordControlRequest(BaseModel):
    """Recording control request"""
    command: str  # 'start', 'mark', 'stop', 'save'
    route_name: Optional[str] = None
    waypoint_name: Optional[str] = None


# ============================================
# Utility Functions
# ============================================

def get_waypoints_directory() -> Path:
    """Get waypoints directory (read from web_config.yaml)"""
    try:
        # Load path from web_config.yaml
        from ..api.config import load_config
        config = load_config()
        waypoints_path = config.get('paths', {}).get('waypoints_dir', '')
        
        if waypoints_path:
            # Expand user directory symbol ~
            waypoints_dir = Path(os.path.expanduser(waypoints_path))
            print(f"[WaypointAPI] Using waypoints_dir from config: {waypoints_dir}")
        else:
            # Not specified in config, use default path
            waypoints_dir = Path.home() / "lododo_bot" / "waypoints"
            print(f"[WaypointAPI] Using default waypoints_dir: {waypoints_dir}")
        
        # Ensure directory exists
        if not waypoints_dir.exists():
            waypoints_dir.mkdir(parents=True, exist_ok=True)
            
        return waypoints_dir
    except Exception as e:
        # Use default path when config loading fails
        print(f"[WaypointAPI] Failed to load config, using default path. Error: {e}")
        waypoints_dir = Path.home() / "lododo_bot" / "waypoints"
        if not waypoints_dir.exists():
            waypoints_dir.mkdir(parents=True, exist_ok=True)
        return waypoints_dir
        return waypoints_dir


def load_waypoint_file(filepath: Path) -> dict:
    """Load waypoint file"""
    try:
        with open(filepath, 'r', encoding='utf-8') as f:
            data = yaml.safe_load(f)
            return data
    except Exception as e:
        raise ValueError(f"Failed to load waypoint file: {e}")


def save_waypoint_file(filepath: Path, route: WaypointRoute):
    """Save waypoint file"""
    try:
        # Build YAML data (maintain format consistent with waypoint_recorder)
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
# API Endpoints
# ============================================

@router.get("/waypoints", response_model=WaypointListResponse)
async def list_waypoint_routes():
    """
    Get list of all waypoint routes
    
    Scans waypoint directory and returns all available routes
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
    Get detailed information of specified waypoint route
    
    Args:
        route_name: Route name
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
    Save waypoint route
    
    Create or update waypoint route file
    """
    try:
        waypoints_dir = get_waypoints_directory()
        file_path = waypoints_dir / f"{req.route_name}.yaml"
        
        # Build route object
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
    Update waypoint route
    
    Modify existing waypoint route
    """
    try:
        waypoints_dir = get_waypoints_directory()
        file_path = waypoints_dir / f"{route_name}.yaml"
        
        if not file_path.exists():
            raise HTTPException(status_code=404, detail=f"Route '{route_name}' not found")
        
        # Build route object
        route = WaypointRoute(
            name=route_name,
            waypoints=req.waypoints,
            description=req.description
        )
        
        # Save to file
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
    Delete waypoint route
    
    Remove specified route file from waypoint directory
    """
    try:
        waypoints_dir = get_waypoints_directory()
        file_path = waypoints_dir / f"{route_name}.yaml"
        
        if not file_path.exists():
            raise HTTPException(status_code=404, detail=f"Route '{route_name}' not found")
        
        # Delete file
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
    Add waypoint to route
    
    Append new waypoint to end of existing route
    """
    try:
        waypoints_dir = get_waypoints_directory()
        file_path = waypoints_dir / f"{route_name}.yaml"
        
        if not file_path.exists():
            raise HTTPException(status_code=404, detail=f"Route '{route_name}' not found")
        
        # Load existing route
        data = load_waypoint_file(file_path)
        waypoints = data.get("waypoints", [])
        
        # Add new waypoint
        waypoints.append({
            "name": waypoint.name,
            "x": waypoint.x,
            "y": waypoint.y,
            "yaw": waypoint.yaw,
            "dwell_time": waypoint.dwell_time
        })
        
        data["waypoints"] = waypoints
        
        # Save back to file
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
# Waypoint Recording API
# ============================================

@router.post("/waypoints/record/start")
async def start_recording():
    """
    Start waypoint recording
    Call waypoint_recorder node's start command
    """
    global _ros_node
    
    # Ensure ROS2 node is initialized
    if _ros_node is None:
        init_ros_node()
    
    if _ros_node is None:
        raise HTTPException(status_code=503, detail="ROS2 node not available")
    
    try:
        # Create service client
        client = _ros_node.create_client(WaypointControl, '/waypoint_recorder/control')
        
        # Wait for service availability (max 3 seconds)
        if not client.wait_for_service(timeout_sec=3.0):
            raise HTTPException(
                status_code=503,
                detail="Waypoint recorder service not available. Is the waypoint_recorder node running?"
            )
        
        # Create request
        request = WaypointControl.Request()
        request.command = 'start'
        
        # Call service synchronously
        from rclpy.task import Future
        future = client.call_async(request)
        
        # Wait for response (max 5 seconds)
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
    Mark current position as waypoint
    
    Args:
        waypoint_name: Waypoint name (optional, auto-generated by backend)
    """
    global _ros_node
    
    if _ros_node is None:
        init_ros_node()
    
    if _ros_node is None:
        raise HTTPException(status_code=503, detail="ROS2 node not available")
    
    try:
        # Create service client
        client = _ros_node.create_client(WaypointControl, '/waypoint_recorder/control')
        
        if not client.wait_for_service(timeout_sec=3.0):
            raise HTTPException(status_code=503, detail="Waypoint recorder service not available")
        
        # Create request
        request = WaypointControl.Request()
        request.command = 'mark'
        if waypoint_name:
            request.filename = waypoint_name  # Use filename field to pass waypoint name
        
        # Call service
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
    Stop recording (without saving)
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
    Save recorded waypoints
    
    Args:
        route_name: Route name
        description: Route description (optional)
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
            # If description provided, update file metadata
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
    Get recording status
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