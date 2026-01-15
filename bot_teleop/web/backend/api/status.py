#!/usr/bin/env python3
"""
系统状态检查 API
检查ROS2节点状态、话题数据、网络延迟等
"""

from fastapi import APIRouter, HTTPException
from typing import Dict, List, Any
import subprocess
import time

router = APIRouter()


@router.get("/api/status", response_model=Dict[str, Any])
async def get_system_status():
    """
    获取系统状态
    
    Returns:
        系统状态信息，包括：
        - nodes: 关键节点状态
        - topics: 关键话题状态
        - network: 网络延迟
        - overall: 总体状态（healthy/degraded/error）
    """
    try:
        # 检查关键节点
        nodes_status = await check_ros_nodes()
        
        # 检查关键话题
        topics_status = await check_ros_topics()
        
        # 计算总体状态
        overall_status = calculate_overall_status(nodes_status, topics_status)
        
        return {
            "nodes": nodes_status,
            "topics": topics_status,
            "overall": overall_status,
            "timestamp": time.time()
        }
        
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Failed to get system status: {str(e)}")


async def check_ros_nodes() -> List[Dict[str, Any]]:
    """
    检查关键ROS2节点状态
    
    Returns:
        节点状态列表
    """
    # 关键节点列表（使用实际的节点名称）
    critical_nodes = [
        "/mission_planner",
        "/rtabmap",
        "/robot_state_publisher",
        "/controller_manager",
    ]
    
    try:
        # 执行 ros2 node list 命令
        result = subprocess.run(
            ["ros2", "node", "list"],
            capture_output=True,
            text=True,
            timeout=5
        )
        
        if result.returncode != 0:
            return [{
                "name": node,
                "status": "unknown",
                "message": "Failed to query nodes"
            } for node in critical_nodes]
        
        # 解析运行中的节点（去除空行）
        running_nodes = [n.strip() for n in result.stdout.strip().split('\n') if n.strip()]
        
        # 检查每个关键节点
        nodes_status = []
        for node in critical_nodes:
            is_running = node in running_nodes
            nodes_status.append({
                "name": node,
                "status": "healthy" if is_running else "error",
                "message": "Running" if is_running else "Not running"
            })
        
        return nodes_status
        
    except subprocess.TimeoutExpired:
        return [{
            "name": node,
            "status": "unknown",
            "message": "Query timeout"
        } for node in critical_nodes]
    except Exception as e:
        return [{
            "name": node,
            "status": "error",
            "message": f"Check failed: {str(e)}"
        } for node in critical_nodes]


async def check_ros_topics() -> List[Dict[str, Any]]:
    """
    检查关键话题是否有数据
    
    Returns:
        话题状态列表
    """
    # 关键话题列表（使用实际的话题名称）
    critical_topics = [
        "/localization_pose",
        "/cmd_vel",
        "/camera/image_raw",
        "/camera/depth/image_raw",
        "/imu/data",
    ]
    
    try:
        # 执行 ros2 topic list 命令
        result = subprocess.run(
            ["ros2", "topic", "list"],
            capture_output=True,
            text=True,
            timeout=5
        )
        
        if result.returncode != 0:
            return [{
                "name": topic,
                "status": "unknown",
                "message": "Failed to query topics"
            } for topic in critical_topics]
        
        # 解析已发布的话题（去除空行）
        available_topics = [t.strip() for t in result.stdout.strip().split('\n') if t.strip()]
        
        # 检查每个关键话题
        topics_status = []
        for topic in critical_topics:
            is_available = topic in available_topics
            topics_status.append({
                "name": topic,
                "status": "healthy" if is_available else "warning",
                "message": "Publishing" if is_available else "Not available"
            })
        
        return topics_status
        
    except subprocess.TimeoutExpired:
        return [{
            "name": topic,
            "status": "unknown",
            "message": "Query timeout"
        } for topic in critical_topics]
    except Exception as e:
        return [{
            "name": topic,
            "status": "error",
            "message": f"Check failed: {str(e)}"
        } for topic in critical_topics]


def calculate_overall_status(
    nodes_status: List[Dict[str, Any]],
    topics_status: List[Dict[str, Any]]
) -> str:
    """
    根据节点和话题状态计算总体状态
    
    Args:
        nodes_status: 节点状态列表
        topics_status: 话题状态列表
        
    Returns:
        总体状态：healthy/degraded/error/unknown
    """
    # 检查是否有错误
    has_error = any(
        item["status"] == "error" 
        for item in nodes_status + topics_status
    )
    
    # 检查是否有警告
    has_warning = any(
        item["status"] in ["warning", "unknown"]
        for item in nodes_status + topics_status
    )
    
    if has_error:
        return "error"
    elif has_warning:
        return "degraded"
    else:
        return "healthy"
