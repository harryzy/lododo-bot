"""
WebSocket 连接管理器
负责管理所有活跃的 WebSocket 连接，广播任务状态更新

架构设计（重构后）：
- 纯粹的连接管理和消息广播
- 不主动查询任务状态（被动接收 TaskManager 的更新）
- 完全异步，非阻塞
- 移除所有轮询逻辑
"""

from fastapi import WebSocket
from typing import List, Dict, Any, Optional
import json
import asyncio
from datetime import datetime


class WebSocketHandler:
    """WebSocket 连接管理器 - 简化版（仅负责连接管理和广播）"""
    
    def __init__(self, event_loop: Optional[asyncio.AbstractEventLoop] = None):
        # 活跃连接列表
        self.active_connections: List[WebSocket] = []
        
        # 连接信息（用于调试）
        self.connection_info: Dict[WebSocket, Dict[str, Any]] = {}
        
        # 事件循环引用（用于跨线程调度异步任务）
        self.event_loop = event_loop
        
        print("[WebSocket] ✓ WebSocketHandler 初始化完成（被动模式）")
    
    async def connect(self, websocket: WebSocket):
        """添加新连接"""
        await websocket.accept()
        self.active_connections.append(websocket)
        
        # 记录连接信息
        self.connection_info[websocket] = {
            "client": websocket.client,
            "connected_at": datetime.now().isoformat()
        }
        
        print(f"[WebSocket] ✓ 新连接: {websocket.client}, 当前连接数: {len(self.active_connections)}")
    
    def disconnect(self, websocket: WebSocket):
        """移除连接"""
        if websocket in self.active_connections:
            self.active_connections.remove(websocket)
        
        if websocket in self.connection_info:
            del self.connection_info[websocket]
        
        print(f"[WebSocket] ✗ 断开连接, 当前连接数: {len(self.active_connections)}")
    
    async def broadcast(self, message: Dict[str, Any]):
        """广播消息给所有连接的客户端"""
        if not self.active_connections:
            return
        
        # 转换为 JSON 字符串
        message_str = json.dumps(message, ensure_ascii=False)
        
        # 并发发送给所有客户端
        disconnected = []
        for connection in self.active_connections:
            try:
                await connection.send_text(message_str)
            except Exception as e:
                print(f"[WebSocket] 发送失败: {e}")
                disconnected.append(connection)
        
        # 清理断开的连接
        for connection in disconnected:
            self.disconnect(connection)
    
    async def broadcast_task_update(self, task: Dict[str, Any]):
        """
        广播任务状态更新（异步版本，用于async上下文）
        
        Args:
            task: Task.to_dict() 的结果
        """
        message = {
            "type": "task_update",
            "timestamp": datetime.now().isoformat(),
            "task": {
                "task_id": task.get('task_id'),
                "request_id": task.get('request_id'),
                "action": task.get('action'),
                "status": task.get('status'),
                "progress": task.get('progress', 0),
                "message": task.get('message', ''),
                "updated_at": task.get('updated_at')
            }
        }
        
        await self.broadcast(message)
        print(f"[WebSocket] ✓ 任务更新已广播: {task.get('request_id')} -> {task.get('status')}")
    
    def broadcast_task_update_sync(self, task: Dict[str, Any]):
        """
        广播任务状态更新（同步版本，用于ROS2回调）
        
        从ROS2线程调用，使用asyncio.run_coroutine_threadsafe调度到主线程
        
        Args:
            task: Task.to_dict() 的结果
        """
        if self.event_loop is None:
            print("[WebSocket] ✗ 事件循环未设置，无法广播")
            return
        
        # 在主线程的事件循环中调度异步任务
        asyncio.run_coroutine_threadsafe(
            self.broadcast_task_update(task),
            self.event_loop
        )
        print(f"[WebSocket] → 任务更新已调度: {task.get('request_id')} -> {task.get('status')}")
    
    async def broadcast_status(self, status: Dict[str, Any]):
        """
        广播机器人状态（异步版本）
        
        Args:
            status: 机器人状态字典（包含type, position, velocity等）
        """
        await self.broadcast(status)
    
    def broadcast_status_sync(self, status: Dict[str, Any]):
        """
        广播机器人状态（同步版本，用于ROS2回调）
        
        从ROS2线程调用，使用asyncio.run_coroutine_threadsafe调度到主线程
        
        Args:
            status: 机器人状态字典（包含type, position, velocity等）
        """
        if self.event_loop is None:
            return
        
        # 在主线程的事件循环中调度异步任务
        asyncio.run_coroutine_threadsafe(
            self.broadcast_status(status),
            self.event_loop
        )
    
    def get_connection_count(self) -> int:
        """获取当前连接数"""
        return len(self.active_connections)
    
    def get_connection_info(self) -> List[Dict[str, Any]]:
        """获取所有连接信息"""
        return [
            {
                "client": str(info.get("client")),
                "connected_at": info.get("connected_at")
            }
            for info in self.connection_info.values()
        ]
