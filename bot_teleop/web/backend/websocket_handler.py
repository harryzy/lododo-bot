"""
WebSocket 连接管理器
负责管理所有活跃的 WebSocket 连接，广播消息
"""

from fastapi import WebSocket
from typing import List, Dict, Any
import asyncio
import json
from datetime import datetime


class WebSocketHandler:
    """WebSocket 连接管理器"""
    
    def __init__(self):
        # 活跃连接列表
        self.active_connections: List[WebSocket] = []
        
        # 连接信息（用于调试）
        self.connection_info: Dict[WebSocket, Dict[str, Any]] = {}
    
    async def connect(self, websocket: WebSocket):
        """接受新连接"""
        await websocket.accept()
        self.active_connections.append(websocket)
        
        # 记录连接信息
        self.connection_info[websocket] = {
            "connected_at": datetime.now().isoformat(),
            "client": websocket.client
        }
        
        print(f"[WebSocket] 新连接: {websocket.client}, 当前连接数: {len(self.active_connections)}")
        
        # 发送欢迎消息
        await self.send_personal_message(
            {
                "type": "connection",
                "status": "connected",
                "timestamp": datetime.now().isoformat(),
                "message": "Welcome to LeKiwi Robot Web Interface"
            },
            websocket
        )
    
    def disconnect(self, websocket: WebSocket):
        """断开连接"""
        if websocket in self.active_connections:
            self.active_connections.remove(websocket)
        
        if websocket in self.connection_info:
            del self.connection_info[websocket]
        
        print(f"[WebSocket] 连接断开，剩余连接数: {len(self.active_connections)}")
    
    async def send_personal_message(self, message: Dict[str, Any], websocket: WebSocket):
        """发送消息给特定客户端"""
        try:
            if isinstance(message, dict):
                message_str = json.dumps(message, ensure_ascii=False)
            else:
                message_str = str(message)
            
            await websocket.send_text(message_str)
        except Exception as e:
            print(f"[WebSocket] 发送消息失败: {e}")
            self.disconnect(websocket)
    
    async def broadcast(self, message: Dict[str, Any]):
        """广播消息给所有连接的客户端"""
        if not self.active_connections:
            return
        
        print(f"[WebSocket] 广播消息给 {len(self.active_connections)} 个客户端")
        
        # 转换为 JSON 字符串
        if isinstance(message, dict):
            message_str = json.dumps(message, ensure_ascii=False)
        else:
            message_str = str(message)
        
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
    
    async def broadcast_response(self, response):
        """广播 CommandResponse（来自 ROS2 节点的回调）"""
        try:
            # response 是 CommandResponse 对象
            message = {
                "type": "command_response",
                "timestamp": datetime.now().isoformat(),
                "request_id": response.request_id,
                "status": response.status,
                "code": response.code,
                "message": response.message,
                "result": response.result,
                "progress": response.progress if hasattr(response, 'progress') else None
            }
            
            await self.broadcast(message)
            
        except Exception as e:
            print(f"[WebSocket] 广播响应失败: {e}")
    
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
