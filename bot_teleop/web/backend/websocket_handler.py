"""
WebSocket Connection Manager
Responsible for managing all active WebSocket connections, broadcasting task status updates

Architecture Design (after refactoring):
- Pure connection management and message broadcasting
- No active task status queries (passively receive updates from TaskManager)
- Completely asynchronous, non-blocking
- All polling logic removed
"""

from fastapi import WebSocket
from typing import List, Dict, Any, Optional
import json
import asyncio
from datetime import datetime


class WebSocketHandler:
    """WebSocket Connection Manager - Simplified version (only responsible for connection management and broadcasting)"""
    
    def __init__(self, event_loop: Optional[asyncio.AbstractEventLoop] = None):
        # Active connection list
        self.active_connections: List[WebSocket] = []
        
        # Connection information (for debugging)
        self.connection_info: Dict[WebSocket, Dict[str, Any]] = {}
        
        # Event loop reference (for scheduling async tasks across threads)
        self.event_loop = event_loop
        
        print("[WebSocket] ✓ WebSocketHandler initialization complete (passive mode)")
    
    async def connect(self, websocket: WebSocket):
        """Add new connection"""
        await websocket.accept()
        self.active_connections.append(websocket)
        
        # Record connection info
        self.connection_info[websocket] = {
            "client": websocket.client,
            "connected_at": datetime.now().isoformat()
        }
        
        print(f"[WebSocket] ✓ New connection: {websocket.client}, total: {len(self.active_connections)}")
    
    def disconnect(self, websocket: WebSocket):
        """Remove connection"""
        if websocket in self.active_connections:
            self.active_connections.remove(websocket)
        
        if websocket in self.connection_info:
            del self.connection_info[websocket]
        
        print(f"[WebSocket] ✗ Disconnected, total: {len(self.active_connections)}")
    
    async def broadcast(self, message: Dict[str, Any]):
        """Broadcast message to all connected clients"""
        if not self.active_connections:
            return
        
        # Convert to JSON string
        message_str = json.dumps(message, ensure_ascii=False)
        
        # Send to all clients concurrently
        disconnected = []
        for connection in self.active_connections:
            try:
                await connection.send_text(message_str)
            except Exception as e:
                print(f"[WebSocket] Send failed: {e}")
                disconnected.append(connection)
        
        # Clean up disconnected connections
        for connection in disconnected:
            self.disconnect(connection)
    
    async def broadcast_task_update(self, task: Dict[str, Any]):
        """
        Broadcast task status update (async version, for async context)
        
        Args:
            task: Result of Task.to_dict()
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
        print(f"[WebSocket] ✓ Task update broadcast: {task.get('request_id')} -> {task.get('status')}")
    
    def broadcast_task_update_sync(self, task: Dict[str, Any]):
        """
        Broadcast task status update (sync version, for ROS2 callback)
        
        Called from ROS2 thread, use asyncio.run_coroutine_threadsafe to schedule to main thread
        
        Args:
            task: Result of Task.to_dict()
        """
        if self.event_loop is None:
            print("[WebSocket] ✗ Event loop not set, cannot broadcast")
            return
        
        # Schedule async task in main thread's event loop
        asyncio.run_coroutine_threadsafe(
            self.broadcast_task_update(task),
            self.event_loop
        )
        print(f"[WebSocket] → Task update scheduled: {task.get('request_id')} -> {task.get('status')}")
    
    async def broadcast_status(self, status: Dict[str, Any]):
        """
        Broadcast robot status (async version)
        
        Args:
            status: Robot status dictionary (including type, position, velocity, etc.)
        """
        await self.broadcast(status)
    
    def broadcast_status_sync(self, status: Dict[str, Any]):
        """
        Broadcast robot status (sync version, for ROS2 callback)
        
        Called from ROS2 thread, use asyncio.run_coroutine_threadsafe to schedule to main thread
        
        Args:
            status: Robot status dictionary (including type, position, velocity, etc.)
        """
        if self.event_loop is None:
            return
        
        # Schedule async task in main thread's event loop
        asyncio.run_coroutine_threadsafe(
            self.broadcast_status(status),
            self.event_loop
        )
    
    def get_connection_count(self) -> int:
        """Get current connection count"""
        return len(self.active_connections)
    
    def get_connection_info(self) -> List[Dict[str, Any]]:
        """Get all connection information"""
        return [
            {
                "client": str(info.get("client")),
                "connected_at": info.get("connected_at")
            }
            for info in self.connection_info.values()
        ]
