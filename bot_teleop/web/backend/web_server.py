#!/usr/bin/env python3
"""
Web Control Interface - FastAPI Main Application
Provides REST API and WebSocket services

After architecture refactoring:
- Use TaskManager to manage task states (singleton pattern)
- WebTerminalNode passively listens to /cmd/response (persistent listener)
- WebSocketHandler only responsible for broadcasting (polling removed)
"""

from fastapi import FastAPI, WebSocket, WebSocketDisconnect, Depends, HTTPException
from fastapi.middleware.cors import CORSMiddleware
from fastapi.responses import JSONResponse
from fastapi.staticfiles import StaticFiles
from contextlib import asynccontextmanager
import asyncio
import yaml
import os
import rclpy
from pathlib import Path
from typing import Optional

from .nodes.web_terminal_node import WebTerminalNode
from .nodes.status_monitor_node import StatusMonitorNode
from .websocket_handler import WebSocketHandler
from .managers.task_manager import TaskManager
from .api import tasks, maps, waypoints, settings, status, config

# Global variables
web_terminal_node: Optional[WebTerminalNode] = None
status_monitor_node: Optional[StatusMonitorNode] = None
websocket_handler: Optional[WebSocketHandler] = None
task_manager: Optional[TaskManager] = None
ros_executor: Optional[asyncio.Future] = None
status_executor: Optional[asyncio.Future] = None


@asynccontextmanager
async def lifespan(app: FastAPI):
    """Application lifecycle management"""
    global web_terminal_node, status_monitor_node, websocket_handler, task_manager, ros_executor, status_executor
    
    print("[WebServer] 🚀 Starting...")
    
    # Load configuration
    config_path = Path(__file__).parent.parent.parent / "config" / "web_config.yaml"
    if config_path.exists():
        with open(config_path, 'r', encoding='utf-8') as f:
            config = yaml.safe_load(f)
            print(f"[WebServer] ✓ Configuration loaded: {config_path}")
    else:
        config = {}
        print(f"[WebServer] ⚠ Configuration file not found, using defaults: {config_path}")
    
    # Get current event loop
    loop = asyncio.get_event_loop()
    
    # Create TaskManager (singleton pattern)
    task_manager = TaskManager()
    print("[WebServer] ✓ TaskManager created")
    
    # Create WebSocket handler (inject event loop)
    websocket_handler = WebSocketHandler(event_loop=loop)
    print("[WebServer] ✓ WebSocketHandler created")
    
    # Initialize ROS2
    try:
        rclpy.init()
        print("[WebServer] ✓ ROS2 initialized")
    except Exception as e:
        print(f"[WebServer] ✗ ROS2 initialization failed: {e}")
    
    # Create ROS2 nodes
    try:
        # Create WebTerminalNode (inject TaskManager and WebSocketHandler)
        web_terminal_node = WebTerminalNode(
            task_manager=task_manager,
            websocket_handler=websocket_handler
        )
        print("[WebServer] ✓ WebTerminalNode created")
        print("[WebServer] ✓ CMD response listener started (persistent listener on /cmd/response)")
        
        # Run ROS2 spin in background thread (reuse same loop)
        ros_executor = loop.run_in_executor(None, web_terminal_node.spin)
        print("[WebServer] ✓ ROS2 node started")
        
        # Create StatusMonitorNode (monitor robot status)
        status_monitor_node = StatusMonitorNode(
            websocket_handler=websocket_handler
        )
        print("[WebServer] ✓ StatusMonitorNode created")
        
        # Run StatusMonitorNode in background thread
        status_executor = loop.run_in_executor(None, status_monitor_node.spin)
        print("[WebServer] ✓ Status monitor node started (2 Hz)")
        
    except Exception as e:
        print(f"[WebServer] ✗ ROS2 node initialization failed: {e}")
        web_terminal_node = None
    
    print("[WebServer] ✅ Startup complete")
    print("[WebServer] 📌 Architecture mode: Passive listener on /cmd/response (non-polling)")
    
    yield
    
    # Cleanup resources
    print("[WebServer] 🛑 Shutting down...")
    
    if status_monitor_node:
        status_monitor_node.shutdown()
        print("[WebServer] ✓ Status monitor node closed")
    
    if web_terminal_node:
        web_terminal_node.shutdown()
        print("[WebServer] ✓ ROS2 node closed")
    
    if status_executor:
        status_executor.cancel()
    
    if ros_executor:
        ros_executor.cancel()
    
    # Shutdown ROS2
    try:
        rclpy.shutdown()
        print("[WebServer] ✓ ROS2 shutdown complete")
    except Exception as e:
        print(f"[WebServer] ✗ ROS2 shutdown failed: {e}")
    
    print("[WebServer] ✅ Shutdown complete")


# Create FastAPI application
app = FastAPI(
    title="Lododo Robot Web API",
    description="Robot Web Control Interface REST API",
    version="1.0.0",
    lifespan=lifespan
)

# Configure CORS
app.add_middleware(
    CORSMiddleware,
    allow_origins=["*"],  # 生产环境应限制具体域名
    allow_credentials=True,
    allow_methods=["*"],
    allow_headers=["*"],
)


# ============================================
# 依赖注入
# ============================================

def get_web_terminal_node() -> WebTerminalNode:
    """Get WebTerminalNode instance"""
    if web_terminal_node is None:
        raise HTTPException(status_code=503, detail="ROS node not initialized")
    return web_terminal_node


def get_websocket_handler() -> WebSocketHandler:
    """Get WebSocketHandler instance"""
    if websocket_handler is None:
        raise HTTPException(status_code=503, detail="WebSocket handler not initialized")
    return websocket_handler


def get_task_manager():
    """Get TaskManager instance (dependency injection)"""
    if task_manager is None:
        raise HTTPException(status_code=503, detail="TaskManager not available")
    return task_manager


# ============================================
# API Routes
# ============================================

@app.get("/api")
async def api_root():
    """API root path"""
    return {
        "name": "Lododo Robot Web API",
        "version": "1.0.0",
        "status": "running",
        "ros_node": web_terminal_node is not None,
        "architecture": "passive_listener"
    }


@app.get("/api/health")
async def health_check():
    """Health check"""
    active_tasks = task_manager.get_active_tasks() if task_manager else []
    
    return {
        "status": "healthy",
        "ros_node": web_terminal_node is not None,
        "websocket_clients": len(websocket_handler.active_connections) if websocket_handler else 0,
        "active_tasks": len(active_tasks),
        "architecture": "passive_listener (non-polling)"
    }


# ============================================
# WebSocket Endpoints
# ============================================

@app.websocket("/ws")
async def websocket_endpoint(
    websocket: WebSocket,
    handler: WebSocketHandler = Depends(get_websocket_handler)
):
    """WebSocket connection endpoint"""
    await handler.connect(websocket)
    
    try:
        while True:
            # Receive client messages
            data = await websocket.receive_text()
            
            # Can process client commands here
            # Currently mainly used for receiving responses, actual commands sent via REST API
            print(f"[WebSocket] Received client message: {data[:100]}")
            
    except WebSocketDisconnect:
        handler.disconnect(websocket)
        print(f"[WebSocket] Client disconnected")
    except Exception as e:
        print(f"[WebSocket] Error: {e}")
        handler.disconnect(websocket)


# ============================================
# Mount Sub-routers
# ============================================

app.include_router(tasks.router, prefix="/api", tags=["tasks"])
app.include_router(maps.router, prefix="/api", tags=["maps"])
app.include_router(waypoints.router, prefix="/api", tags=["waypoints"])
app.include_router(settings.router, prefix="/api", tags=["settings"])
app.include_router(status.router, tags=["status"])
app.include_router(config.router, tags=["config"])


# ============================================
# Mount Static Files (Frontend)
# ============================================

# Get frontend build directory
frontend_dist = Path(__file__).parent.parent.parent / "web_frontend" / "dist"

if frontend_dist.exists():
    # Mount static assets (CSS, JS, etc.)
    app.mount("/assets", StaticFiles(directory=str(frontend_dist / "assets")), name="assets")
    
    # Mount root path (index.html)
    app.mount("/", StaticFiles(directory=str(frontend_dist), html=True), name="frontend")
    print(f"[WebServer] Static file serving enabled: {frontend_dist}")
else:
    print(f"[WebServer] Warning: Frontend build directory not found: {frontend_dist}")
    print("[WebServer] Please run: cd web_frontend && npm run build")


# ============================================
# Error Handling
# ============================================

@app.exception_handler(Exception)
async def global_exception_handler(request, exc):
    """Global exception handler"""
    print(f"[WebServer] Unhandled exception: {exc}")
    return JSONResponse(
        status_code=500,
        content={
            "error": "Internal server error",
            "detail": str(exc)
        }
    )


if __name__ == "__main__":
    import uvicorn
    
    # Development mode startup
    uvicorn.run(
        "web_server:app",
        host="0.0.0.0",
        port=8000,
        reload=False,  # Set to False in production
        log_level="info"
    )
