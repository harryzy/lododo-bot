#!/usr/bin/env python3
"""
Web控制界面 - FastAPI 主应用
提供 REST API 和 WebSocket 服务
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
from .websocket_handler import WebSocketHandler
from .api import tasks, maps, waypoints, settings

# 全局变量
web_terminal_node: Optional[WebTerminalNode] = None
websocket_handler: Optional[WebSocketHandler] = None
ros_executor: Optional[asyncio.Future] = None


@asynccontextmanager
async def lifespan(app: FastAPI):
    """应用生命周期管理"""
    global web_terminal_node, websocket_handler, ros_executor
    
    print("[WebServer] 启动中...")
    
    # 加载配置
    config_path = Path(__file__).parent.parent.parent / "config" / "web_config.yaml"
    if config_path.exists():
        with open(config_path, 'r', encoding='utf-8') as f:
            config = yaml.safe_load(f)
            print(f"[WebServer] 配置已加载: {config_path}")
    else:
        config = {}
        print(f"[WebServer] 配置文件不存在，使用默认配置: {config_path}")
    
    # 创建 WebSocket 处理器
    websocket_handler = WebSocketHandler()
    
    # 初始化 ROS2
    try:
        rclpy.init()
        print("[WebServer] ROS2 已初始化")
    except Exception as e:
        print(f"[WebServer] ROS2 初始化失败: {e}")
    
    # 创建 ROS2 节点
    try:
        web_terminal_node = WebTerminalNode(response_callback=websocket_handler.broadcast_response)
        print("[WebServer] WebTerminalNode 已创建")
        
        # 在后台线程运行 ROS2 spin
        loop = asyncio.get_event_loop()
        ros_executor = loop.run_in_executor(None, web_terminal_node.spin)
        print("[WebServer] ROS2 节点已启动")
        
    except Exception as e:
        print(f"[WebServer] 初始化 ROS2 节点失败: {e}")
        web_terminal_node = None
    
    print("[WebServer] 启动完成 ✓")
    
    yield
    
    # 清理资源
    print("[WebServer] 关闭中...")
    
    if web_terminal_node:
        web_terminal_node.shutdown()
        print("[WebServer] ROS2 节点已关闭")
    
    if ros_executor:
        ros_executor.cancel()
    
    # 关闭 ROS2
    try:
        rclpy.shutdown()
        print("[WebServer] ROS2 已关闭")
    except Exception as e:
        print(f"[WebServer] ROS2 关闭失败: {e}")
    
    print("[WebServer] 关闭完成")


# 创建 FastAPI 应用
app = FastAPI(
    title="LeKiwi Robot Web API",
    description="机器人 Web 控制界面 REST API",
    version="1.0.0",
    lifespan=lifespan
)

# 配置 CORS
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
    """获取 WebTerminalNode 实例"""
    if web_terminal_node is None:
        raise HTTPException(status_code=503, detail="ROS node not initialized")
    return web_terminal_node


def get_websocket_handler() -> WebSocketHandler:
    """获取 WebSocketHandler 实例"""
    if websocket_handler is None:
        raise HTTPException(status_code=503, detail="WebSocket handler not initialized")
    return websocket_handler


# ============================================
# API 路由
# ============================================

@app.get("/api")
async def api_root():
    """API 根路径"""
    return {
        "name": "LeKiwi Robot Web API",
        "version": "1.0.0",
        "status": "running",
        "ros_node": web_terminal_node is not None
    }


@app.get("/api/health")
async def health_check():
    """健康检查"""
    return {
        "status": "healthy",
        "ros_node": web_terminal_node is not None,
        "websocket_clients": len(websocket_handler.active_connections) if websocket_handler else 0
    }


# ============================================
# WebSocket 端点
# ============================================

@app.websocket("/ws")
async def websocket_endpoint(
    websocket: WebSocket,
    handler: WebSocketHandler = Depends(get_websocket_handler)
):
    """WebSocket 连接端点"""
    await handler.connect(websocket)
    
    try:
        while True:
            # 接收客户端消息
            data = await websocket.receive_text()
            
            # 这里可以处理客户端发送的命令
            # 目前主要用于接收响应，实际命令通过 REST API 发送
            print(f"[WebSocket] 收到客户端消息: {data[:100]}")
            
    except WebSocketDisconnect:
        handler.disconnect(websocket)
        print(f"[WebSocket] 客户端断开连接")
    except Exception as e:
        print(f"[WebSocket] 错误: {e}")
        handler.disconnect(websocket)


# ============================================
# 挂载子路由
# ============================================

app.include_router(tasks.router, prefix="/api", tags=["tasks"])
app.include_router(maps.router, prefix="/api", tags=["maps"])
app.include_router(waypoints.router, prefix="/api", tags=["waypoints"])
app.include_router(settings.router, prefix="/api", tags=["settings"])


# ============================================
# 挂载静态文件（前端）
# ============================================

# 获取前端构建目录
frontend_dist = Path(__file__).parent.parent.parent / "web_frontend" / "dist"

if frontend_dist.exists():
    # 挂载静态资源（CSS、JS等）
    app.mount("/assets", StaticFiles(directory=str(frontend_dist / "assets")), name="assets")
    
    # 挂载根路径（index.html）
    app.mount("/", StaticFiles(directory=str(frontend_dist), html=True), name="frontend")
    print(f"[WebServer] 静态文件服务已启用: {frontend_dist}")
else:
    print(f"[WebServer] 警告: 前端构建目录不存在: {frontend_dist}")
    print("[WebServer] 请运行: cd web_frontend && npm run build")


# ============================================
# 错误处理
# ============================================

@app.exception_handler(Exception)
async def global_exception_handler(request, exc):
    """全局异常处理"""
    print(f"[WebServer] 未处理的异常: {exc}")
    return JSONResponse(
        status_code=500,
        content={
            "error": "Internal server error",
            "detail": str(exc)
        }
    )


if __name__ == "__main__":
    import uvicorn
    
    # 开发模式启动
    uvicorn.run(
        "web_server:app",
        host="0.0.0.0",
        port=8000,
        reload=False,  # 生产环境设为 False
        log_level="info"
    )
