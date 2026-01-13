#!/usr/bin/env python3
"""
Web Terminal Node - ROS2 集成节点
通过 bot_cmd_interface 与 MissionPlanner 交互

架构设计：
- 常驻监听 /cmd/response（被动接收，非主动轮询）
- 所有响应委托给 TaskManager 处理（智能匹配 task_id/request_id）
- WebSocketHandler 仅负责广播更新给前端
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from typing import Optional

from bot_cmd_interface.sdk import (
    CommandRequest,
    CommandResponse,
    ResponseStatus,
    ErrorCode,
    create_navigate_request,
    create_exploration_request,
    create_patrol_request,
    create_get_status_request,
    create_cancel_task_request,
    create_emergency_stop_request,
)


class WebTerminalNode(Node):
    """Web 终端节点 - 负责 ROS2 通信和 CMD 消息路由"""
    
    def __init__(self, task_manager, websocket_handler):
        """
        初始化节点
        
        Args:
            task_manager: TaskManager 实例（处理任务状态更新）
            websocket_handler: WebSocketHandler 实例（广播更新给前端）
        """
        super().__init__('web_terminal_node')
        
        self.task_manager = task_manager
        self.websocket_handler = websocket_handler
        self._shutdown_flag = False
        
        # 发布器：发送命令请求到 /cmd/request
        self.cmd_publisher = self.create_publisher(
            String,
            '/cmd/request',
            10
        )
        
        # 订阅器：接收命令响应从 /cmd/response
        self.cmd_subscriber = self.create_subscription(
            String,
            '/cmd/response',
            self._response_callback,
            10
        )
        
        self.get_logger().info('Web Terminal Node 已初始化')
    
    def _response_callback(self, msg: String):
        """
        处理来自 /cmd/response 的响应（常驻监听）
        
        架构核心：被动接收所有响应，委托给 TaskManager 智能处理
        - TaskManager 根据 task_id/request_id 匹配任务
        - 不相关的响应自动忽略（not in cache）
        - 完全异步，不阻塞 ROS2 线程
        """
        try:
            # 使用 SDK 反序列化
            response = CommandResponse.from_json(msg.data)
            
            self.get_logger().debug(
                f"收到响应: request_id={response.request_id}, "
                f"status={response.status}, code={response.code}"
            )
            
            # 委托给 TaskManager 处理（智能匹配）
            response_dict = {
                'header': {'request_id': response.request_id},
                'body': {
                    'status': response.status.value if hasattr(response.status, 'value') else response.status,
                    'message': response.message,
                    'data': response.result if hasattr(response, 'result') else {},
                    'error_code': response.code.value if hasattr(response.code, 'value') else response.code
                }
            }
            
            updated_task = self.task_manager.update_task(response_dict)
            
            # 如果任务更新成功，通过 WebSocket 广播给前端（使用同步版本）
            if updated_task:
                self.websocket_handler.broadcast_task_update_sync(updated_task.to_dict())
                self.get_logger().info(
                    f"✓ 任务更新: {updated_task.request_id} -> {updated_task.status}"
                )
            
        except Exception as e:
            self.get_logger().error(f"处理响应失败: {e}")
    
    def _publish_request(self, request: CommandRequest) -> str:
        """
        发布请求到 /cmd/request
        
        Returns:
            request_id: SDK 自动生成的请求 ID
        """
        try:
            # 使用 SDK 序列化
            msg = String()
            msg.data = request.to_json()
            
            # 发布到 /cmd/request
            self.cmd_publisher.publish(msg)
            
            request_id = request.request_id
            self.get_logger().info(f"已发送请求: request_id={request_id}")
            
            return request_id
            
        except Exception as e:
            self.get_logger().error(f"发送请求失败: {e}")
            raise
    
    # ============================================
    # 公共 API 方法（供 FastAPI 调用）
    # ============================================
    
    def navigate_to_pose(self, x: float, y: float, yaw: float = 0.0) -> str:
        """
        发送导航到指定位姿的命令
        
        Args:
            x: 目标 X 坐标
            y: 目标 Y 坐标
            yaw: 目标朝向（弧度）
            
        Returns:
            request_id: 请求 ID（用于后续查询状态）
        """
        request = create_navigate_request(x=x, y=y, yaw=yaw)
        return self._publish_request(request)
    
    def start_exploration(
        self,
        map_name: Optional[str] = None,
        save_on_completion: bool = True
    ) -> str:
        """
        发送启动探索建图的命令
        
        Args:
            map_name: 地图名称（可选）
            save_on_completion: 完成后是否保存地图
            
        Returns:
            request_id: 请求 ID
        """
        request = create_exploration_request(
            map_name=map_name,
            save_on_completion=save_on_completion
        )
        return self._publish_request(request)
    
    def start_patrol(
        self,
        waypoint_file: str,
        mode: str = "loop"
    ) -> str:
        """
        发送启动巡逻的命令
        
        Args:
            waypoint_file: 路点文件路径
            mode: 巡逻模式（loop/once/bounce）
            
        Returns:
            request_id: 请求 ID
        """
        request = create_patrol_request(
            waypoint_file=waypoint_file,
            mode=mode
        )
        return self._publish_request(request)
    
    def query_task_status(self, task_id: Optional[str] = None) -> str:
        """
        查询任务状态
        
        Args:
            task_id: 任务 ID（可选，为 None 时查询所有任务）
            
        Returns:
            request_id: 请求 ID
        """
        request = create_get_status_request(task_id=task_id)
        return self._publish_request(request)
    
    def cancel_task(self, task_id: str) -> str:
        """
        取消任务
        
        Args:
            task_id: 任务 ID
            
        Returns:
            request_id: 请求 ID
        """
        request = create_cancel_task_request(task_id=task_id)
        return self._publish_request(request)
    
    def pause_task(self, task_id: str) -> str:
        """
        暂停任务
        
        Args:
            task_id: 任务 ID
            
        Returns:
            request_id: 请求 ID
        """
        from bot_cmd_interface.sdk import ActionType
        
        request = CommandRequest(
            action=ActionType.PAUSE_TASK,
            params={'task_id': task_id},
            priority=2,
            timeout=5.0
        )
        return self._publish_request(request)
    
    def resume_task(self, task_id: str) -> str:
        """
        恢复任务
        
        Args:
            task_id: 任务 ID
            
        Returns:
            request_id: 请求 ID
        """
        from bot_cmd_interface.sdk import ActionType
        
        request = CommandRequest(
            action=ActionType.RESUME_TASK,
            params={'task_id': task_id},
            priority=2,
            timeout=5.0
        )
        return self._publish_request(request)
    
    def emergency_stop(self) -> str:
        """
        紧急停止
        
        Returns:
            request_id: 请求 ID
        """
        request = create_emergency_stop_request()
        return self._publish_request(request)
    
    def spin(self):
        """运行节点（在独立线程中调用）"""
        while rclpy.ok() and not self._shutdown_flag:
            rclpy.spin_once(self, timeout_sec=0.1)
    
    def shutdown(self):
        """关闭节点"""
        self._shutdown_flag = True
        self.get_logger().info('Web Terminal Node 正在关闭...')
        self.destroy_node()
