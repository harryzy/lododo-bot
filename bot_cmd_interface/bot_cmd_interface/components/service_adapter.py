#!/usr/bin/env python3
"""
ServiceAdapter - 服务适配器
Service Adapter - bridges CommandAdapter to ROS2 services

功能 / Features:
- 将统一命令接口的请求转换为对应的ROS2服务调用
- Converts unified command interface requests to corresponding ROS2 service calls
- 处理服务调用超时和错误 / Handles service call timeout and errors
- 支持异步服务调用 / Supports async service calls
"""

import rclpy
from rclpy.node import Node
from typing import Dict, Any, Optional, Tuple
import threading

# 导入服务消息类型 / Import service message types
from bot_navigation_msgs.srv import (
    NavigateToPose,
    EmergencyStop,
    StartExploration,
    StartPatrol,
    GetTaskStatus,
    TaskControl
)

# 导入SDK / Import SDK
from bot_cmd_interface.sdk.action_types import ActionType
from bot_cmd_interface.sdk.message import CommandRequest, CommandResponse, ErrorCode


class ServiceAdapter:
    """
    服务适配器 - 桥接命令接口和ROS2服务
    Service Adapter - bridges command interface to ROS2 services
    
    将ActionType映射到对应的ROS2服务调用 /
    Maps ActionType to corresponding ROS2 service calls
    """
    
    def __init__(self, node: Node, timeout: float = 10.0):
        """
        初始化服务适配器 / Initialize service adapter
        
        Args:
            node: ROS2节点 / ROS2 node
            timeout: 服务调用默认超时时间（秒）/ Default service call timeout (seconds)
        """
        self.node = node
        self.timeout = timeout
        
        # 创建服务客户端 / Create service clients
        self._clients: Dict[str, Any] = {}
        self._create_service_clients()
        
        # 统计信息 / Statistics
        self._stats = {
            'total_calls': 0,
            'successful_calls': 0,
            'failed_calls': 0,
            'timeout_calls': 0
        }
        self._stats_lock = threading.Lock()
        
        self.node.get_logger().info('ServiceAdapter initialized')
    
    def _create_service_clients(self):
        """创建所有服务客户端 / Create all service clients"""
        # 导航服务 / Navigation services
        self._clients['navigate_to_pose'] = self.node.create_client(
            NavigateToPose, '/mission/navigate_to_pose'
        )
        
        # 紧急停止 / Emergency stop
        self._clients['emergency_stop'] = self.node.create_client(
            EmergencyStop, '/mission/emergency_stop'
        )
        
        # 探索服务 / Exploration service
        self._clients['start_exploration'] = self.node.create_client(
            StartExploration, '/mission/start_exploration'
        )
        
        # 巡航服务 / Patrol service
        self._clients['start_patrol'] = self.node.create_client(
            StartPatrol, '/mission/start_patrol'
        )
        
        # 任务状态查询 / Task status query
        self._clients['get_task_status'] = self.node.create_client(
            GetTaskStatus, '/mission/get_task_status'
        )
        
        # 任务控制服务 / Task control services
        self._clients['pause_task'] = self.node.create_client(
            TaskControl, '/mission/pause_task'
        )
        self._clients['resume_task'] = self.node.create_client(
            TaskControl, '/mission/resume_task'
        )
        self._clients['cancel_task'] = self.node.create_client(
            TaskControl, '/mission/cancel_task'
        )
        
        self.node.get_logger().info(
            f'Created {len(self._clients)} service clients'
        )
    
    def wait_for_services(self, timeout: float = 10.0) -> bool:
        """
        等待所有服务可用 / Wait for all services to be available
        
        Args:
            timeout: 超时时间（秒）/ Timeout in seconds
        
        Returns:
            bool: 是否所有服务都可用 / Whether all services are available
        """
        self.node.get_logger().info(f'Waiting for services (timeout: {timeout}s)...')
        
        all_ready = True
        for name, client in self._clients.items():
            if not client.wait_for_service(timeout_sec=timeout):
                self.node.get_logger().warn(f'Service not available: {name}')
                all_ready = False
            else:
                self.node.get_logger().debug(f'Service ready: {name}')
        
        if all_ready:
            self.node.get_logger().info('All services are ready')
        else:
            self.node.get_logger().warn('Some services are not available')
        
        return all_ready
    
    async def process_request(self, request: CommandRequest) -> Tuple[bool, Dict[str, Any], str]:
        """
        处理请求，调用相应的ROS2服务 / Process request, call corresponding ROS2 service
        
        Args:
            request: 命令请求 / Command request
        
        Returns:
            Tuple[bool, Dict, str]: (是否成功, 结果字典, 错误消息) /
                                    (success, result dict, error message)
        """
        with self._stats_lock:
            self._stats['total_calls'] += 1
        
        try:
            # 根据action类型分发到不同的处理器 / Dispatch to different handlers based on action type
            if request.action == ActionType.NAVIGATE_TO_POSE:
                return await self._handle_navigate_to_pose(request)
            
            elif request.action == ActionType.EMERGENCY_STOP:
                return await self._handle_emergency_stop(request)
            
            elif request.action == ActionType.START_EXPLORATION:
                return await self._handle_start_exploration(request)
            
            elif request.action == ActionType.START_PATROL:
                return await self._handle_start_patrol(request)
            
            elif request.action == ActionType.GET_TASK_STATUS:
                return await self._handle_get_task_status(request)
            
            elif request.action == ActionType.PAUSE_TASK:
                return await self._handle_pause_task(request)
            
            elif request.action == ActionType.RESUME_TASK:
                return await self._handle_resume_task(request)
            
            elif request.action == ActionType.CANCEL_TASK:
                return await self._handle_cancel_task(request)
            
            elif request.action == ActionType.GET_ROBOT_STATUS:
                # 机器人状态查询（可以在这里实现或使用mock）
                # Robot status query (can be implemented here or use mock)
                return await self._handle_get_robot_status(request)
            
            else:
                # 不支持的操作 / Unsupported action
                error_msg = f'Unsupported action: {request.action}'
                self.node.get_logger().warn(error_msg)
                with self._stats_lock:
                    self._stats['failed_calls'] += 1
                return False, {}, error_msg
        
        except Exception as e:
            error_msg = f'Failed to process request: {str(e)}'
            self.node.get_logger().error(error_msg)
            with self._stats_lock:
                self._stats['failed_calls'] += 1
            return False, {}, error_msg
    
    async def _call_service(self, client_name: str, request_msg) -> Tuple[bool, Any, str]:
        """
        调用ROS2服务 / Call ROS2 service
        
        Args:
            client_name: 客户端名称 / Client name
            request_msg: 服务请求消息 / Service request message
        
        Returns:
            Tuple[bool, response, str]: (是否成功, 响应, 错误消息) /
                                        (success, response, error message)
        """
        client = self._clients.get(client_name)
        if not client:
            return False, None, f'Service client not found: {client_name}'
        
        if not client.service_is_ready():
            return False, None, f'Service not ready: {client_name}'
        
        try:
            # 异步调用服务 / Call service asynchronously
            future = client.call_async(request_msg)
            
            # 等待响应（使用rclpy的executor）/ Wait for response
            rclpy.spin_until_future_complete(
                self.node,
                future,
                timeout_sec=self.timeout
            )
            
            if future.done():
                response = future.result()
                return True, response, ''
            else:
                with self._stats_lock:
                    self._stats['timeout_calls'] += 1
                return False, None, f'Service call timeout: {client_name}'
        
        except Exception as e:
            return False, None, f'Service call failed: {str(e)}'
    
    # ========== 具体服务处理方法 / Specific service handlers ==========
    
    async def _handle_navigate_to_pose(self, request: CommandRequest) -> Tuple[bool, Dict, str]:
        """处理导航到目标位姿 / Handle navigate to pose"""
        params = request.params
        
        # 构造服务请求 / Construct service request
        srv_request = NavigateToPose.Request()
        srv_request.x = float(params.get('x', 0.0))
        srv_request.y = float(params.get('y', 0.0))
        srv_request.yaw = float(params.get('yaw', 0.0))
        srv_request.frame_id = params.get('frame_id', 'map')
        
        # 调用服务 / Call service
        success, response, error_msg = await self._call_service('navigate_to_pose', srv_request)
        
        if success and response and response.success:
            with self._stats_lock:
                self._stats['successful_calls'] += 1
            return True, {'task_id': response.task_id}, response.message
        else:
            with self._stats_lock:
                self._stats['failed_calls'] += 1
            error = error_msg if error_msg else (response.message if response else 'Unknown error')
            return False, {}, error
    
    async def _handle_emergency_stop(self, request: CommandRequest) -> Tuple[bool, Dict, str]:
        """处理紧急停止 / Handle emergency stop"""
        params = request.params
        
        # 构造服务请求 / Construct service request
        srv_request = EmergencyStop.Request()
        srv_request.clear_tasks = params.get('clear_tasks', True)
        
        # 调用服务 / Call service
        success, response, error_msg = await self._call_service('emergency_stop', srv_request)
        
        if success and response and response.success:
            with self._stats_lock:
                self._stats['successful_calls'] += 1
            return True, {'message': response.message}, response.message
        else:
            with self._stats_lock:
                self._stats['failed_calls'] += 1
            error = error_msg if error_msg else (response.message if response else 'Unknown error')
            return False, {}, error
    
    async def _handle_start_exploration(self, request: CommandRequest) -> Tuple[bool, Dict, str]:
        """处理开始探索 / Handle start exploration"""
        params = request.params
        
        # 构造服务请求 / Construct service request
        srv_request = StartExploration.Request()
        srv_request.map_name = params.get('map_name', 'exploration_map')
        srv_request.save_map = params.get('save_map', True)
        srv_request.max_duration = float(params.get('max_duration', 0.0))
        srv_request.coverage_threshold = float(params.get('coverage_threshold', 0.8))
        
        # 调用服务 / Call service
        success, response, error_msg = await self._call_service('start_exploration', srv_request)
        
        if success and response and response.success:
            with self._stats_lock:
                self._stats['successful_calls'] += 1
            return True, {'task_id': response.task_id}, response.message
        else:
            with self._stats_lock:
                self._stats['failed_calls'] += 1
            error = error_msg if error_msg else (response.message if response else 'Unknown error')
            return False, {}, error
    
    async def _handle_start_patrol(self, request: CommandRequest) -> Tuple[bool, Dict, str]:
        """处理开始巡航 / Handle start patrol"""
        params = request.params
        
        # 构造服务请求 / Construct service request
        srv_request = StartPatrol.Request()
        srv_request.waypoint_file = params.get('waypoint_file', '')
        srv_request.patrol_mode = params.get('patrol_mode', 'loop')
        srv_request.speed_factor = float(params.get('speed_factor', 1.0))
        
        # 调用服务 / Call service
        success, response, error_msg = await self._call_service('start_patrol', srv_request)
        
        if success and response and response.success:
            with self._stats_lock:
                self._stats['successful_calls'] += 1
            return True, {'task_id': response.task_id}, response.message
        else:
            with self._stats_lock:
                self._stats['failed_calls'] += 1
            error = error_msg if error_msg else (response.message if response else 'Unknown error')
            return False, {}, error
    
    async def _handle_get_task_status(self, request: CommandRequest) -> Tuple[bool, Dict, str]:
        """处理获取任务状态 / Handle get task status"""
        params = request.params
        
        # 构造服务请求 / Construct service request
        srv_request = GetTaskStatus.Request()
        srv_request.task_id = params.get('task_id', '')
        
        # 调用服务 / Call service
        success, response, error_msg = await self._call_service('get_task_status', srv_request)
        
        if success and response and response.success:
            with self._stats_lock:
                self._stats['successful_calls'] += 1
            result = {
                'task_id': response.task_id,
                'task_type': response.task_type,
                'state': response.state,
                'progress': response.progress
            }
            return True, result, response.message
        else:
            with self._stats_lock:
                self._stats['failed_calls'] += 1
            error = error_msg if error_msg else (response.message if response else 'Unknown error')
            return False, {}, error
    
    async def _handle_pause_task(self, request: CommandRequest) -> Tuple[bool, Dict, str]:
        """处理暂停任务 / Handle pause task"""
        params = request.params
        
        # 构造服务请求 / Construct service request
        srv_request = TaskControl.Request()
        srv_request.task_id = params.get('task_id', '')
        
        # 调用服务 / Call service
        success, response, error_msg = await self._call_service('pause_task', srv_request)
        
        if success and response and response.success:
            with self._stats_lock:
                self._stats['successful_calls'] += 1
            return True, {'message': response.message}, response.message
        else:
            with self._stats_lock:
                self._stats['failed_calls'] += 1
            error = error_msg if error_msg else (response.message if response else 'Unknown error')
            return False, {}, error
    
    async def _handle_resume_task(self, request: CommandRequest) -> Tuple[bool, Dict, str]:
        """处理恢复任务 / Handle resume task"""
        params = request.params
        
        # 构造服务请求 / Construct service request
        srv_request = TaskControl.Request()
        srv_request.task_id = params.get('task_id', '')
        
        # 调用服务 / Call service
        success, response, error_msg = await self._call_service('resume_task', srv_request)
        
        if success and response and response.success:
            with self._stats_lock:
                self._stats['successful_calls'] += 1
            return True, {'message': response.message}, response.message
        else:
            with self._stats_lock:
                self._stats['failed_calls'] += 1
            error = error_msg if error_msg else (response.message if response else 'Unknown error')
            return False, {}, error
    
    async def _handle_cancel_task(self, request: CommandRequest) -> Tuple[bool, Dict, str]:
        """处理取消任务 / Handle cancel task"""
        params = request.params
        
        # 构造服务请求 / Construct service request
        srv_request = TaskControl.Request()
        srv_request.task_id = params.get('task_id', '')
        
        # 调用服务 / Call service
        success, response, error_msg = await self._call_service('cancel_task', srv_request)
        
        if success and response and response.success:
            with self._stats_lock:
                self._stats['successful_calls'] += 1
            return True, {'message': response.message}, response.message
        else:
            with self._stats_lock:
                self._stats['failed_calls'] += 1
            error = error_msg if error_msg else (response.message if response else 'Unknown error')
            return False, {}, error
    
    async def _handle_get_robot_status(self, request: CommandRequest) -> Tuple[bool, Dict, str]:
        """
        处理获取机器人状态 / Handle get robot status
        
        注意：这个功能需要额外实现，这里返回基本信息
        Note: This feature needs additional implementation, returning basic info here
        """
        # TODO: 实现真实的机器人状态查询 / Implement real robot status query
        # 可以订阅/battery_state, /robot_status等话题 / Can subscribe to /battery_state, /robot_status etc.
        
        result = {
            'status': 'ready',
            'battery_level': None,  # 需要从实际话题获取 / Need to get from actual topic
            'current_task': None,   # 可以查询MissionPlanner / Can query MissionPlanner
            'message': 'Robot status query not fully implemented yet'
        }
        
        with self._stats_lock:
            self._stats['successful_calls'] += 1
        
        return True, result, 'OK'
    
    def get_statistics(self) -> Dict[str, int]:
        """获取统计信息 / Get statistics"""
        with self._stats_lock:
            return self._stats.copy()
