#!/usr/bin/env python3
"""
waypoint_service_handler.py - 路点相关服务处理器

处理路点录制、保存、加载等服务请求
"""

import rclpy
from rclpy.node import Node
from bot_navigation_msgs.srv import WaypointControl, RecordWaypoints
from std_srvs.srv import Trigger


class WaypointServiceHandler:
    """路点服务处理器"""
    
    def __init__(self, node: Node, waypoint_recorder_clients: dict):
        """
        初始化路点服务处理器
        
        Args:
            node: ROS2 节点实例
            waypoint_recorder_clients: WaypointRecorder 服务客户端字典
        """
        self.node = node
        self._clients = waypoint_recorder_clients
    
    def handle_waypoint_control(self, request, response):
        """
        处理路点控制请求（保存/加载/清空）
        
        服务: /mission/waypoint_control
        """
        try:
            action = request.action
            
            if action == 'save':
                return self._handle_save(request, response)
            elif action == 'load':
                return self._handle_load(request, response)
            elif action == 'clear':
                return self._handle_clear(request, response)
            else:
                response.success = False
                response.message = f"Unknown action: {action}. Supported: save, load, clear"
                response.waypoint_count = 0
                
            self.node.get_logger().info(f"Waypoint control action: {action}, result: {response.message}")
            
        except Exception as e:
            response.success = False
            response.message = f"Waypoint control error: {str(e)}"
            response.waypoint_count = 0
            self.node.get_logger().error(f"Error in waypoint control: {str(e)}")
        
        return response
    
    def handle_record_waypoints(self, request, response):
        """
        处理路点录制控制请求（开始/停止/录制当前）
        
        服务: /mission/record_waypoints
        """
        try:
            action = request.action
            
            if action == 'start':
                return self._handle_start_recording(request, response)
            elif action == 'stop':
                return self._handle_stop_recording(request, response)
            elif action == 'record_current':
                return self._handle_record_current(request, response)
            else:
                response.success = False
                response.message = f"Unknown action: {action}. Supported: start, stop, record_current"
                response.recorded_count = 0
                
            self.node.get_logger().info(f"Record waypoints action: {action}, result: {response.message}")
            
        except Exception as e:
            response.success = False
            response.message = f"Record waypoints error: {str(e)}"
            response.recorded_count = 0
            self.node.get_logger().error(f"Error in record waypoints: {str(e)}")
        
        return response
    
    # ========== 内部方法 ==========
    
    def _handle_save(self, request, response):
        """处理保存路点"""
        if not self._clients['save'].wait_for_service(timeout_sec=2.0):
            response.success = False
            response.message = "WaypointRecorder service not available"
            response.waypoint_count = 0
            return response
        
        req = WaypointControl.Request()
        req.action = 'save'
        req.filename = request.filename
        future = self._clients['save'].call_async(req)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=5.0)
        
        if future.result() is not None:
            result = future.result()
            response.success = result.success
            response.message = result.message
            response.waypoint_count = result.waypoint_count
        else:
            response.success = False
            response.message = "Service call failed"
            response.waypoint_count = 0
        
        return response
    
    def _handle_load(self, request, response):
        """处理加载路点"""
        if not self._clients['load'].wait_for_service(timeout_sec=2.0):
            response.success = False
            response.message = "WaypointRecorder service not available"
            response.waypoint_count = 0
            return response
        
        req = WaypointControl.Request()
        req.action = 'load'
        req.filename = request.filename
        future = self._clients['load'].call_async(req)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=5.0)
        
        if future.result() is not None:
            result = future.result()
            response.success = result.success
            response.message = result.message
            response.waypoint_count = result.waypoint_count
        else:
            response.success = False
            response.message = "Service call failed"
            response.waypoint_count = 0
        
        return response
    
    def _handle_clear(self, request, response):
        """处理清空路点"""
        if not self._clients['clear'].wait_for_service(timeout_sec=2.0):
            response.success = False
            response.message = "WaypointRecorder service not available"
            response.waypoint_count = 0
            return response
        
        req = Trigger.Request()
        future = self._clients['clear'].call_async(req)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=5.0)
        
        if future.result() is not None:
            result = future.result()
            response.success = result.success
            response.message = result.message
            response.waypoint_count = 0
        else:
            response.success = False
            response.message = "Service call failed"
            response.waypoint_count = 0
        
        return response
    
    def _handle_start_recording(self, request, response):
        """处理开始自动录制"""
        if not self._clients['start_recording'].wait_for_service(timeout_sec=2.0):
            response.success = False
            response.message = "WaypointRecorder service not available"
            response.recorded_count = 0
            return response
        
        req = Trigger.Request()
        future = self._clients['start_recording'].call_async(req)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=5.0)
        
        if future.result() is not None:
            result = future.result()
            response.success = result.success
            response.message = result.message
            response.recorded_count = 0
        else:
            response.success = False
            response.message = "Service call failed"
            response.recorded_count = 0
        
        return response
    
    def _handle_stop_recording(self, request, response):
        """处理停止录制"""
        if not self._clients['stop_recording'].wait_for_service(timeout_sec=2.0):
            response.success = False
            response.message = "WaypointRecorder service not available"
            response.recorded_count = 0
            return response
        
        req = Trigger.Request()
        future = self._clients['stop_recording'].call_async(req)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=5.0)
        
        if future.result() is not None:
            result = future.result()
            response.success = result.success
            response.message = result.message
            response.recorded_count = 0
        else:
            response.success = False
            response.message = "Service call failed"
            response.recorded_count = 0
        
        return response
    
    def _handle_record_current(self, request, response):
        """处理录制当前位置"""
        if not self._clients['record_current'].wait_for_service(timeout_sec=2.0):
            response.success = False
            response.message = "WaypointRecorder service not available"
            response.recorded_count = 0
            return response
        
        req = Trigger.Request()
        future = self._clients['record_current'].call_async(req)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=5.0)
        
        if future.result() is not None:
            result = future.result()
            response.success = result.success
            response.message = result.message
            response.recorded_count = 1 if result.success else 0
        else:
            response.success = False
            response.message = "Service call failed"
            response.recorded_count = 0
        
        return response
