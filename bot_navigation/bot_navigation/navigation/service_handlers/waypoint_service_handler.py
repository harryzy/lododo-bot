#!/usr/bin/env python3
"""
waypoint_service_handler.py - 路点相关服务处理器

处理路点录制、保存、加载等服务请求

重构说明:
- 内置 WaypointRecorder 实例，不依赖外部服务
- 订阅定位话题获取实时位姿
- 统一状态管理和异常处理
- 提供完整的服务控制接口
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from bot_navigation_msgs.srv import WaypointControl, RecordWaypoints
from std_srvs.srv import Trigger
from ..waypoint_recorder import WaypointRecorder
import os


class WaypointServiceHandler:
    """
    路点服务处理器
    
    提供路点录制、保存、加载等服务的统一接口
    内置 WaypointRecorder 实例，直接处理录制逻辑
    """
    
    def __init__(self, node: Node, pose_topic: str = '/rtabmap/localization_pose', 
                 backup_pose_topic: str = '/wheel/odom', persistence_dir: str = None):
        """
        初始化路点服务处理器
        
        Args:
            node: ROS2 节点实例
            pose_topic: 定位话题名称
            backup_pose_topic: 备用定位话题（如里程计）
            persistence_dir: 路点持久化目录
        """
        self.node = node
        
        # 展开路径中的~符号
        if persistence_dir:
            persistence_dir = os.path.expanduser(persistence_dir)
        
        # 内置 WaypointRecorder 实例
        self._recorder = WaypointRecorder(persistence_dir=persistence_dir)
        
        # 当前位姿
        self._current_pose: PoseStamped = None
        self._pose_received = False
        
        # 订阅定位话题
        self._pose_topic = pose_topic
        self._backup_pose_topic = backup_pose_topic
        self._setup_pose_subscriptions()
        
        self.node.get_logger().info(f'WaypointServiceHandler initialized (pose_topic: {pose_topic})')
    
    def _setup_pose_subscriptions(self):
        """设置位姿话题订阅"""
        # 尝试订阅 PoseWithCovarianceStamped（RTABMap 定位）
        try:
            self._pose_sub = self.node.create_subscription(
                PoseWithCovarianceStamped,
                self._pose_topic,
                self._pose_with_cov_callback,
                10
            )
            self.node.get_logger().info(f'Subscribed to {self._pose_topic} (PoseWithCovarianceStamped)')
        except Exception:
            # 备用：订阅 PoseStamped
            self._pose_sub = self.node.create_subscription(
                PoseStamped,
                self._pose_topic,
                self._pose_callback,
                10
            )
            self.node.get_logger().info(f'Subscribed to {self._pose_topic} (PoseStamped)')
        
        # 备用：订阅里程计
        self._odom_sub = self.node.create_subscription(
            Odometry,
            self._backup_pose_topic,
            self._odom_callback,
            10
        )
        
        # 自动录制定时器（100ms 检查一次）
        self._auto_record_timer = self.node.create_timer(
            0.1,
            self._auto_record_callback
        )
    
    def _pose_with_cov_callback(self, msg: PoseWithCovarianceStamped):
        """位姿话题回调 (PoseWithCovarianceStamped)"""
        pose = PoseStamped()
        pose.header = msg.header
        pose.pose = msg.pose.pose
        self._current_pose = pose
        self._pose_received = True
    
    def _pose_callback(self, msg: PoseStamped):
        """位姿话题回调 (PoseStamped)"""
        self._current_pose = msg
        self._pose_received = True
    
    def _odom_callback(self, msg: Odometry):
        """里程计话题回调（备用）"""
        if not self._pose_received:
            pose = PoseStamped()
            pose.header = msg.header
            pose.pose = msg.pose.pose
            self._current_pose = pose
    
    def _auto_record_callback(self):
        """自动录制定时器回调"""
        if not self._recorder.is_recording():
            return
        
        if self._current_pose is None:
            # 每5秒提醒一次未收到位姿
            if not hasattr(self, '_last_pose_warning_time'):
                self._last_pose_warning_time = 0
            current_time = self.node.get_clock().now().nanoseconds / 1e9
            if current_time - self._last_pose_warning_time > 5.0:
                self.node.get_logger().warn('Auto-recording active but no pose received yet')
                self._last_pose_warning_time = current_time
            return
        
        # 尝试添加路点
        current_time = self.node.get_clock().now().nanoseconds / 1e9
        result = self._recorder.add_waypoint(self._current_pose, current_time)
        
        if result['added']:
            count = self._recorder.get_waypoint_count()
            x = self._current_pose.pose.position.x
            y = self._current_pose.pose.position.y
            self.node.get_logger().info(f'Auto-recorded waypoint #{count} at ({x:.2f}, {y:.2f})')
    
    # ========== 服务处理方法 ==========
    
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
        try:
            count = self._recorder.get_waypoint_count()
            if count == 0:
                response.success = False
                response.message = "No waypoints to save"
                response.waypoint_count = 0
                return response
            
            filename = request.filename
            if self._recorder.save_waypoints(filename):
                response.success = True
                response.message = f"Saved {count} waypoints to {filename}"
                response.waypoint_count = count
                self.node.get_logger().info(f'Saved {count} waypoints to {filename}')
            else:
                response.success = False
                response.message = f"Failed to save waypoints to {filename}"
                response.waypoint_count = 0
                
        except Exception as e:
            response.success = False
            response.message = f"Save error: {str(e)}"
            response.waypoint_count = 0
            self.node.get_logger().error(f"Error saving waypoints: {str(e)}")
        
        return response
    
    def _handle_load(self, request, response):
        """处理加载路点"""
        try:
            filename = request.filename
            if self._recorder.load_waypoints(filename):
                count = self._recorder.get_waypoint_count()
                response.success = True
                response.message = f"Loaded {count} waypoints from {filename}"
                response.waypoint_count = count
                self.node.get_logger().info(f'Loaded {count} waypoints from {filename}')
            else:
                response.success = False
                response.message = f"Failed to load waypoints from {filename}"
                response.waypoint_count = 0
                
        except Exception as e:
            response.success = False
            response.message = f"Load error: {str(e)}"
            response.waypoint_count = 0
            self.node.get_logger().error(f"Error loading waypoints: {str(e)}")
        
        return response
    
    def _handle_clear(self, request, response):
        """处理清空路点"""
        try:
            count = self._recorder.get_waypoint_count()
            self._recorder.clear_waypoints()
            response.success = True
            response.message = f"Cleared {count} waypoints"
            response.waypoint_count = 0
            self.node.get_logger().info(f'Cleared {count} waypoints')
            
        except Exception as e:
            response.success = False
            response.message = f"Clear error: {str(e)}"
            response.waypoint_count = 0
            self.node.get_logger().error(f"Error clearing waypoints: {str(e)}")
        
        return response
    
    def _handle_start_recording(self, request, response):
        """处理开始自动录制"""
        try:
            if self._recorder.is_recording():
                response.success = False
                response.message = "Already recording"
                response.recorded_count = self._recorder.get_waypoint_count()
                return response
            
            # 从请求中获取参数，如果没有则使用默认值
            interval = getattr(request, 'interval', 1.0) if hasattr(request, 'interval') else 1.0
            min_distance = getattr(request, 'min_distance', 0.5) if hasattr(request, 'min_distance') else 0.5
            
            current_count = self._recorder.get_waypoint_count()
            self._recorder.start_recording(
                interval=interval,
                min_distance=min_distance,
                clear_existing=False  # 不清空已有路点
            )
            
            response.success = True
            response.message = f"Started auto-recording (interval={interval}s, min_distance={min_distance}m). Current waypoints: {current_count}"
            response.recorded_count = current_count
            self.node.get_logger().info(f'Started auto-recording with {current_count} existing waypoints')
            
        except Exception as e:
            response.success = False
            response.message = f"Start recording error: {str(e)}"
            response.recorded_count = 0
            self.node.get_logger().error(f"Error starting recording: {str(e)}")
        
        return response
    
    def _handle_stop_recording(self, request, response):
        """处理停止录制"""
        try:
            if not self._recorder.is_recording():
                response.success = False
                response.message = "Not recording"
                response.recorded_count = self._recorder.get_waypoint_count()
                return response
            
            count = self._recorder.stop_recording()
            response.success = True
            response.message = f"Stopped recording. Total waypoints: {count}"
            response.recorded_count = count
            self.node.get_logger().info(f'Stopped recording with {count} total waypoints')
            
        except Exception as e:
            response.success = False
            response.message = f"Stop recording error: {str(e)}"
            response.recorded_count = 0
            self.node.get_logger().error(f"Error stopping recording: {str(e)}")
        
        return response
    
    def _handle_record_current(self, request, response):
        """处理录制当前位置"""
        try:
            if self._current_pose is None:
                response.success = False
                response.message = "No pose available yet"
                response.recorded_count = 0
                return response
            
            self._recorder.add_waypoint_manual(self._current_pose)
            count = self._recorder.get_waypoint_count()
            x = self._current_pose.pose.position.x
            y = self._current_pose.pose.position.y
            
            response.success = True
            response.message = f"Recorded waypoint #{count} at ({x:.2f}, {y:.2f})"
            response.recorded_count = 1
            self.node.get_logger().info(f'Manually recorded waypoint #{count} at ({x:.2f}, {y:.2f})')
            
        except Exception as e:
            response.success = False
            response.message = f"Record current error: {str(e)}"
            response.recorded_count = 0
            self.node.get_logger().error(f"Error recording current pose: {str(e)}")
        
        return response
    
    # ========== 辅助方法 ==========
    
    def get_waypoint_count(self) -> int:
        """获取当前路点数量"""
        return self._recorder.get_waypoint_count()
    
    def get_waypoints(self):
        """获取所有路点"""
        return self._recorder.get_waypoints()
    
    def is_recording(self) -> bool:
        """检查是否正在录制"""
        return self._recorder.is_recording()
