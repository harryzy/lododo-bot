#!/usr/bin/env python3
"""
NavigationExecutor - 通用导航执行器基类

功能 / Features:
  - 封装 Nav2 ActionClient 通用接口
  - TF 监听与坐标转换
  - 目标点验证与碰撞检测
  - 导航状态管理

Author: LeKiwi Bot Development Team
Date: 2025-12-22
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.duration import Duration

from nav2_msgs.action import NavigateToPose, Spin
from geometry_msgs.msg import PoseStamped, Quaternion
from nav_msgs.msg import OccupancyGrid

from tf2_ros import TransformException, Buffer, TransformListener
import tf2_geometry_msgs

import math
from typing import Optional, Tuple
from enum import Enum


class NavigationState(Enum):
    """导航状态枚举"""
    IDLE = 'idle'
    WAITING_ACCEPT = 'waiting_accept'
    EXECUTING = 'executing'
    CANCELING = 'canceling'
    SUCCESS = 'success'
    FAILED = 'failed'
    CANCELED = 'canceled'


class NavigationExecutor:
    """
    通用导航执行器基类
    
    提供 Nav2 导航的通用功能：
    - 发送导航目标
    - 取消导航
    - 查询导航状态
    - TF 转换
    - 目标点验证
    """
    
    def __init__(self, node: Node, action_name: str = 'navigate_to_pose'):
        """
        初始化导航执行器
        
        Args:
            node: ROS2 节点实例
            action_name: Nav2 action 名称
        """
        self.node = node
        self.logger = node.get_logger()
        
        # Action客户端
        self._nav_action_client = ActionClient(
            node,
            NavigateToPose,
            action_name
        )
        
        # Spin Action客户端（用于原地旋转）
        self._spin_action_client = ActionClient(
            node,
            Spin,
            'spin'
        )
        
        # Spin状态管理
        self._spin_goal_handle = None
        self._is_spinning = False
        
        # TF 监听器
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, node)
        
        # 状态管理
        self._nav_state = NavigationState.IDLE
        self._current_goal_handle = None
        self._current_goal = None
        self._goal_start_time = None
        self._goal_timeout = 60.0  # 默认超时60秒
        
        # 地图信息（用于目标点验证）
        self._current_map: Optional[OccupancyGrid] = None
        self._safe_distance = 0.4  # 默认安全距离(m)
        
        self.logger.info(f'NavigationExecutor initialized with action: {action_name}')
    
    def set_map(self, map_msg: OccupancyGrid):
        """
        更新地图信息
        
        Args:
            map_msg: 占据栅格地图消息
        """
        self._current_map = map_msg
    
    def set_timeout(self, timeout: float):
        """
        设置导航超时时间
        
        Args:
            timeout: 超时时间(秒)
        """
        self._goal_timeout = timeout
    
    def set_safe_distance(self, distance: float):
        """
        设置安全距离
        
        Args:
            distance: 安全距离(m)
        """
        self._safe_distance = distance
    
    def get_robot_pose(self, target_frame: str = 'map') -> Optional[PoseStamped]:
        """
        获取机器人当前位姿
        
        Args:
            target_frame: 目标坐标系
            
        Returns:
            PoseStamped: 机器人位姿，失败返回None
        """
        try:
            # 从 base_link 到 target_frame 的变换
            transform = self._tf_buffer.lookup_transform(
                target_frame,
                'base_link',
                rclpy.time.Time(),
                timeout=Duration(seconds=0.5)
            )
            
            # 构造 PoseStamped
            pose = PoseStamped()
            pose.header.frame_id = target_frame
            pose.header.stamp = self.node.get_clock().now().to_msg()
            pose.pose.position.x = transform.transform.translation.x
            pose.pose.position.y = transform.transform.translation.y
            pose.pose.position.z = transform.transform.translation.z
            pose.pose.orientation = transform.transform.rotation
            
            return pose
            
        except TransformException as e:
            self.logger.warning(f'Failed to get robot pose: {e}')
            return None
    
    def transform_pose(self, pose: PoseStamped, target_frame: str) -> Optional[PoseStamped]:
        """
        转换位姿到目标坐标系
        
        Args:
            pose: 源位姿
            target_frame: 目标坐标系
            
        Returns:
            转换后的位姿，失败返回None
        """
        try:
            transformed_pose = self._tf_buffer.transform(
                pose,
                target_frame,
                timeout=Duration(seconds=0.5)
            )
            return transformed_pose
        except TransformException as e:
            self.logger.warning(f'Failed to transform pose: {e}')
            return None
    
    def is_goal_valid(self, goal: PoseStamped) -> bool:
        """
        验证目标点是否有效（不在障碍物中）
        
        Args:
            goal: 目标位姿
            
        Returns:
            bool: 目标有效返回True
        """
        if self._current_map is None:
            self.logger.warning('No map available for goal validation')
            return True  # 无地图时默认有效
        
        # 转换到地图坐标系
        if goal.header.frame_id != self._current_map.header.frame_id:
            goal = self.transform_pose(goal, self._current_map.header.frame_id)
            if goal is None:
                self.logger.warning('Goal validation: transform failed')
                return False
        
        # 将世界坐标转换为栅格坐标
        map_x = int((goal.pose.position.x - self._current_map.info.origin.position.x) 
                    / self._current_map.info.resolution)
        map_y = int((goal.pose.position.y - self._current_map.info.origin.position.y) 
                    / self._current_map.info.resolution)
        
        # 检查是否越界
        if (map_x < 0 or map_x >= self._current_map.info.width or
            map_y < 0 or map_y >= self._current_map.info.height):
            self.logger.warning(
                f'Goal validation: out of bounds '
                f'(map_x={map_x}, map_y={map_y}, '
                f'width={self._current_map.info.width}, height={self._current_map.info.height})'
            )
            return False
        
        # 检查目标点及周围是否有障碍物
        safe_cells = int(self._safe_distance / self._current_map.info.resolution)
        for dx in range(-safe_cells, safe_cells + 1):
            for dy in range(-safe_cells, safe_cells + 1):
                check_x = map_x + dx
                check_y = map_y + dy
                
                if (check_x < 0 or check_x >= self._current_map.info.width or
                    check_y < 0 or check_y >= self._current_map.info.height):
                    continue
                
                index = check_y * self._current_map.info.width + check_x
                if index >= len(self._current_map.data):
                    continue
                
                # 占据值: -1未知, 0-100占据概率
                cell_value = self._current_map.data[index]
                if cell_value > 50:  # 障碍物阈值
                    self.logger.warning(
                        f'Goal validation: obstacle detected at '
                        f'({check_x}, {check_y}), cell_value={cell_value}, '
                        f'goal=({goal.pose.position.x:.2f}, {goal.pose.position.y:.2f})'
                    )
                    return False
        
        return True
    
    def navigate_to_pose(self, goal: PoseStamped) -> bool:
        """
        导航到目标位姿（异步发送，立即返回）
        
        Args:
            goal: 目标位姿
            
        Returns:
            bool: 发送成功返回True
        """
        if self._nav_state != NavigationState.IDLE:
            self.logger.warning(f'Cannot send goal, current state: {self._nav_state.value}')
            return False
        
        # 验证目标点
        if not self.is_goal_valid(goal):
            self.logger.warning('Goal validation failed')
            return False
        
        # 等待action服务器
        if not self._nav_action_client.wait_for_server(timeout_sec=5.0):
            self.logger.error('Nav2 action server not available')
            return False
        
        # 构造action goal
        nav_goal = NavigateToPose.Goal()
        nav_goal.pose = goal
        
        # 发送goal
        self.logger.info(f'Sending navigation goal: ({goal.pose.position.x:.2f}, {goal.pose.position.y:.2f})')
        
        send_goal_future = self._nav_action_client.send_goal_async(
            nav_goal,
            feedback_callback=self._navigation_feedback_callback
        )
        
        self._nav_state = NavigationState.WAITING_ACCEPT
        self._current_goal = goal
        self._goal_start_time = self.node.get_clock().now()
        
        # 等待goal被接受
        send_goal_future.add_done_callback(self._goal_response_callback)
        
        return True
    
    def _goal_response_callback(self, future):
        """Goal响应回调"""
        goal_handle = future.result()
        
        if not goal_handle.accepted:
            self.logger.warning('Navigation goal rejected')
            self._nav_state = NavigationState.FAILED
            self._current_goal_handle = None
            return
        
        self.logger.info('Navigation goal accepted')
        self._nav_state = NavigationState.EXECUTING
        self._current_goal_handle = goal_handle
        
        # 等待结果
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._goal_result_callback)
    
    def _goal_result_callback(self, future):
        """Goal结果回调"""
        result = future.result()
        
        if result.status == 4:  # SUCCEEDED
            self.logger.info('Navigation succeeded')
            self._nav_state = NavigationState.SUCCESS
        elif result.status == 5:  # ABORTED
            self.logger.warning('Navigation aborted')
            self._nav_state = NavigationState.FAILED
        elif result.status == 6:  # CANCELED
            self.logger.info('Navigation canceled')
            self._nav_state = NavigationState.CANCELED
        else:
            self.logger.warning(f'Navigation ended with status: {result.status}')
            self._nav_state = NavigationState.FAILED
        
        self._current_goal_handle = None
        self._current_goal = None
    
    def _navigation_feedback_callback(self, feedback_msg):
        """导航反馈回调（子类可重写）"""
        pass
    
    def cancel_navigation(self) -> bool:
        """
        取消当前导航
        
        Returns:
            bool: 取消成功返回True
        """
        if self._current_goal_handle is None:
            self.logger.warning('No active goal to cancel')
            return False
        
        if self._nav_state == NavigationState.CANCELING:
            self.logger.warning('Cancel already in progress')
            return False
        
        self.logger.info('Canceling navigation...')
        self._nav_state = NavigationState.CANCELING
        
        cancel_future = self._current_goal_handle.cancel_goal_async()
        cancel_future.add_done_callback(self._cancel_response_callback)
        
        return True
    
    def _cancel_response_callback(self, future):
        """取消响应回调"""
        cancel_response = future.result()
        
        if len(cancel_response.goals_canceling) > 0:
            self.logger.info('Navigation cancel request accepted')
        else:
            self.logger.warning('Navigation cancel request failed')
            self._nav_state = NavigationState.FAILED
    
    def get_state(self) -> NavigationState:
        """
        获取当前导航状态
        
        Returns:
            NavigationState: 当前状态
        """
        return self._nav_state
    
    def reset_state(self):
        """重置状态到IDLE（在处理完SUCCESS/FAILED后调用）"""
        if self._nav_state in [NavigationState.SUCCESS, NavigationState.FAILED, NavigationState.CANCELED]:
            self._nav_state = NavigationState.IDLE
            self.logger.debug('State reset to IDLE')
        else:
            self.logger.warning(f'Cannot reset state from {self._nav_state.value}')
    
    def is_busy(self) -> bool:
        """
        检查是否正在导航
        
        Returns:
            bool: 忙碌返回True
        """
        return self._nav_state in [
            NavigationState.WAITING_ACCEPT,
            NavigationState.EXECUTING,
            NavigationState.CANCELING
        ]
    
    def reset(self):
        """重置导航状态到IDLE"""
        if self.is_busy():
            self.logger.warning('Cannot reset while navigation is active')
            return
        
        self._nav_state = NavigationState.IDLE
        self._current_goal_handle = None
        self._current_goal = None
        self._goal_start_time = None
    
    def check_timeout(self) -> bool:
        """
        检查导航是否超时
        
        Returns:
            bool: 超时返回True
        """
        if self._goal_start_time is None:
            return False
        
        if not self.is_busy():
            return False
        
        elapsed = (self.node.get_clock().now() - self._goal_start_time).nanoseconds / 1e9
        return elapsed > self._goal_timeout
    
    @staticmethod
    def calculate_distance(pose1: PoseStamped, pose2: PoseStamped) -> float:
        """
        计算两个位姿之间的欧氏距离
        
        Args:
            pose1: 位姿1
            pose2: 位姿2
            
        Returns:
            float: 距离(m)
        """
        dx = pose2.pose.position.x - pose1.pose.position.x
        dy = pose2.pose.position.y - pose1.pose.position.y
        return math.sqrt(dx * dx + dy * dy)
    
    @staticmethod
    def calculate_yaw(pose: PoseStamped) -> float:
        """
        从四元数计算yaw角
        
        Args:
            pose: 位姿
            
        Returns:
            float: yaw角(弧度)
        """
        q = pose.pose.orientation
        siny_cosp = 2 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1 - 2 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)
    
    def world_to_map(self, x: float, y: float, map_msg: OccupancyGrid) -> Optional[Tuple[int, int]]:
        """
        世界坐标转地图坐标
        
        Args:
            x: 世界坐标x
            y: 世界坐标y
            map_msg: 地图消息
            
        Returns:
            (mx, my): 地图坐标，失败返回None
        """
        mx = int((x - map_msg.info.origin.position.x) / map_msg.info.resolution)
        my = int((y - map_msg.info.origin.position.y) / map_msg.info.resolution)
        return (mx, my)
    
    def map_to_world(self, mx: int, my: int, map_msg: OccupancyGrid) -> Optional[Tuple[float, float]]:
        """
        地图坐标转世界坐标
        
        Args:
            mx: 地图坐标x
            my: 地图坐标y
            map_msg: 地图消息
            
        Returns:
            (x, y): 世界坐标，失败返回None
        """
        x = mx * map_msg.info.resolution + map_msg.info.origin.position.x
        y = my * map_msg.info.resolution + map_msg.info.origin.position.y
        return (x, y)
    
    def get_robot_yaw(self) -> Optional[float]:
        """
        获取机器人当前朝向（yaw角）
        
        Returns:
            float: yaw角(弧度)，失败返回None
        """
        try:
            pose_stamped = self.get_robot_pose()
            if pose_stamped:
                return self.calculate_yaw(pose_stamped)
        except Exception as e:
            self.logger.warning(f'无法获取机器人朝向: {e}')
        
        return None
    
    def calculate_angle_to_target(self, target_x: float, target_y: float) -> Optional[float]:
        """
        计算机器人到目标点的角度
        
        Args:
            target_x: 目标世界坐标x
            target_y: 目标世界坐标y
            
        Returns:
            float: 角度(弧度)，失败返回None
        """
        robot_pose = self.get_robot_pose()
        if robot_pose is None:
            return None
        
        dx = target_x - robot_pose.pose.position.x
        dy = target_y - robot_pose.pose.position.y
        return math.atan2(dy, dx)
    
    def rotate_to_heading(self, target_yaw: float, timeout: float = 20.0) -> bool:
        """
        旋转到指定朝向（同步，阻塞至完成）
        
        Args:
            target_yaw: 目标朝向(弧度)
            timeout: 超时时间(秒)，默认20秒
            
        Returns:
            bool: 旋转成功返回True
        """
        import time
        import rclpy
        
        # 检查是否已经在旋转
        if self._is_spinning:
            self.logger.warning('已经在旋转中，跳过')
            return False
        
        # 等待Spin服务器
        self.logger.info('等待Spin action服务器...')
        if not self._spin_action_client.wait_for_server(timeout_sec=5.0):
            self.logger.error('Spin action服务器未响应，可能未启动')
            return False
        
        self.logger.info('Spin action服务器已就绪')
        
        # 计算需要旋转的角度
        current_yaw = self.get_robot_yaw()
        if current_yaw is None:
            self.logger.error('无法获取当前朝向')
            return False
        
        # 角度差归一化到[-pi, pi]
        angle_diff = target_yaw - current_yaw
        while angle_diff > math.pi:
            angle_diff -= 2 * math.pi
        while angle_diff < -math.pi:
            angle_diff += 2 * math.pi
        
        self.logger.info(
            f'🔄 旋转朝向: 当前={math.degrees(current_yaw):.1f}°, '
            f'目标={math.degrees(target_yaw):.1f}°, '
            f'转动={math.degrees(angle_diff):.1f}°'
        )
        
        # 如果角度差很小，跳过旋转
        if abs(angle_diff) < math.radians(5):  # 5度容差
            self.logger.info('朝向差异较小，跳过旋转')
            return True
        
        # 创建Spin目标
        spin_goal = Spin.Goal()
        spin_goal.target_yaw = angle_diff  # 相对旋转角度
        
        self._is_spinning = True
        
        # 发送目标
        send_goal_future = self._spin_action_client.send_goal_async(spin_goal)
        
        # 等待goal被接受
        start_time = time.time()
        while not send_goal_future.done() and (time.time() - start_time) < timeout:
            rclpy.spin_once(self.node, timeout_sec=0.1)
        
        if not send_goal_future.done():
            self.logger.error('Spin请求超时')
            self._is_spinning = False
            return False
        
        self._spin_goal_handle = send_goal_future.result()
        if not self._spin_goal_handle.accepted:
            self.logger.error('Spin目标被拒绝')
            self._is_spinning = False
            return False
        
        self.logger.info('✅ Spin目标已接受，等待完成...')
        
        # 等待结果
        result_future = self._spin_goal_handle.get_result_async()
        
        start_time = time.time()
        while not result_future.done() and (time.time() - start_time) < timeout:
            rclpy.spin_once(self.node, timeout_sec=0.1)
        
        self._is_spinning = False
        self._spin_goal_handle = None
        
        if not result_future.done():
            self.logger.error(f'Spin执行超时（>{timeout:.1f}秒）')
            return False
        
        result = result_future.result()
        if result.status == 4:  # SUCCEEDED
            self.logger.info('✅ 旋转完成')
            return True
        else:
            self.logger.warning(f'旋转失败: 状态={result.status}')
            return False
