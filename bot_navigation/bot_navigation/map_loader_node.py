#!/usr/bin/env python3
"""
地图加载节点 / Map Loader Node

功能:
- 从地图库加载指定地图
- 发布地图到/map话题
- 支持版本选择

Author: GitHub Copilot
Date: 2025-12-24
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import OccupancyGrid
from std_srvs.srv import Trigger
import yaml
import os
from pathlib import Path

from .map_library_manager import MapLibraryManager


class MapLoaderNode(Node):
    """
    地图加载节点 / Map loader node
    
    从地图库加载地图并发布
    """
    
    def __init__(self):
        super().__init__('map_loader_node')
        
        self.get_logger().info('Initializing MapLoaderNode...')
        
        # ===== 参数 =====
        self.declare_parameters(
            namespace='',
            parameters=[
                ('map_name', ''),
                ('map_version', 0),  # 0表示最新版本
                ('library_path', '~/lododo_bot/maps'),
                ('auto_load', True),  # 启动时自动加载
            ]
        )
        
        self._map_name = self.get_parameter('map_name').value
        self._map_version = self.get_parameter('map_version').value
        self._library_path = self.get_parameter('library_path').value
        self._auto_load = self.get_parameter('auto_load').value
        
        # ===== 地图库管理器 =====
        self._map_library = MapLibraryManager(
            node=self,
            library_path=self._library_path
        )
        
        # ===== 状态 =====
        self._map_loaded = False
        self._map_yaml_path = None
        
        # ===== 发布器 =====
        # 注意：通常地图由map_server发布，这里只是提供地图路径
        # 实际应用中应该调用map_server的服务来加载地图
        
        # ===== 服务 =====
        self._load_srv = self.create_service(
            Trigger,
            '/map_library/load_map',
            self._handle_load_map
        )
        
        self.get_logger().info('MapLoaderNode initialized successfully')
        self.get_logger().info(f'  - Map name: {self._map_name}')
        self.get_logger().info(f'  - Map version: {self._map_version} (0=latest)')
        self.get_logger().info(f'  - Library path: {self._library_path}')
        self.get_logger().info(f'  - Auto load: {self._auto_load}')
        
        # 自动加载地图
        if self._auto_load and self._map_name:
            self._load_map()
    
    def _handle_load_map(self, request, response):
        """
        处理加载地图请求 / Handle load map request
        
        Args:
            request: 触发请求
            response: 触发响应
        
        Returns:
            响应对象
        """
        self.get_logger().info('📝 Manual load map requested')
        
        success, message = self._load_map()
        
        response.success = success
        response.message = message
        return response
    
    def _load_map(self):
        """
        加载地图 / Load map
        
        Returns:
            (success, message)
        """
        if not self._map_name:
            message = 'Map name not specified'
            self.get_logger().error(message)
            return False, message
        
        self.get_logger().info(f'📍 Loading map: {self._map_name} (v{self._map_version})')
        
        # 从地图库加载地图
        version = self._map_version if self._map_version > 0 else None
        success, message, map_yaml_path = self._map_library.load_map(
            map_name=self._map_name,
            version=version
        )
        
        if success:
            self._map_loaded = True
            self._map_yaml_path = map_yaml_path
            self.get_logger().info(f'✅ Map loaded: {map_yaml_path}')
            self.get_logger().info('')
            self.get_logger().info('=' * 80)
            self.get_logger().info('⚠️  重要提示 / IMPORTANT:')
            self.get_logger().info('=' * 80)
            self.get_logger().info('')
            self.get_logger().info(f'地图已从地图库加载，但还需要由map_server发布到/map话题。')
            self.get_logger().info(f'Map loaded from library, but needs to be published by map_server.')
            self.get_logger().info('')
            self.get_logger().info('请在launch文件中配置map_server使用此地图:')
            self.get_logger().info('Please configure map_server in launch file to use this map:')
            self.get_logger().info('')
            self.get_logger().info(f'  map_file: {map_yaml_path}')
            self.get_logger().info('')
            self.get_logger().info('或者使用命令行启动map_server:')
            self.get_logger().info('Or start map_server via command line:')
            self.get_logger().info('')
            self.get_logger().info(f'  ros2 run nav2_map_server map_server --ros-args -p yaml_filename:={map_yaml_path}')
            self.get_logger().info('')
            self.get_logger().info('=' * 80)
        else:
            self.get_logger().error(f'❌ Failed to load map: {message}')
        
        return success, message
    
    def get_map_yaml_path(self):
        """获取加载的地图YAML路径"""
        return self._map_yaml_path


def main(args=None):
    rclpy.init(args=args)
    
    node = MapLoaderNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Received interrupt signal, shutting down...')
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
