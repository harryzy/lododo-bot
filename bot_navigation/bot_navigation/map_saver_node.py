#!/usr/bin/env python3
"""
地图保存节点 / Map Saver Node

功能:
- 监听探索完成信号 (/exploration/complete)
- 自动调用地图库管理器保存地图
- 提供手动保存服务

Author: GitHub Copilot
Date: 2025-12-24
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from std_srvs.srv import Trigger

from .map_library_manager import MapLibraryManager


class MapSaverNode(Node):
    """
    地图保存节点 / Map saver node
    
    监听探索完成信号并自动保存地图到地图库
    """
    
    def __init__(self):
        super().__init__('map_saver_node')
        
        self.get_logger().info('Initializing MapSaverNode...')
        
        # ===== 参数 =====
        self.declare_parameters(
            namespace='',
            parameters=[
                ('map_name', 'auto_explored_map'),
                ('description', 'Automatically explored map'),
                ('tags', 'auto,exploration'),
                ('auto_save', True),
                ('library_path', '~/lododo_bot/maps'),
            ]
        )
        
        self._map_name = self.get_parameter('map_name').value
        self._description = self.get_parameter('description').value
        self._tags_str = self.get_parameter('tags').value
        self._tags = [t.strip() for t in self._tags_str.split(',') if t.strip()]
        self._auto_save = self.get_parameter('auto_save').value
        self._library_path = self.get_parameter('library_path').value
        
        # ===== 地图库管理器 =====
        self._map_library = MapLibraryManager(
            node=self,
            library_path=self._library_path
        )
        
        # ===== 状态 =====
        self._exploration_completed = False
        self._map_saved = False
        
        # ===== 订阅探索完成信号 =====
        self._complete_sub = self.create_subscription(
            Bool,
            '/exploration/complete',
            self._exploration_complete_callback,
            10
        )
        
        # ===== 服务 =====
        self._save_srv = self.create_service(
            Trigger,
            '/map_library/save_current_map',
            self._handle_save_map
        )
        
        self.get_logger().info('MapSaverNode initialized successfully')
        self.get_logger().info(f'  - Map name: {self._map_name}')
        self.get_logger().info(f'  - Description: {self._description}')
        self.get_logger().info(f'  - Tags: {self._tags}')
        self.get_logger().info(f'  - Auto save: {self._auto_save}')
        self.get_logger().info(f'  - Library path: {self._library_path}')
    
    def _exploration_complete_callback(self, msg: Bool):
        """
        探索完成回调 / Exploration complete callback
        
        Args:
            msg: 完成信号
        """
        if not msg.data:
            return
        
        if self._exploration_completed:
            return  # 已经处理过
        
        self._exploration_completed = True
        self.get_logger().info('🎉 Exploration completed!')
        
        if self._auto_save and not self._map_saved:
            self.get_logger().info('⏳ Auto-saving map to library...')
            self._save_map()
    
    def _handle_save_map(self, request, response):
        """
        处理手动保存地图请求 / Handle manual save map request
        
        Args:
            request: 触发请求
            response: 触发响应
        
        Returns:
            响应对象
        """
        self.get_logger().info('📝 Manual save map requested')
        
        success, message = self._save_map()
        
        response.success = success
        response.message = message
        return response
    
    def _save_map(self):
        """
        保存地图到地图库 / Save map to library
        
        Returns:
            (success, message)
        """
        if self._map_saved:
            message = 'Map already saved'
            self.get_logger().warn(message)
            return True, message
        
        self.get_logger().info(f'💾 Saving map: {self._map_name}')
        
        # 调用地图库管理器保存地图
        success, message = self._map_library.save_map(
            map_name=self._map_name,
            map_topic='/map',
            description=self._description,
            tags=self._tags
        )
        
        if success:
            self._map_saved = True
            self.get_logger().info(f'✅ {message}')
        else:
            self.get_logger().error(f'❌ {message}')
        
        return success, message


def main(args=None):
    rclpy.init(args=args)
    
    node = MapSaverNode()
    
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
