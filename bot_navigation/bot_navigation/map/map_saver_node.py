#!/usr/bin/env python3
"""
地图保存节点 / Map Saver Node

功能:
- 监听探索完成信号 (/exploration/complete)
- 自动调用地图库管理器保存地图
- 保存RTABMap数据库文件
- 提供手动保存服务

Author: GitHub Copilot
Date: 2025-12-24
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool
from std_srvs.srv import Trigger
import os
import shutil
import time

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
        
        # 调用地图库管理器保存地图（占据栅格地图）
        success, message = self._map_library.save_map(
            map_name=self._map_name,
            map_topic='/map',
            description=self._description,
            tags=self._tags
        )
        
        if not success:
            self.get_logger().error(f'❌ Failed to save occupancy grid map: {message}')
            return success, message
        
        self.get_logger().info(f'✅ Occupancy grid map saved: {message}')
        
        # 保存RTABMap数据库文件
        db_success, db_message = self._save_rtabmap_database()
        
        if db_success:
            self.get_logger().info(f'✅ RTABMap database saved: {db_message}')
            self._map_saved = True
            final_message = f'{message}; {db_message}'
            return True, final_message
        else:
            self.get_logger().warn(f'⚠️  RTABMap database save warning: {db_message}')
            self._map_saved = True
            final_message = f'{message}; RTABMap DB warning: {db_message}'
            return True, final_message  # 占据栅格地图已保存，即使DB保存失败也返回成功
    
    def _save_rtabmap_database(self):
        """
        保存RTABMap数据库到地图库目录 / Save RTABMap database to map library
        
        ⚠️ 重要：在复制数据库前，先触发RTABMap将内存中的数据（包括词典）写入数据库
        
        Returns:
            (success, message)
        """
        # ===== 步骤1: 触发RTABMap数据保存 =====
        self.get_logger().info('📤 Triggering RTABMap to flush data to database...')
        
        # 调用RTABMap的backup service来强制保存内存中的数据（包括词典）到数据库
        try:
            from std_srvs.srv import Empty
            
            # 创建backup service客户端
            backup_client = self.create_client(Empty, '/rtabmap/backup')
            
            # 等待service可用（最多5秒）
            if not backup_client.wait_for_service(timeout_sec=5.0):
                self.get_logger().error('❌ RTABMap backup service not available!')
                return False, 'RTABMap backup service not available'
            
            # 调用backup service
            self.get_logger().info('📞 Calling /rtabmap/backup service...')
            request = Empty.Request()
            future = backup_client.call_async(request)
            
            # 等待backup完成
            rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)
            
            if future.result() is not None:
                self.get_logger().info('✅ RTABMap backup completed successfully!')
            else:
                self.get_logger().error('❌ RTABMap backup service call failed!')
                return False, 'RTABMap backup service call failed'
                
        except Exception as e:
            self.get_logger().error(f'❌ Failed to call RTABMap backup: {str(e)}')
            return False, f'Failed to call RTABMap backup: {str(e)}'
        
        # ⚠️ 关键：等待RTABMap完成数据写入到磁盘
        # 即使backup service返回，数据库文件可能还在写入
        self.get_logger().info('⏳ Waiting for RTABMap to complete database write to disk...')
        time.sleep(2.0)  # 等待数据完全写入磁盘
        
        # ===== 步骤2: 复制数据库文件 =====
        # RTABMap默认数据库路径
        default_db_path = os.path.expanduser('~/.ros/rtabmap.db')
        
        # 目标地图目录
        library_path = os.path.expanduser(self._library_path)
        map_dir = os.path.join(library_path, self._map_name)
        
        if not os.path.exists(map_dir):
            return False, f'Map directory not found: {map_dir}'
        
        # 检查RTABMap数据库是否存在
        if not os.path.exists(default_db_path):
            return False, f'RTABMap database not found at {default_db_path}'
        
        # 等待RTABMap完成写入（避免文件未完全写入）
        time.sleep(1.0)
        
        # 复制数据库文件到地图目录
        target_db_path = os.path.join(map_dir, 'rtabmap.db')
        
        try:
            shutil.copy2(default_db_path, target_db_path)
            
            # 验证文件大小
            src_size = os.path.getsize(default_db_path)
            dst_size = os.path.getsize(target_db_path)
            
            if src_size != dst_size:
                return False, f'File size mismatch: {src_size} != {dst_size}'
            
            self.get_logger().info(f'RTABMap database copied: {target_db_path} ({src_size} bytes)')
            
            # ⚠️ 关键步骤：检查并修复Word表
            # 如果Word表为空但Feature表有数据，说明词典还在内存中没有保存
            # 我们需要提醒用户或者尝试修复
            word_count = self._check_word_table(target_db_path)
            if word_count == 0:
                self.get_logger().warn(
                    '⚠️  Word table is empty! This means the vocabulary dictionary was not saved.'
                )
                self.get_logger().warn(
                    '   Localization will use brute-force feature matching instead of dictionary.'
                )
                self.get_logger().warn(
                    '   This is acceptable but slower. For better performance, ensure RTABMap'
                )
                self.get_logger().warn(
                    '   shuts down gracefully (wait for it to finish) before saving the map.'
                )
            else:
                self.get_logger().info(f'✅ Word table contains {word_count} entries')
            
            return True, f'RTABMap database saved ({src_size / 1024 / 1024:.2f} MB, {word_count} words)'
            
        except Exception as e:
            return False, f'Failed to copy RTABMap database: {str(e)}'
    
    def _check_word_table(self, db_path):
        """
        检查Word表中的记录数 / Check Word table entry count
        
        Args:
            db_path: 数据库文件路径
        
        Returns:
            Word表中的记录数
        """
        try:
            import subprocess
            result = subprocess.run(
                ['sqlite3', db_path, 'SELECT COUNT(*) FROM Word;'],
                capture_output=True,
                text=True,
                timeout=5.0
            )
            if result.returncode == 0:
                count = int(result.stdout.strip())
                return count
            else:
                self.get_logger().warn(f'Failed to query Word table: {result.stderr}')
                return -1
        except Exception as e:
            self.get_logger().warn(f'Failed to check Word table: {str(e)}')
            return -1


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
