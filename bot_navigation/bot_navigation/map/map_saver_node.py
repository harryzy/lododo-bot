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
        
        ⚠️ 简化策略：
        1. 调用 /rtabmap/backup (触发数据库写入)
        2. 等待5秒让RTABMap完成词典写入
        3. 复制数据库文件
        
        Note: pause/resume在timer callback中会死锁，而且RTABMap持续运行反而能保证数据完整性
        
        Returns:
            (success, message)
        """
        from std_srvs.srv import Empty
        
        # ===== 步骤1: 调用backup service =====
        self.get_logger().info('💾 Calling RTABMap backup to flush database...')
        
        try:
            backup_client = self.create_client(Empty, '/rtabmap/backup')
            
            if backup_client.wait_for_service(timeout_sec=3.0):
                self.get_logger().info('📞 Calling /rtabmap/backup...')
                request = Empty.Request()
                backup_client.call_async(request)  # 异步调用，不等待结果
                self.get_logger().info('✅ Backup request sent')
            else:
                self.get_logger().warn('⚠️  Backup service not available (continuing anyway)')
        except Exception as e:
            self.get_logger().warn(f'⚠️  Backup call error: {str(e)} (continuing anyway)')
        
        # ⚠️ 关键：等待足够时间让RTABMap完成写入
        # RTABMap会在后台持续保存数据，包括Word表（词典）
        # 测试显示5秒足够让1400+词条写入磁盘
        wait_time = 5.0
        self.get_logger().info(f'⏳ Waiting {wait_time}s for database write (including Word table)...')
        time.sleep(wait_time)
        
        # ===== 步骤3: 复制数据库文件 =====
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
        
        # 复制数据库文件到地图目录
        target_db_path = os.path.join(map_dir, 'rtabmap.db')
        
        copy_success = False
        try:
            shutil.copy2(default_db_path, target_db_path)
            
            # 验证文件大小
            src_size = os.path.getsize(default_db_path)
            dst_size = os.path.getsize(target_db_path)
            
            if src_size != dst_size:
                self.get_logger().error(f'❌ File size mismatch: {src_size} != {dst_size}')
                copy_success = False
            else:
                self.get_logger().info(f'📋 Database copied: {target_db_path} ({src_size/1024/1024:.2f} MB)')
                copy_success = True
            
            # ⚠️ 关键步骤：检查Word表
            word_count = self._check_word_table(target_db_path)
            if word_count == 0:
                self.get_logger().warn('⚠️  Word table is EMPTY!')
                self.get_logger().warn('   Possible reasons:')
                self.get_logger().warn('   1. Mapping session too short (< 50 nodes)')
                self.get_logger().warn('   2. Kp/IncrementalDictionary: false in mapping config')
                self.get_logger().warn('   Localization will use brute-force matching (slower but works)')
            elif word_count > 0:
                self.get_logger().info(f'✅ Word table verified: {word_count} entries')
            
            result_msg = f'Database saved: {src_size/1024/1024:.2f}MB, {word_count} words'
            return True, result_msg
            
        except Exception as e:
            return False, f'Failed to copy database: {str(e)}'
    
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
