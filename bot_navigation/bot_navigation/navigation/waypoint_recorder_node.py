#!/usr/bin/env python3
"""
waypoint_recorder_node.py - 路点录制节点

提供交互式 CLI 界面用于录制和管理路点
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from visualization_msgs.msg import MarkerArray
from nav_msgs.msg import Odometry
from bot_navigation_msgs.srv import WaypointControl
from std_srvs.srv import Trigger
from .waypoint_recorder import WaypointRecorder
import threading
import math,os


class WaypointRecorderNode(Node):
    """路点录制节点 - 交互式 CLI"""
    
    def __init__(self):
        super().__init__('waypoint_recorder_node')
        
        self.get_logger().info('Initializing WaypointRecorderNode...')
        
        # 参数配置
        self.declare_parameters(
            namespace='',
            parameters=[
                ('pose_topic', '/localization_pose'),
                ('backup_pose_topic', '/odom'),
                ('use_odom_backup', True),
                ('recording_interval', 1.0),
                ('min_distance', 0.5),
                ('frame_id', 'map'),
                ('persistence_dir', ''),
            ]
        )
        
        self._pose_topic = self.get_parameter('pose_topic').value
        self._backup_pose_topic = self.get_parameter('backup_pose_topic').value
        self._use_odom_backup = self.get_parameter('use_odom_backup').value
        self._recording_interval = self.get_parameter('recording_interval').value
        self._min_distance = self.get_parameter('min_distance').value
        self._frame_id = self.get_parameter('frame_id').value
        self._persistence_dir = self.get_parameter('persistence_dir').value
        
        # 展开路径中的~符号
        if self._persistence_dir:
            self._persistence_dir = os.path.expanduser(self._persistence_dir)
        
        # 路点录制器
        # 如果指定了persistence_dir参数，则使用；否则使用默认路径
        if self._persistence_dir:
            self._recorder = WaypointRecorder(persistence_dir=self._persistence_dir)
        else:
            self._recorder = WaypointRecorder()
        
        # 当前位姿
        self._current_pose: PoseStamped = None
        self._pose_received = False
        
        # 订阅位姿话题（先尝试PoseWithCovarianceStamped，如果话题不存在再尝试PoseStamped）
        try:
            # RTABMap localization 发布 PoseWithCovarianceStamped
            from geometry_msgs.msg import PoseWithCovarianceStamped
            self._pose_sub = self.create_subscription(
                PoseWithCovarianceStamped,
                self._pose_topic,
                self._pose_with_cov_callback,
                10
            )
            self.get_logger().info(f'Subscribed to {self._pose_topic} (PoseWithCovarianceStamped)')
        except Exception as e:
            # 如果话题类型不匹配，尝试PoseStamped
            self._pose_sub = self.create_subscription(
                PoseStamped,
                self._pose_topic,
                self._pose_callback,
                10
            )
            self.get_logger().info(f'Subscribed to {self._pose_topic} (PoseStamped)')
        
        # 备用：订阅里程计
        if self._use_odom_backup:
            self._odom_sub = self.create_subscription(
                Odometry,
                self._backup_pose_topic,
                self._odom_callback,
                10
            )
        
        # 发布可视化标记
        self._marker_pub = self.create_publisher(
            MarkerArray,
            '/waypoint_markers',
            10
        )
        
        # 定时器（用于自动录制）
        self._auto_record_timer = self.create_timer(
            0.1,  # 100ms 检查一次
            self._auto_record_callback
        )
        
        # 可视化更新定时器
        self._viz_timer = self.create_timer(
            1.0,  # 1秒更新一次
            self._publish_markers
        )
        
        # ROS2 服务接口（供 MissionPlanner 调用）
        self._save_srv = self.create_service(
            WaypointControl,
            '/waypoint_recorder/save_waypoints',
            lambda req, res: service_handle_save(self, req, res)
        )
        self._load_srv = self.create_service(
            WaypointControl,
            '/waypoint_recorder/load_waypoints',
            lambda req, res: service_handle_load(self, req, res)
        )
        self._record_current_srv = self.create_service(
            Trigger,
            '/waypoint_recorder/record_current',
            lambda req, res: service_handle_record_current(self, req, res)
        )
        self._start_recording_srv = self.create_service(
            Trigger,
            '/waypoint_recorder/start_recording',
            lambda req, res: service_handle_start_recording(self, req, res)
        )
        self._stop_recording_srv = self.create_service(
            Trigger,
            '/waypoint_recorder/stop_recording',
            lambda req, res: service_handle_stop_recording(self, req, res)
        )
        self._clear_waypoints_srv = self.create_service(
            Trigger,
            '/waypoint_recorder/clear_waypoints',
            lambda req, res: service_handle_clear(self, req, res)
        )
        
        self.get_logger().info('WaypointRecorderNode initialized')
        self.get_logger().info(f'Listening to pose topic: {self._pose_topic}')
        
    def _pose_with_cov_callback(self, msg):
        """位姿话题回调 (PoseWithCovarianceStamped)"""
        # 转换为 PoseStamped
        pose = PoseStamped()
        pose.header = msg.header
        pose.pose = msg.pose.pose
        self._current_pose = pose
        self._pose_received = True
    
    def _pose_callback(self, msg: PoseStamped):
        """位姿话题回调 (PoseStamped)"""
    def _odom_callback(self, msg: Odometry):
        """里程计话题回调（备用）"""
        if not self._pose_received:
            # 转换 Odometry 到 PoseStamped
            pose = PoseStamped()
            pose.header = msg.header
            pose.pose = msg.pose.pose
            self._current_pose = pose
    
    def _auto_record_callback(self):
        """自动录制回调"""
        if self._recorder.is_recording():
            if self._current_pose is None:
                # 没收到pose，每5秒提醒一次
                if not hasattr(self, '_last_pose_warning_time'):
                    self._last_pose_warning_time = 0
                current_time = self.get_clock().now().nanoseconds / 1e9
                if current_time - self._last_pose_warning_time > 5.0:
                    self.get_logger().warn('⚠ Auto-recording active but no pose received yet')
                    self._last_pose_warning_time = current_time
            else:
                current_time = self.get_clock().now().nanoseconds / 1e9
                result = self._recorder.add_waypoint(self._current_pose, current_time)
                if result['added']:
                    count = self._recorder.get_waypoint_count()
                    x = self._current_pose.pose.position.x
                    y = self._current_pose.pose.position.y
                    self.get_logger().info(f'✓ Auto-recorded waypoint #{count} at ({x:.2f}, {y:.2f})')
                elif result['reason'] and self._recorder.get_waypoint_count() == 0:
                    # 第一个路点如果失败，输出原因
                    if not hasattr(self, '_first_waypoint_debug_shown'):
                        self.get_logger().info(f'Debug: {result["reason"]}')
                        self._first_waypoint_debug_shown = True
    
    def _publish_markers(self):
        """发布可视化标记"""
        markers = self._recorder.create_visualization_markers()
        self._marker_pub.publish(markers)
    
    def record_current_pose(self):
        """手动录制当前位姿"""
        if self._current_pose is None:
            self.get_logger().warn('No pose available yet')
            return False
        
        self._recorder.add_waypoint_manual(self._current_pose)
        count = self._recorder.get_waypoint_count()
        x = self._current_pose.pose.position.x
        y = self._current_pose.pose.position.y
        self.get_logger().info(f'✓ Manually recorded waypoint #{count} at ({x:.2f}, {y:.2f})')
        return True
    
    def start_recording(self):
        """开始自动录制"""
        current_count = self._recorder.get_waypoint_count()
        self._recorder.start_recording(
            interval=self._recording_interval,
            min_distance=self._min_distance,
            clear_existing=False  # 不清空已有路点
        )
        self.get_logger().info(
            f'Started auto-recording (interval={self._recording_interval}s, '
            f'min_distance={self._min_distance}m). Current waypoints: {current_count}'
        )
        self.get_logger().info('Move the robot to start recording waypoints...')
    
    def stop_recording(self):
        """停止自动录制"""
        count = self._recorder.stop_recording()
        self.get_logger().info(f'Stopped recording. Total waypoints: {count}')
        return count
    
    def delete_last(self):
        """删除最后一个路点"""
        count = self._recorder.get_waypoint_count()
        if count > 0:
            self._recorder.remove_waypoint(count - 1)
            self.get_logger().info(f'Deleted last waypoint. Remaining: {count - 1}')
            return True
        else:
            self.get_logger().warn('No waypoints to delete')
            return False
    
    def clear_all(self):
        """清空所有路点"""
        self._recorder.clear_waypoints()
        self.get_logger().info('Cleared all waypoints')
    
    def save_waypoints(self, filename: str):
        """保存路点到文件"""
        # 检查是否有路点
        count = self._recorder.get_waypoint_count()
        if count == 0:
            self.get_logger().error('✗ Failed to save: No waypoints recorded yet. Use "r" to record or "a" to start auto-recording.')
            return False
        
        # WaypointRecorder会自动添加.yaml扩展名，这里不需要添加
        if self._recorder.save_waypoints(filename):
            # 构建完整保存路径（展开~）
            save_dir = os.path.expanduser(self._persistence_dir) if self._persistence_dir else os.path.expanduser('~/.ros/lekiwi_bot/navigation/waypoints')
            save_path = os.path.join(save_dir, filename if filename.endswith('.yaml') else filename + '.yaml')
            self.get_logger().info(f'✓ Saved {count} waypoints to: {save_path}')
            return True
        else:
            self.get_logger().error(f'✗ Failed to save waypoints to file')
            return False
    
    def load_waypoints(self, filename: str):
        """从文件加载路点"""
        # WaypointRecorder会自动添加.yaml扩展名
        if self._recorder.load_waypoints(filename):
            count = self._recorder.get_waypoint_count()
            save_dir = os.path.expanduser(self._persistence_dir) if self._persistence_dir else os.path.expanduser('~/.ros/lekiwi_bot/navigation/waypoints')
            save_path = os.path.join(save_dir, filename if filename.endswith('.yaml') else filename + '.yaml')
            self.get_logger().info(f'✓ Loaded {count} waypoints from: {save_path}')
            return True
        else:
            self.get_logger().error(f'✗ Failed to load waypoints from: {filename}')
            return False
    
    def list_waypoints(self):
        """列出当前所有路点"""
        waypoints = self._recorder.get_waypoints()
        count = len(waypoints)
        
        if count == 0:
            self.get_logger().info('No waypoints recorded')
            return
        
        self.get_logger().info(f'\n===== Waypoints ({count}) =====')
        for i, wp in enumerate(waypoints):
            # 计算 yaw 角度
            q = wp.pose.orientation
            yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                           1.0 - 2.0 * (q.y * q.y + q.z * q.z))
            
            self.get_logger().info(
                f'  [{i+1}] x={wp.pose.position.x:.3f}, '
                f'y={wp.pose.position.y:.3f}, yaw={yaw:.3f} rad'
            )
        self.get_logger().info('=' * 30)
    
    def list_saved_files(self):
        """列出所有保存的路点文件"""
        files = self._recorder.list_saved_waypoints()
        
        if len(files) == 0:
            self.get_logger().info('No saved waypoint files')
            return
        
        self.get_logger().info(f'\n===== Saved Files ({len(files)}) =====')
        for i, filename in enumerate(files):
            self.get_logger().info(f'  [{i+1}] {filename}')
        self.get_logger().info('=' * 30)
    
    def show_current_pose(self):
        """显示当前位姿"""
        if self._current_pose is None:
            self.get_logger().info('No pose available yet')
            return
        
        q = self._current_pose.pose.orientation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                       1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        
        self.get_logger().info(
            f'Current Pose: x={self._current_pose.pose.position.x:.3f}, '
            f'y={self._current_pose.pose.position.y:.3f}, yaw={yaw:.3f} rad'
        )


def print_menu():
    """打印菜单"""
    print('\n' + '='*50)
    print('LeKiwi 路点录制器 / Waypoint Recorder')
    print('='*50)
    print('命令 / Commands:')
    print('  r - 录制当前位置 / Record current position')
    print('  a - 开始自动录制 / Start auto recording')
    print('  t - 停止自动录制 / Stop auto recording')
    print('  d - 删除最后一个路点 / Delete last waypoint')
    print('  c - 清空所有路点 / Clear all waypoints')
    print('  l - 列出所有路点 / List all waypoints')
    print('  p - 显示当前位姿 / Show current pose')
    print('  s - 保存路点到文件 / Save waypoints')
    print('  o - 从文件加载路点 / Load waypoints')
    print('  f - 列出已保存文件 / List saved files')
    print('  h - 显示帮助 / Show help')
    print('  q - 退出 / Quit')
    print('='*50)


def run_interactive_cli(node: WaypointRecorderNode):
    """运行交互式 CLI"""
    import threading
    
    print_menu()
    
    # CLI 在单独线程运行（避免阻塞 ROS2 回调）
    def cli_thread():
        while rclpy.ok():
            try:
                # 阻塞式读取用户输入
                command = input('\n[%d waypoints] > ' % node._recorder.get_waypoint_count()).strip().lower()
                
                if command == 'r':
                    node.record_current_pose()
                
                elif command == 'a':
                    node.start_recording()
                
                elif command == 't':
                    node.stop_recording()
                
                elif command == 'd':
                    node.delete_last()
                
                elif command == 'c':
                    confirm = input('确认清空所有路点? (y/n): ')
                    if confirm.lower() == 'y':
                        node.clear_all()
                
                elif command == 'l':
                    node.list_waypoints()
                
                elif command == 'p':
                    node.show_current_pose()
                
                elif command == 's':
                    filename = input('输入文件名 (不含扩展名): ')
                    if filename:
                        node.save_waypoints(filename)
                
                elif command == 'o':
                    filename = input('输入文件名 (不含扩展名): ')
                    if filename:
                        node.load_waypoints(filename)
                
                elif command == 'f':
                    node.list_saved_files()
                
                elif command == 'h':
                    print_menu()
                
                elif command == 'q':
                    print('退出...')
                    rclpy.shutdown()
                    break
                
                elif command:
                    print(f'未知命令: {command}. 输入 h 查看帮助')
                
            except KeyboardInterrupt:
                print('\n退出...')
                rclpy.shutdown()
                break
            except EOFError:
                # 处理 Ctrl+D
                print('\n退出...')
                rclpy.shutdown()
                break
            except Exception as e:
                node.get_logger().error(f'CLI Error: {str(e)}')
    
    # 启动 CLI 线程
    cli_worker = threading.Thread(target=cli_thread, daemon=True)
    cli_worker.start()
    
    # 主线程运行 ROS2 spin（保证回调能正常执行）
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


# ========== ROS2 服务处理函数（独立于交互式CLI） ==========

def service_handle_save(node, request, response):
    """处理保存路点请求"""
    try:
        filename = request.filename if request.filename else 'waypoints'
        # 扩展名由 waypoint_recorder 自动添加
        success = node._recorder.save_waypoints(filename)
        response.success = success
        response.message = f"Saved {node._recorder.get_waypoint_count()} waypoints to {filename}.yaml" if success else "Save failed"
        response.waypoint_count = node._recorder.get_waypoint_count()
        node.get_logger().info(f"Service: Save waypoints to {filename}, result: {success}")
    except Exception as e:
        response.success = False
        response.message = f"Error: {str(e)}"
        response.waypoint_count = 0
        node.get_logger().error(f"Save service error: {str(e)}")
    return response

def service_handle_load(node, request, response):
    """处理加载路点请求"""
    try:
        filename = request.filename if request.filename else 'waypoints'
        # 扩展名由 waypoint_recorder 自动添加
        success = node._recorder.load_waypoints(filename)
        response.success = success
        response.message = f"Loaded {node._recorder.get_waypoint_count()} waypoints from {filename}.yaml" if success else "Load failed"
        response.waypoint_count = node._recorder.get_waypoint_count()
        node.get_logger().info(f"Service: Load waypoints from {filename}, result: {success}")
    except Exception as e:
        response.success = False
        response.message = f"Error: {str(e)}"
        response.waypoint_count = 0
        node.get_logger().error(f"Load service error: {str(e)}")
    return response

def service_handle_record_current(node, request, response):
    """处理录制当前位置请求"""
    try:
        if node._current_pose is None:
            response.success = False
            response.message = "No pose available"
        else:
            node._recorder.add_waypoint_manual(node._current_pose)
            count = node._recorder.get_waypoint_count()
            response.success = True
            response.message = f"Recorded waypoint #{count}"
            node.get_logger().info(f"Service: Recorded current pose, total: {count}")
    except Exception as e:
        response.success = False
        response.message = f"Error: {str(e)}"
        node.get_logger().error(f"Record current service error: {str(e)}")
    return response

def service_handle_start_recording(node, request, response):
    """处理开始自动录制请求"""
    try:
        node._recorder.start_recording()
        response.success = True
        response.message = f"Started recording (interval={node._recording_interval}s, min_dist={node._min_distance}m)"
        node.get_logger().info("Service: Started auto recording")
    except Exception as e:
        response.success = False
        response.message = f"Error: {str(e)}"
        node.get_logger().error(f"Start recording service error: {str(e)}")
    return response

def service_handle_stop_recording(node, request, response):
    """处理停止录制请求"""
    try:
        node._recorder.stop_recording()
        count = node._recorder.get_waypoint_count()
        response.success = True
        response.message = f"Stopped recording. Total waypoints: {count}"
        node.get_logger().info(f"Service: Stopped recording, total: {count}")
    except Exception as e:
        response.success = False
        response.message = f"Error: {str(e)}"
        node.get_logger().error(f"Stop recording service error: {str(e)}")
    return response

def service_handle_clear(node, request, response):
    """处理清空路点请求"""
    try:
        node._recorder.clear_waypoints()
        response.success = True
        response.message = "All waypoints cleared"
        node.get_logger().info("Service: Cleared all waypoints")
    except Exception as e:
        response.success = False
        response.message = f"Error: {str(e)}"
        node.get_logger().error(f"Clear service error: {str(e)}")
    return response


def main(args=None):
    rclpy.init(args=args)
    
    node = WaypointRecorderNode()
    
    # 在单独的线程中运行交互式 CLI
    try:
        run_interactive_cli(node)
    except Exception as e:
        node.get_logger().error(f'Fatal error: {str(e)}')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
