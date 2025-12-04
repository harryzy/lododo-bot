#!/usr/bin/env python3
"""
深度图转激光雷达扫描节点 / Depth to LaserScan Conversion Node
将RGB-D相机的深度图像转换为2D激光雷达数据，用于导航
Converts RGB-D camera depth image to 2D laser scan data for navigation
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy, QoSDurabilityPolicy
from sensor_msgs.msg import Image, LaserScan
from cv_bridge import CvBridge
import numpy as np
import math


class DepthToLaserScan(Node):
    """将深度图像转换为激光雷达扫描数据 / Convert depth image to laser scan data"""
    
    def __init__(self):
        super().__init__('depth_to_laserscan')
        
        # ====================================================================
        # QoS 配置 / QoS Configuration
        # ====================================================================
        
        # 订阅者QoS - 使用RELIABLE以匹配Gazebo相机发布者
        # Subscriber QoS - Use RELIABLE to match Gazebo camera publisher
        # 注意：真实硬件可能需要改为BEST_EFFORT
        # Note: Real hardware may need to change to BEST_EFFORT
        sub_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=5
        )
        
        # 发布者QoS - BEST_EFFORT（模拟真实激光雷达传感器行为）
        # Publisher QoS - BEST_EFFORT (mimic real laser scanner sensor behavior)
        pub_qos_best_effort = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # 发布者QoS - RELIABLE（供Nav2 costmap使用）
        # Publisher QoS - RELIABLE (for Nav2 costmap)
        pub_qos_reliable = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.VOLATILE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # 声明参数 / Declare parameters
        self.declare_parameter('scan_height', 10)  # 扫描行数 / Number of rows to scan
        self.declare_parameter('scan_time', 0.033)  # 扫描周期（30Hz）/ Scan period
        # 最小距离：设为0.05m(5cm)以支持近距离检测，配合机械臂操作
        # Min range: set to 0.05m(5cm) for close-range detection, for arm manipulation
        self.declare_parameter('range_min', 0.05)  # 最小距离（米）
        self.declare_parameter('range_max', 8.0)  # 最大距离（米）/ Max range
        self.declare_parameter('angle_min', -0.5236)  # 最小角度（-30度，相机FOV约60度）
        self.declare_parameter('angle_max', 0.5236)  # 最大角度（30度）
        self.declare_parameter('angle_increment', 0.00436)  # 角度增量（约0.25度）
        self.declare_parameter('output_frame', 'base_link')  # 输出坐标系 - LaserScan在base_link平面
        
        # 获取参数
        self.scan_height = self.get_parameter('scan_height').value
        self.scan_time = self.get_parameter('scan_time').value
        self.range_min = self.get_parameter('range_min').value
        self.range_max = self.get_parameter('range_max').value
        self.angle_min = self.get_parameter('angle_min').value
        self.angle_max = self.get_parameter('angle_max').value
        self.angle_increment = self.get_parameter('angle_increment').value
        self.output_frame = self.get_parameter('output_frame').value
        
        # CvBridge用于图像转换 / CvBridge for image conversion
        self.bridge = CvBridge()
        
        # 订阅深度图像（使用RELIABLE QoS匹配Gazebo）
        # Subscribe to depth image (RELIABLE QoS to match Gazebo)
        self.depth_sub = self.create_subscription(
            Image,
            '/camera/depth/image_raw',
            self.depth_callback,
            sub_qos
        )
        
        # 发布激光雷达数据 - 双话题策略 / Dual-topic strategy
        # /scan: BEST_EFFORT - 模拟真实传感器行为，供RVIZ等工具使用
        # /scan: BEST_EFFORT - mimic real sensor, for RVIZ and other tools
        self.scan_pub = self.create_publisher(LaserScan, '/scan', pub_qos_best_effort)
        
        # /scan_reliable: RELIABLE - 供Nav2 costmap使用
        # /scan_reliable: RELIABLE - for Nav2 costmap
        self.scan_reliable_pub = self.create_publisher(LaserScan, '/scan_reliable', pub_qos_reliable)
        
        self.get_logger().info('Depth to LaserScan node started')
        self.get_logger().info(f'  Subscribe QoS: RELIABLE (match Gazebo camera)')
        self.get_logger().info(f'  Publish /scan: BEST_EFFORT (sensor-like)')
        self.get_logger().info(f'  Publish /scan_reliable: RELIABLE (for Nav2)')
        self.get_logger().info(f'  Range: [{self.range_min}, {self.range_max}] m')
        self.get_logger().info(f'  Angle: [{math.degrees(self.angle_min):.1f}, {math.degrees(self.angle_max):.1f}] deg')
        self.get_logger().info(f'  Scan height: {self.scan_height} pixels')
    
    def depth_callback(self, depth_msg):
        """处理深度图像并转换为激光雷达扫描"""
        try:
            # 将ROS图像消息转换为OpenCV格式
            depth_image = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding='32FC1')
            
            # 获取图像尺寸
            height, width = depth_image.shape
            
            # 计算扫描中心行
            center_row = height // 2
            row_start = max(0, center_row - self.scan_height // 2)
            row_end = min(height, center_row + self.scan_height // 2)
            
            # 提取中间区域的深度值
            depth_slice = depth_image[row_start:row_end, :]
            
            # 对每列取最小值（最近的障碍物）
            # 替换NaN和无效值
            depth_slice = np.nan_to_num(depth_slice, nan=float('inf'), posinf=float('inf'), neginf=float('inf'))
            
            # 将0值（无效深度）也设为inf
            depth_slice[depth_slice <= 0] = float('inf')
            
            min_depths = np.nanmin(depth_slice, axis=0)
            
            # 计算角度范围
            # LaserScan包含端点，所以点数 = (angle_max - angle_min) / angle_increment + 1
            # LaserScan includes endpoints, so num_readings = (angle_max - angle_min) / angle_increment + 1
            num_readings = int((self.angle_max - self.angle_min) / self.angle_increment) + 1
            
            # 从完整宽度采样到目标点数 / Sample from full width to target number of points
            if width > num_readings:
                # 下采样
                indices = np.linspace(0, width - 1, num_readings, dtype=int)
                ranges = min_depths[indices]
            elif width < num_readings:
                # 上采样（插值）
                x_old = np.arange(width)
                x_new = np.linspace(0, width - 1, num_readings)
                ranges = np.interp(x_new, x_old, min_depths)
            else:
                ranges = min_depths
            
            # 光学坐标系中，图像左边对应正角度（机器人左边），右边对应负角度
            # LaserScan 从 angle_min 到 angle_max，所以需要翻转
            ranges = ranges[::-1]
            
            # 将inf替换为range_max，用于LaserScan
            ranges = np.where(np.isinf(ranges), self.range_max, ranges)
            
            # 限制范围
            ranges = np.clip(ranges, self.range_min, self.range_max)
            
            # 创建LaserScan消息
            scan = LaserScan()
            scan.header.stamp = depth_msg.header.stamp
            scan.header.frame_id = self.output_frame
            
            scan.angle_min = self.angle_min
            scan.angle_max = self.angle_max
            scan.angle_increment = self.angle_increment
            scan.time_increment = self.scan_time / num_readings
            scan.scan_time = self.scan_time
            scan.range_min = self.range_min
            scan.range_max = self.range_max
            
            scan.ranges = ranges.astype(np.float32).tolist()
            
            # 发布激光雷达数据到两个话题 / Publish laser scan to both topics
            self.scan_pub.publish(scan)           # BEST_EFFORT
            self.scan_reliable_pub.publish(scan)  # RELIABLE
            
        except Exception as e:
            self.get_logger().error(f'Error processing depth image: {str(e)}')


def main(args=None):
    rclpy.init(args=args)
    node = DepthToLaserScan()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
