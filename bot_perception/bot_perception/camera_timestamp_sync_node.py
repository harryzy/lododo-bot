#!/usr/bin/env python3
"""
相机时间戳同步节点 - Camera Timestamp Synchronization Node
===========================================================

功能：
- 订阅color和depth图像，强制使用统一时间戳
- 解决Astra相机color/depth时间戳不同步问题（相差1-2秒）
- 使用当前ROS时间重写所有消息的header.stamp

原理：
- Astra Pro使用UVC模式时，color用系统时间，depth用设备时间
- 导致同一帧的color和depth时间戳相差1-2秒
- RTABMap的approx_sync无法匹配这种大时间差
- 解决方案：统一使用ROS当前时间重写时间戳

作者：hurry  
日期：2026-02-03
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import Image, CameraInfo
from builtin_interfaces.msg import Time


class CameraTimestampSyncNode(Node):
    """相机时间戳同步节点"""
    
    def __init__(self):
        super().__init__('camera_timestamp_sync_node')
        
        # ⚠️ CRITICAL: 使用RELIABLE QoS匹配Astra相机驱动
        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RELIABLE,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=10
        )
        
        # 订阅原始话题（使用RELIABLE QoS）
        self.sub_color = self.create_subscription(
            Image,
            '/camera/color/image_raw',
            self.color_callback,
            qos_profile
        )
        
        self.sub_depth = self.create_subscription(
            Image,
            '/camera/depth/image_raw',
            self.depth_callback,
            qos_profile
        )
        
        self.sub_color_info = self.create_subscription(
            CameraInfo,
            '/camera/color/camera_info',
            self.color_info_callback,
            qos_profile
        )
        
        self.sub_depth_info = self.create_subscription(
            CameraInfo,
            '/camera/depth/camera_info',
            self.depth_info_callback,
            qos_profile
        )
        
        # 发布同步后的话题（同样使用RELIABLE QoS）
        self.pub_color = self.create_publisher(
            Image,
            '/camera/color/image_raw_synced',
            qos_profile
        )
        
        self.pub_depth = self.create_publisher(
            Image,
            '/camera/depth/image_raw_synced',
            qos_profile
        )
        
        self.pub_color_info = self.create_publisher(
            CameraInfo,
            '/camera/color/camera_info_synced',
            qos_profile
        )
        
        self.pub_depth_info = self.create_publisher(
            CameraInfo,
            '/camera/depth/camera_info_synced',
            qos_profile
        )
        
        # 添加计数器用于调试
        self.color_count = 0
        self.depth_count = 0
        
        self.get_logger().info('Camera timestamp sync node initialized with RELIABLE QoS')
        self.get_logger().info('Subscribing to: /camera/color/image_raw, /camera/depth/image_raw')
        self.get_logger().info('Publishing to: /camera/*/image_raw_synced, /camera/*/camera_info_synced')
    
    def get_synced_timestamp(self):
        """获取统一的ROS时间戳"""
        now = self.get_clock().now()
        stamp = Time()
        stamp.sec = now.seconds_nanoseconds()[0]
        stamp.nanosec = now.seconds_nanoseconds()[1]
        return stamp
    
    def color_callback(self, msg):
        """Color图像回调 - 重写时间戳"""
        self.color_count += 1
        if self.color_count % 30 == 1:  # 每30帧打印一次
            self.get_logger().info(f'Color callback triggered ({self.color_count} frames)')
        synced_msg = msg  # 直接使用原消息（Python对象引用）
        synced_msg.header.stamp = self.get_synced_timestamp()
        self.pub_color.publish(synced_msg)
    
    def depth_callback(self, msg):
        """Depth图像回调 - 重写时间戳"""
        self.depth_count += 1
        if self.depth_count % 30 == 1:  # 每30帧打印一次
            self.get_logger().info(f'Depth callback triggered ({self.depth_count} frames)')
        synced_msg = msg
        synced_msg.header.stamp = self.get_synced_timestamp()
        self.pub_depth.publish(synced_msg)
    
    def color_info_callback(self, msg):
        """Color CameraInfo回调 - 重写时间戳"""
        synced_msg = msg
        synced_msg.header.stamp = self.get_synced_timestamp()
        self.pub_color_info.publish(synced_msg)
    
    def depth_info_callback(self, msg):
        """Depth CameraInfo回调 - 重写时间戳"""
        synced_msg = msg
        synced_msg.header.stamp = self.get_synced_timestamp()
        self.pub_depth_info.publish(synced_msg)


def main(args=None):
    rclpy.init(args=args)
    node = CameraTimestampSyncNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
