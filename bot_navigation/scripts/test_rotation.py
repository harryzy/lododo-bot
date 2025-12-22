#!/usr/bin/env python3
"""
测试原地转向功能
用于诊断Nav2配置或运动学矩阵问题
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped, Twist
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
import math
import time


class RotationTester(Node):
    """原地转向测试节点"""
    
    def __init__(self):
        super().__init__('rotation_tester')
        
        self.get_logger().info('🔄 原地转向测试节点启动')
        
        # 创建Nav2 ActionClient
        self.nav_client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        
        # 创建cmd_vel发布器（用于直接控制测试）
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        
        self.get_logger().info('等待Nav2 action服务器...')
        self.nav_client.wait_for_server()
        self.get_logger().info('✅ Nav2 action服务器已连接')
    
    def test_direct_rotation(self, angular_speed=0.5, duration=3.14):
        """
        测试方法1: 直接发送cmd_vel指令进行原地旋转
        
        Args:
            angular_speed: 角速度 (rad/s)
            duration: 持续时间 (秒)，默认3.14秒旋转约180度
        """
        self.get_logger().info(
            f'🔄 测试直接cmd_vel旋转: 角速度={angular_speed:.2f}rad/s, 持续{duration:.1f}秒'
        )
        
        twist = Twist()
        twist.linear.x = 0.0
        twist.linear.y = 0.0
        twist.angular.z = angular_speed
        
        start_time = time.time()
        
        while (time.time() - start_time) < duration:
            self.cmd_vel_pub.publish(twist)
            time.sleep(0.05)  # 20Hz
        
        # 停止
        twist.angular.z = 0.0
        self.cmd_vel_pub.publish(twist)
        self.get_logger().info(f'🛑 已停止旋转')
        
        self.get_logger().info('✅ 直接旋转测试完成')
    
    def test_nav2_rotation(self, angle_degrees=180):
        """
        测试方法2: 通过Nav2发送原地旋转目标
        
        Args:
            angle_degrees: 旋转角度（度）
        """
        self.get_logger().info(f'🔄 测试Nav2旋转: {angle_degrees}度')
        
        # 获取当前位置（简化：假设在原点）
        # 实际使用中应该从TF获取
        current_x = 0.0
        current_y = 0.0
        
        # 计算目标姿态（位置不变，只改变朝向）
        target_yaw = math.radians(angle_degrees)
        
        goal_msg = NavigateToPose.Goal()
        goal_msg.pose.header.frame_id = 'map'
        goal_msg.pose.header.stamp = self.get_clock().now().to_msg()
        goal_msg.pose.pose.position.x = current_x
        goal_msg.pose.pose.position.y = current_y
        goal_msg.pose.pose.position.z = 0.0
        
        # 四元数：yaw转换
        qz = math.sin(target_yaw / 2.0)
        qw = math.cos(target_yaw / 2.0)
        
        goal_msg.pose.pose.orientation.x = 0.0
        goal_msg.pose.pose.orientation.y = 0.0
        goal_msg.pose.pose.orientation.z = qz
        goal_msg.pose.pose.orientation.w = qw
        
        self.get_logger().info(f'发送Nav2目标: yaw={angle_degrees}°')
        
        send_goal_future = self.nav_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_goal_future)
        
        goal_handle = send_goal_future.result()
        if not goal_handle.accepted:
            self.get_logger().error('❌ Nav2目标被拒绝')
            return
        
        self.get_logger().info('✅ Nav2目标已接受，等待结果...')
        
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=30.0)
        
        result = result_future.result()
        if result:
            self.get_logger().info(f'✅ Nav2旋转完成: 状态={result.status}')
        else:
            self.get_logger().error('❌ Nav2旋转超时')


def main(args=None):
    rclpy.init(args=args)
    
    tester = RotationTester()
    
    try:
        # 等待系统稳定
        time.sleep(2.0)
        
        print("\n" + "="*60)
        print("原地转向测试程序")
        print("="*60)
        print("测试1: 直接cmd_vel旋转 (绕过Nav2，测试底层运动学)")
        print("测试2: Nav2原地旋转 (测试Nav2配置和行为树)")
        print("="*60 + "\n")
        
        while True:
            print("\n请选择测试模式:")
            print("  1 - 直接cmd_vel旋转90度 (顺时针)")
            print("  2 - 直接cmd_vel旋转90度 (逆时针)")
            print("  3 - 直接cmd_vel旋转180度")
            print("  4 - 直接cmd_vel旋转360度")
            print("  5 - 紧急停止 (cmd_vel=0)")
            print("  6 - Nav2原地旋转90度")
            print("  7 - Nav2原地旋转180度")
            print("  0 - 退出")
            
            choice = input("\n输入选项: ").strip()
            
            if choice == '1':
                tester.test_direct_rotation(angular_speed=0.5, duration=math.pi/2/0.5)
            elif choice == '2':
                tester.test_direct_rotation(angular_speed=-0.5, duration=math.pi/2/0.5)
            elif choice == '3':
                tester.test_direct_rotation(angular_speed=0.5, duration=math.pi/0.5)
            elif choice == '4':
                tester.test_direct_rotation(angular_speed=0.5, duration=2*math.pi/0.5)
            elif choice == '5':
                # 紧急停止
                twist = Twist()
                tester.cmd_vel_pub.publish(twist)
                tester.get_logger().info('🛑 已发送停止指令')
            elif choice == '6':
                tester.test_nav2_rotation(angle_degrees=90)
            elif choice == '7':
                tester.test_nav2_rotation(angle_degrees=180)
            elif choice == '0':
                break
            else:
                print("❌ 无效选项，请重试")
            
            time.sleep(1.0)
    
    except KeyboardInterrupt:
        pass
    finally:
        tester.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
