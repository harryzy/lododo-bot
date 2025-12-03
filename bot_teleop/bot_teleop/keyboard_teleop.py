#!/usr/bin/env python3
"""
键盘遥控节点
支持全向移动控制
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import sys
import termios
import tty


HELP_MSG = """
LeKiwi Robot Keyboard Control / LeKiwi机器人键盘控制
---------------------------
Movement Control / 移动控制:
   w/s: Forward/Backward / 前进/后退
   a/d: Left/Right / 左移/右移
   q/e: Turn Left/Turn Right / 左转/右转
   
   i: Forward+Left / 前进+左移
   o: Forward+Right / 前进+右移
   k: Backward+Left / 后退+左移
   l: Backward+Right / 后退+右移

Speed Control / 速度控制:
   z: Increase Linear Speed / 增加线速度
   x: Decrease Linear Speed / 减少线速度
   c: Increase Angular Speed / 增加角速度
   v: Decrease Angular Speed / 减少角速度

Other / 其他:
   space: Emergency Stop / 急停
   h: Show Help / 显示帮助
   Ctrl+C: Exit / 退出
---------------------------
Current Speed / 当前速度: Linear={:.2f} m/s, Angular={:.2f} rad/s
"""


class KeyboardTeleop(Node):
    """键盘遥控节点"""

    def __init__(self):
        super().__init__('keyboard_teleop')
        
        # 速度参数
        self.linear_speed = 0.1   # 线速度 (m/s)
        self.angular_speed = 0.5  # 角速度 (rad/s)
        self.speed_step = 0.05    # 速度增量
        
        # 最大速度限制
        self.max_linear_speed = 0.20  # m/s
        self.max_angular_speed = 1.5  # rad/s
        
        # 发布cmd_vel
        self.cmd_vel_pub = self.create_publisher(Twist, 'cmd_vel', 10)
        
        # 保存终端设置
        self.settings = termios.tcgetattr(sys.stdin)
        
        self.get_logger().info('Keyboard teleop node started')
        self.print_help()

    def print_help(self):
        """打印帮助信息"""
        print(HELP_MSG.format(self.linear_speed, self.angular_speed))

    def get_key(self):
        """获取键盘输入"""
        tty.setraw(sys.stdin.fileno())
        key = sys.stdin.read(1)
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
        return key

    def run(self):
        """主循环"""
        try:
            while rclpy.ok():
                key = self.get_key()
                
                twist = Twist()
                
                # 线速度控制
                if key == 'w':  # 前进
                    twist.linear.x = self.linear_speed
                elif key == 's':  # 后退
                    twist.linear.x = -self.linear_speed
                elif key == 'a':  # 左移
                    twist.linear.y = self.linear_speed
                elif key == 'd':  # 右移
                    twist.linear.y = -self.linear_speed
                
                # 对角线移动
                elif key == 'i':  # 前进+左移
                    twist.linear.x = self.linear_speed
                    twist.linear.y = self.linear_speed
                elif key == 'o':  # 前进+右移
                    twist.linear.x = self.linear_speed
                    twist.linear.y = -self.linear_speed
                elif key == 'k':  # 后退+左移
                    twist.linear.x = -self.linear_speed
                    twist.linear.y = self.linear_speed
                elif key == 'l':  # 后退+右移
                    twist.linear.x = -self.linear_speed
                    twist.linear.y = -self.linear_speed
                
                # 角速度控制
                elif key == 'q':  # 左转
                    twist.angular.z = self.angular_speed
                elif key == 'e':  # 右转
                    twist.angular.z = -self.angular_speed
                
                # 速度调整
                elif key == 'z':  # 增加线速度
                    self.linear_speed = min(
                        self.linear_speed + self.speed_step,
                        self.max_linear_speed
                    )
                    print(f'Linear speed: {self.linear_speed:.2f} m/s')
                elif key == 'x':  # 减少线速度
                    self.linear_speed = max(
                        self.linear_speed - self.speed_step,
                        0.0
                    )
                    print(f'Linear speed: {self.linear_speed:.2f} m/s')
                elif key == 'c':  # 增加角速度
                    self.angular_speed = min(
                        self.angular_speed + self.speed_step,
                        self.max_angular_speed
                    )
                    print(f'Angular speed: {self.angular_speed:.2f} rad/s')
                elif key == 'v':  # 减少角速度
                    self.angular_speed = max(
                        self.angular_speed - self.speed_step,
                        0.0
                    )
                    print(f'Angular speed: {self.angular_speed:.2f} rad/s')
                
                # 急停
                elif key == ' ':
                    twist = Twist()  # 全零
                    print('Emergency stop!')
                
                # 帮助
                elif key == 'h':
                    self.print_help()
                
                # 退出
                elif key == '\x03':  # Ctrl+C
                    break
                
                # 发布速度命令
                self.cmd_vel_pub.publish(twist)
                
        except Exception as e:
            self.get_logger().error(f'Error: {e}')
        finally:
            # 恢复终端设置
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, self.settings)
            # 发送停止命令
            self.cmd_vel_pub.publish(Twist())


def main(args=None):
    rclpy.init(args=args)
    teleop = KeyboardTeleop()
    
    try:
        teleop.run()
    except KeyboardInterrupt:
        pass
    finally:
        teleop.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
