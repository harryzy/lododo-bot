#!/usr/bin/env python3
"""
URDF轮子运动学分析脚本
通过加载实际URDF，计算轮子旋转轴在base_link坐标系下的真实方向
并推导理论雅可比矩阵

Author: LeKiwi Team
Date: 2025-12-16
"""

import numpy as np
import xml.etree.ElementTree as ET
from scipy.spatial.transform import Rotation
import os
import sys
import subprocess
import tempfile


class URDFKinematicsAnalyzer:
    """URDF运动学分析器"""
    
    def __init__(self, urdf_path):
        """
        初始化分析器
        
        Args:
            urdf_path: URDF文件路径
        """
        self.urdf_path = urdf_path
        self.tree = ET.parse(urdf_path)
        self.root = self.tree.getroot()
        
        # 存储关节和连杆信息
        self.joints = {}
        self.links = {}
        
        self._parse_urdf()
    
    def _parse_urdf(self):
        """解析URDF文件"""
        # 解析所有关节
        for joint in self.root.findall('joint'):
            name = joint.get('name')
            joint_type = joint.get('type')
            
            # 获取父子连杆
            parent = joint.find('parent').get('link')
            child = joint.find('child').get('link')
            
            # 获取origin
            origin = joint.find('origin')
            if origin is not None:
                xyz = origin.get('xyz', '0 0 0').split()
                rpy = origin.get('rpy', '0 0 0').split()
                xyz = np.array([float(x) for x in xyz])
                rpy = np.array([float(x) for x in rpy])
            else:
                xyz = np.zeros(3)
                rpy = np.zeros(3)
            
            # 获取axis（仅对revolute/continuous关节）
            axis_elem = joint.find('axis')
            if axis_elem is not None:
                axis = axis_elem.get('xyz', '1 0 0').split()
                axis = np.array([float(x) for x in axis])
            else:
                axis = np.array([1, 0, 0])  # 默认X轴
            
            self.joints[name] = {
                'type': joint_type,
                'parent': parent,
                'child': child,
                'xyz': xyz,
                'rpy': rpy,
                'axis': axis
            }
    
    def get_transform_chain(self, from_link, to_link):
        """
        获取从from_link到to_link的变换链
        
        Returns:
            list: 关节列表，按顺序从from_link到to_link
        """
        # 简化版本：假设是树状结构，找到路径
        chain = []
        current = to_link
        
        # 从to_link向上找到from_link
        while current != from_link:
            found = False
            for joint_name, joint_info in self.joints.items():
                if joint_info['child'] == current:
                    chain.insert(0, joint_name)
                    current = joint_info['parent']
                    found = True
                    break
            
            if not found:
                raise ValueError(f"无法找到从 {from_link} 到 {to_link} 的路径")
        
        return chain
    
    def compute_transform(self, xyz, rpy):
        """
        计算4x4齐次变换矩阵
        
        Args:
            xyz: 平移向量 [x, y, z]
            rpy: 欧拉角 [roll, pitch, yaw]
        
        Returns:
            4x4变换矩阵
        """
        # 从RPY创建旋转矩阵
        r = Rotation.from_euler('xyz', rpy)
        rotation_matrix = r.as_matrix()
        
        # 构建齐次变换矩阵
        transform = np.eye(4)
        transform[:3, :3] = rotation_matrix
        transform[:3, 3] = xyz
        
        return transform
    
    def get_axis_in_base_link(self, wheel_joint_name):
        """
        计算轮子旋转轴在base_link坐标系下的方向
        
        Args:
            wheel_joint_name: 轮子关节名称
        
        Returns:
            dict: 包含轮子位置、旋转轴方向等信息
        """
        joint_info = self.joints[wheel_joint_name]
        
        # 获取从base_link到轮子关节的变换链
        chain = self.get_transform_chain('base_link', joint_info['parent'])
        
        # 计算累积变换
        cumulative_transform = np.eye(4)
        
        for joint_name in chain:
            j_info = self.joints[joint_name]
            T = self.compute_transform(j_info['xyz'], j_info['rpy'])
            cumulative_transform = cumulative_transform @ T
        
        # 再乘以轮子关节本身的变换
        wheel_transform = self.compute_transform(joint_info['xyz'], joint_info['rpy'])
        cumulative_transform = cumulative_transform @ wheel_transform
        
        # 提取轮子位置（在base_link坐标系中）
        wheel_position = cumulative_transform[:3, 3]
        
        # 计算轮子旋转轴在base_link坐标系中的方向
        # 轴在局部坐标系中是joint_info['axis']
        # 需要用旋转矩阵变换到base_link坐标系
        rotation_matrix = cumulative_transform[:3, :3]
        axis_in_base = rotation_matrix @ joint_info['axis']
        
        # 归一化
        axis_in_base = axis_in_base / np.linalg.norm(axis_in_base)
        
        # 计算轮子到base_link原点的距离
        distance = np.linalg.norm(wheel_position[:2])  # XY平面距离
        
        # 计算轮子滚动方向（垂直于旋转轴的方向）
        # 对于全向轮，滚动方向就是旋转轴在XY平面的垂直方向
        # 旋转轴 × Z轴 = 滚动方向
        z_axis = np.array([0, 0, 1])
        rolling_direction = np.cross(axis_in_base, z_axis)
        if np.linalg.norm(rolling_direction) > 1e-6:
            rolling_direction = rolling_direction / np.linalg.norm(rolling_direction)
        else:
            # 如果轴就是Z轴，默认滚动方向为X
            rolling_direction = np.array([1, 0, 0])
        
        # 计算滚动方向的角度（相对于+X轴，逆时针为正）
        rolling_angle = np.arctan2(rolling_direction[1], rolling_direction[0])
        
        return {
            'name': wheel_joint_name,
            'position': wheel_position,
            'distance': distance,
            'axis_local': joint_info['axis'],
            'axis_in_base': axis_in_base,
            'rolling_direction': rolling_direction,
            'rolling_angle_rad': rolling_angle,
            'rolling_angle_deg': np.degrees(rolling_angle)
        }
    
    def analyze_wheel_configuration(self):
        """
        分析所有轮子的配置
        
        Returns:
            dict: 轮子配置信息
        """
        # 轮子关节名称（基于URDF）- 使用轮子滚动关节
        wheel_joints = [
            'omni_wheel_mount-v5-2_to_wheel',  # wheel1 (后轮)
            'omni_wheel_mount-v5-1_to_wheel',  # wheel2 (右前轮)
            'omni_wheel_mount-v5_to_wheel'     # wheel3 (左前轮)
        ]
        
        results = {}
        
        print("=" * 80)
        print("轮子运动学配置分析")
        print("=" * 80)
        
        for i, joint_name in enumerate(wheel_joints, 1):
            print(f"\n【轮子{i}: {joint_name}】")
            
            info = self.get_axis_in_base_link(joint_name)
            results[f'wheel{i}'] = info
            
            print(f"  位置 (base_link坐标系): [{info['position'][0]:.6f}, {info['position'][1]:.6f}, {info['position'][2]:.6f}]")
            print(f"  到中心距离: {info['distance']:.6f} m")
            print(f"  旋转轴 (局部): [{info['axis_local'][0]:.6f}, {info['axis_local'][1]:.6f}, {info['axis_local'][2]:.6f}]")
            print(f"  旋转轴 (base_link): [{info['axis_in_base'][0]:.6f}, {info['axis_in_base'][1]:.6f}, {info['axis_in_base'][2]:.6f}]")
            print(f"  滚动方向: [{info['rolling_direction'][0]:.6f}, {info['rolling_direction'][1]:.6f}, {info['rolling_direction'][2]:.6f}]")
            print(f"  滚动角度: {info['rolling_angle_deg']:.2f}° (弧度: {info['rolling_angle_rad']:.6f})")
        
        return results
    
    def compute_jacobian_matrix(self, wheel_config, wheel_radius=0.05):
        """
        基于轮子配置计算理论雅可比矩阵
        
        Args:
            wheel_config: analyze_wheel_configuration()的返回值
            wheel_radius: 轮子半径(m)
        
        Returns:
            雅可比矩阵 J，使得 [w1, w2, w3]^T = J * [vx, vy, omega_z]^T
        """
        print("\n" + "=" * 80)
        print("理论雅可比矩阵计算")
        print("=" * 80)
        print("\n正确公式: w_i = (1/R) * [cos(θ_i)*vx + sin(θ_i)*vy + L_i*ω_z]")
        
        J = []
        
        for i in range(1, 4):
            wheel = wheel_config[f'wheel{i}']
            
            # 提取参数
            theta = wheel['rolling_angle_rad']  # 滚动方向角度
            L = wheel['distance']  # 到中心距离
            R = wheel_radius
            
            # 雅可比矩阵第i行（正确公式）：
            # w_i = (1/R) * [cos(theta) * vx + sin(theta) * vy + L * omega_z]
            row = [
                np.cos(theta) / R,   # vx系数
                np.sin(theta) / R,   # vy系数
                L / R                # omega_z系数
            ]
            
            J.append(row)
            
            print(f"\n轮子{i}:")
            print(f"  theta = {np.degrees(theta):.2f}°")
            print(f"  cos(θ) = {np.cos(theta):.6f}")
            print(f"  sin(θ) = {np.sin(theta):.6f}")
            print(f"  L = {L:.6f} m")
            print(f"  R = {R:.6f} m")
            print(f"  雅可比行: [{row[0]:.6f}, {row[1]:.6f}, {row[2]:.6f}]")
        
        J = np.array(J)
        
        print("\n完整雅可比矩阵 J:")
        print(J)
        
        # 计算伪逆
        J_pinv = np.linalg.pinv(J)
        print("\n雅可比伪逆矩阵 J_pinv:")
        print(J_pinv)
        
        # 测试前进命令
        print("\n" + "=" * 80)
        print("验证：前进命令 vx=0.2")
        print("=" * 80)
        cmd_vel = np.array([0.2, 0.0, 0.0])
        wheel_speeds = J @ cmd_vel
        print(f"轮子速度: [{wheel_speeds[0]:.2f}, {wheel_speeds[1]:.2f}, {wheel_speeds[2]:.2f}]")
        
        if abs(wheel_speeds[0]) < 0.5:
            print(f"✓ 后轮速度接近0 ({wheel_speeds[0]:.2f})，符合物理规律！")
        else:
            print(f"❌ 后轮速度过大 ({wheel_speeds[0]:.2f})，公式可能有误！")
        
        return J, J_pinv
    
    def compare_with_current(self, J_theory, current_J):
        """
        比较理论雅可比矩阵和当前使用的矩阵
        
        Args:
            J_theory: 理论雅可比矩阵
            current_J: 当前使用的雅可比矩阵
        """
        print("\n" + "=" * 80)
        print("与当前矩阵对比")
        print("=" * 80)
        
        print("\n当前使用的雅可比矩阵:")
        print(current_J)
        
        print("\n理论雅可比矩阵:")
        print(J_theory)
        
        print("\n差异 (理论 - 当前):")
        diff = J_theory - current_J
        print(diff)
        
        print("\n相对差异 (%):")
        # 避免除以零
        relative_diff = np.divide(diff, current_J, 
                                 out=np.zeros_like(diff), 
                                 where=np.abs(current_J) > 1e-6) * 100
        print(relative_diff)
        
        # 计算范数差异
        norm_diff = np.linalg.norm(diff)
        print(f"\n矩阵差异的Frobenius范数: {norm_diff:.6f}")


def convert_xacro_to_urdf(xacro_path):
    """
    将xacro文件转换为纯URDF
    
    Args:
        xacro_path: xacro文件路径
    
    Returns:
        转换后的URDF文件路径
    """
    print(f"正在将xacro转换为URDF...")
    
    # 创建临时文件
    temp_urdf = tempfile.NamedTemporaryFile(mode='w', suffix='.urdf', delete=False)
    temp_urdf_path = temp_urdf.name
    temp_urdf.close()
    
    try:
        # 使用xacro命令转换
        result = subprocess.run(
            ['xacro', xacro_path],
            capture_output=True,
            text=True,
            check=True
        )
        
        # 写入临时文件
        with open(temp_urdf_path, 'w') as f:
            f.write(result.stdout)
        
        print(f"转换成功，临时URDF: {temp_urdf_path}\n")
        return temp_urdf_path
        
    except subprocess.CalledProcessError as e:
        print(f"xacro转换失败: {e}")
        print(f"标准错误输出: {e.stderr}")
        sys.exit(1)
    except FileNotFoundError:
        print("错误: 找不到xacro命令")
        print("请安装: sudo apt-get install ros-<distro>-xacro")
        sys.exit(1)


def main():
    """主函数"""
    # 获取URDF路径
    script_dir = os.path.dirname(os.path.abspath(__file__))
    xacro_path = os.path.join(script_dir, '../../bot_description/urdf/lekiwi_bot.xacro')
    
    # 检查文件是否存在
    if not os.path.exists(xacro_path):
        print(f"错误: 找不到xacro文件: {xacro_path}")
        
        # 尝试其他可能的路径
        alternative_paths = [
            os.path.join(script_dir, '../../../bot_description/urdf/lekiwi_bot.xacro'),
            'src/bot_description/urdf/lekiwi_bot.xacro'
        ]
        
        for alt_path in alternative_paths:
            if os.path.exists(alt_path):
                xacro_path = alt_path
                print(f"找到xacro文件: {xacro_path}")
                break
        else:
            print("请手动指定xacro文件路径")
            sys.exit(1)
    
    print(f"正在分析xacro: {xacro_path}\n")
    
    # 转换xacro为URDF
    urdf_path = convert_xacro_to_urdf(xacro_path)
    
    try:
        # 创建分析器
        analyzer = URDFKinematicsAnalyzer(urdf_path)
        
        # 分析轮子配置
        wheel_config = analyzer.analyze_wheel_configuration()
        
        # 计算理论雅可比矩阵
        J_theory, J_pinv_theory = analyzer.compute_jacobian_matrix(wheel_config)
        
        # 当前使用的雅可比矩阵（从omni_controller_node.py复制）
        current_J = np.array([
            [-20.6180, -2.4408,  2.1351],
            [ 9.2779, -16.2609,  1.9657],
            [12.8186, 17.2044,  1.7161]
        ])
        
        # 对比
        analyzer.compare_with_current(J_theory, current_J)
        
        # 生成建议
        print("\n" + "=" * 80)
        print("修改建议")
        print("=" * 80)
        print("\n将以下理论雅可比矩阵应用到 omni_controller_node.py:")
        print("\nself.J = np.array([")
        for i, row in enumerate(J_theory):
            print(f"    [{row[0]:8.4f}, {row[1]:8.4f}, {row[2]:7.4f}]" +
                  ("," if i < len(J_theory)-1 else ""))
        print("])")
        
        print("\n或者如果需要更高精度:")
        print("\nself.J = np.array([")
        for i, row in enumerate(J_theory):
            print(f"    [{row[0]:12.8f}, {row[1]:12.8f}, {row[2]:11.8f}]" +
                  ("," if i < len(J_theory)-1 else ""))
        print("])")
        
    finally:
        # 删除临时文件
        if os.path.exists(urdf_path):
            os.unlink(urdf_path)
            print(f"\n已删除临时文件: {urdf_path}")


if __name__ == '__main__':
    main()