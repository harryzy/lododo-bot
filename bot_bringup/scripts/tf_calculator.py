#!/usr/bin/env python3
"""
TF坐标转换计算工具 / TF Coordinate Transformation Calculator
用途：根据目标在base_link中的期望位置和轴方向，计算URDF joint的xyz和rpy值
Usage: Calculate URDF joint xyz and rpy values based on desired position and axis directions in base_link

核心功能：
1. 输入目标坐标系在base_link中的位置和各轴指向
2. 输入父坐标系在base_link中的TF（从ros2 run tf2_ros tf2_echo获取）
3. 自动计算URDF joint中应设置的xyz和rpy值

作者：lododo Bot Team
日期：2025-12-03
"""

import numpy as np
import sys
from scipy.spatial.transform import Rotation


def rotation_matrix_from_rpy(roll, pitch, yaw):
    """
    从RPY欧拉角创建旋转矩阵
    Create rotation matrix from RPY Euler angles
    
    Args:
        roll: 绕X轴旋转角度（弧度）
        pitch: 绕Y轴旋转角度（弧度）
        yaw: 绕Z轴旋转角度（弧度）
    
    Returns:
        3x3旋转矩阵
    """
    r = Rotation.from_euler('xyz', [roll, pitch, yaw])
    return r.as_matrix()


def rpy_from_rotation_matrix(R):
    """
    从旋转矩阵提取RPY欧拉角
    Extract RPY Euler angles from rotation matrix
    
    Args:
        R: 3x3旋转矩阵
    
    Returns:
        (roll, pitch, yaw) 元组（弧度）
    """
    r = Rotation.from_matrix(R)
    return r.as_euler('xyz')


def tf_to_transform_matrix(xyz, rpy):
    """
    将xyz和rpy转换为4x4齐次变换矩阵
    Convert xyz and rpy to 4x4 homogeneous transformation matrix
    
    Args:
        xyz: [x, y, z] 位置向量
        rpy: [roll, pitch, yaw] 旋转角度（弧度）
    
    Returns:
        4x4齐次变换矩阵
    """
    T = np.eye(4)
    T[:3, :3] = rotation_matrix_from_rpy(rpy[0], rpy[1], rpy[2])
    T[:3, 3] = xyz
    return T


def transform_matrix_to_tf(T):
    """
    将4x4齐次变换矩阵转换为xyz和rpy
    Convert 4x4 homogeneous transformation matrix to xyz and rpy
    
    Args:
        T: 4x4齐次变换矩阵
    
    Returns:
        (xyz, rpy) 元组，xyz为位置向量，rpy为旋转角度（弧度）
    """
    xyz = T[:3, 3]
    rpy = rpy_from_rotation_matrix(T[:3, :3])
    return xyz, rpy


def rotation_matrix_from_axes(x_axis, y_axis, z_axis):
    """
    从三个轴方向创建旋转矩阵
    Create rotation matrix from three axis directions
    
    旋转矩阵的列向量就是子坐标系各轴在父坐标系中的方向
    The column vectors of rotation matrix are the directions of child frame axes in parent frame
    
    Args:
        x_axis: X轴在父坐标系中的方向向量
        y_axis: Y轴在父坐标系中的方向向量
        z_axis: Z轴在父坐标系中的方向向量
    
    Returns:
        3x3旋转矩阵
    """
    x = np.array(x_axis, dtype=float)
    y = np.array(y_axis, dtype=float)
    z = np.array(z_axis, dtype=float)
    
    # 归一化
    x = x / np.linalg.norm(x)
    y = y / np.linalg.norm(y)
    z = z / np.linalg.norm(z)
    
    # 旋转矩阵的列向量是各轴方向
    R = np.column_stack([x, y, z])
    return R


def build_rotation_from_z_axis(z_axis_direction, hint_up=None):
    """
    根据Z轴指向方向构建完整的旋转矩阵
    Build complete rotation matrix from Z-axis direction
    
    对于相机光学坐标系，用户通常只关心Z轴（光轴）的方向
    For camera optical frame, user usually only cares about Z-axis (optical axis) direction
    
    Args:
        z_axis_direction: Z轴在base_link中的方向，例如 'X', '-X', 'Y', '-Y', 'Z', '-Z'
                         或者直接给向量 [x, y, z]
        hint_up: 提示"上"方向，用于确定X/Y轴，默认为[0,0,1]
    
    Returns:
        3x3旋转矩阵
    """
    # 解析Z轴方向
    if isinstance(z_axis_direction, str):
        direction_map = {
            'X': [1, 0, 0], '+X': [1, 0, 0],
            '-X': [-1, 0, 0],
            'Y': [0, 1, 0], '+Y': [0, 1, 0],
            '-Y': [0, -1, 0],
            'Z': [0, 0, 1], '+Z': [0, 0, 1],
            '-Z': [0, 0, -1],
        }
        z = np.array(direction_map.get(z_axis_direction.upper(), [1, 0, 0]), dtype=float)
    else:
        z = np.array(z_axis_direction, dtype=float)
    
    z = z / np.linalg.norm(z)
    
    # 默认提示"上"为Z轴
    if hint_up is None:
        hint_up = np.array([0, 0, 1], dtype=float)
    else:
        hint_up = np.array(hint_up, dtype=float)
    
    # 如果Z轴与hint_up平行，使用X轴作为参考
    if abs(np.dot(z, hint_up)) > 0.99:
        hint_up = np.array([1, 0, 0], dtype=float)
    
    # 计算X轴 = hint_up × Z（右手系）
    # 对于相机：X轴应该指向"右"，这里用hint_up叉乘Z
    x = np.cross(hint_up, z)
    x = x / np.linalg.norm(x)
    
    # 计算Y轴 = Z × X
    y = np.cross(z, x)
    y = y / np.linalg.norm(y)
    
    return rotation_matrix_from_axes(x, y, z)


def calculate_joint_transform_from_axes(target_xyz, target_rotation_matrix, 
                                        parent_xyz, parent_rpy):
    """
    根据目标位置和旋转矩阵，计算joint相对于parent的变换
    Calculate joint transform relative to parent from target position and rotation matrix
    
    Args:
        target_xyz: 目标在base_link中的位置 [x, y, z]
        target_rotation_matrix: 目标在base_link中的旋转矩阵 (3x3)
        parent_xyz: 父坐标系在base_link中的位置 [x, y, z]
        parent_rpy: 父坐标系在base_link中的姿态 [roll, pitch, yaw]（弧度）
    
    Returns:
        {'xyz': [x, y, z], 'rpy': [roll, pitch, yaw]} - joint的origin值
    """
    # 创建父坐标系到base_link的变换矩阵
    T_base_parent = tf_to_transform_matrix(parent_xyz, parent_rpy)
    
    # 创建目标在base_link中的变换矩阵
    T_base_target = np.eye(4)
    T_base_target[:3, :3] = target_rotation_matrix
    T_base_target[:3, 3] = target_xyz
    
    # 计算目标相对于父坐标系的变换: T_parent_target = inv(T_base_parent) * T_base_target
    T_parent_target = np.linalg.inv(T_base_parent) @ T_base_target
    
    # 提取xyz和rpy
    xyz, rpy = transform_matrix_to_tf(T_parent_target)
    
    return {'xyz': xyz.tolist(), 'rpy': rpy.tolist()}


def calculate_joint_transform(target_in_base, parent_in_base_xyz, parent_in_base_rpy):
    """
    计算joint相对于parent的变换（使用RPY方式指定目标姿态）
    Calculate joint transform relative to parent
    
    Args:
        target_in_base: 目标在base_link中的期望位置和姿态
                       格式: {'xyz': [x, y, z], 'rpy': [roll, pitch, yaw]}
        parent_in_base_xyz: 父坐标系在base_link中的位置 [x, y, z]
        parent_in_base_rpy: 父坐标系在base_link中的姿态 [roll, pitch, yaw]（弧度）
    
    Returns:
        {'xyz': [x, y, z], 'rpy': [roll, pitch, yaw]} - joint的origin值
    """
    # 创建父坐标系到base_link的变换矩阵
    T_base_parent = tf_to_transform_matrix(parent_in_base_xyz, parent_in_base_rpy)
    
    # 创建目标在base_link中的变换矩阵
    T_base_target = tf_to_transform_matrix(
        target_in_base['xyz'], 
        target_in_base['rpy']
    )
    
    # 计算目标相对于父坐标系的变换: T_parent_target = inv(T_base_parent) * T_base_target
    T_parent_target = np.linalg.inv(T_base_parent) @ T_base_target
    
    # 提取xyz和rpy
    xyz, rpy = transform_matrix_to_tf(T_parent_target)
    
    return {'xyz': xyz.tolist(), 'rpy': rpy.tolist()}


def search_rpy_for_axis_alignment(parent_rpy, target_axis='Z', target_direction='X', 
                                   pitch_offset=0, search_step=90):
    """
    搜索使子坐标系的某个轴指向base_link某个方向的RPY组合
    Search for RPY combination that aligns child axis to target direction in base_link
    
    这是从实际调试中学到的方法：当父坐标系姿态复杂时，
    直接计算RPY可能不直观，搜索法更可靠
    
    Args:
        parent_rpy: 父坐标系在base_link中的RPY (弧度)
        target_axis: 目标轴 'X', 'Y', 或 'Z'
        target_direction: 在base_link中的目标方向 'X', '-X', 'Y', '-Y', 'Z', '-Z'
        pitch_offset: pitch方向的额外偏移（弧度），如相机的10度俯仰
        search_step: 搜索步长（度）
    
    Returns:
        最佳的 [roll, pitch, yaw] (弧度)
    """
    # 解析目标方向
    direction_map = {
        'X': [1, 0, 0], '+X': [1, 0, 0], '-X': [-1, 0, 0],
        'Y': [0, 1, 0], '+Y': [0, 1, 0], '-Y': [0, -1, 0],
        'Z': [0, 0, 1], '+Z': [0, 0, 1], '-Z': [0, 0, -1],
    }
    target_vec = np.array(direction_map[target_direction.upper()])
    
    # 父坐标系旋转矩阵
    R_parent = rotation_matrix_from_rpy(*parent_rpy)
    
    # 轴索引
    axis_idx = {'X': 0, 'Y': 1, 'Z': 2}[target_axis.upper()]
    
    best_rpy = None
    best_error = float('inf')
    
    # 搜索所有可能的RPY组合
    angles = [np.deg2rad(a) for a in range(-180, 181, search_step)]
    pitch_angles = [np.deg2rad(a) + pitch_offset for a in range(-180, 181, search_step)]
    
    for roll in angles:
        for pitch in pitch_angles:
            for yaw in angles:
                # 计算joint的旋转矩阵
                R_joint = rotation_matrix_from_rpy(roll, pitch, yaw)
                
                # 子坐标系在base_link中的旋转 = R_parent * R_joint
                R_child_in_base = R_parent @ R_joint
                
                # 获取目标轴在base_link中的方向
                axis_in_base = R_child_in_base[:, axis_idx]
                
                # 计算与目标方向的误差
                error = np.linalg.norm(axis_in_base - target_vec)
                
                if error < best_error:
                    best_error = error
                    best_rpy = [roll, pitch, yaw]
    
    return best_rpy, best_error


def print_transform(name, xyz, rpy):
    """打印变换信息 / Print transformation information"""
    print(f"\n{name}:")
    print(f"  xyz: [{xyz[0]:.6f}, {xyz[1]:.6f}, {xyz[2]:.6f}]")
    print(f"  rpy: [{rpy[0]:.6f}, {rpy[1]:.6f}, {rpy[2]:.6f}]")
    print(f"  rpy(deg): [{np.rad2deg(rpy[0]):.2f}, {np.rad2deg(rpy[1]):.2f}, {np.rad2deg(rpy[2]):.2f}]")


def print_rotation_matrix(name, R):
    """打印旋转矩阵及其轴方向 / Print rotation matrix and axis directions"""
    print(f"\n{name} Rotation Matrix:")
    for i in range(3):
        print(f"  [{R[i,0]:7.4f}, {R[i,1]:7.4f}, {R[i,2]:7.4f}]")
    
    print(f"\n{name} Axes in Parent Frame:")
    print(f"  X-axis: [{R[0,0]:7.4f}, {R[1,0]:7.4f}, {R[2,0]:7.4f}]")
    print(f"  Y-axis: [{R[0,1]:7.4f}, {R[1,1]:7.4f}, {R[2,1]:7.4f}]")
    print(f"  Z-axis: [{R[0,2]:7.4f}, {R[1,2]:7.4f}, {R[2,2]:7.4f}]")


def print_urdf_origin(xyz, rpy):
    """打印URDF格式的origin标签 / Print URDF format origin tag"""
    # 常用的xacro常量
    xacro_constants = {
        np.pi: "M_PI",
        np.pi/2: "M_PI_2",
        np.deg2rad(10): "DEG_10",
        np.deg2rad(80): "DEG_80",
    }
    
    print("\nURDF origin tag:")
    print(f'<origin xyz="{xyz[0]:.6f} {xyz[1]:.6f} {xyz[2]:.6f}" rpy="{rpy[0]:.6f} {rpy[1]:.6f} {rpy[2]:.6f}" />')
    
    # 尝试用xacro常量表示
    def angle_to_xacro(angle):
        for val, name in xacro_constants.items():
            if abs(abs(angle) - val) < 0.01:
                sign = "-" if angle < 0 else ""
                return f"{sign}${{{name}}}"
        return f"{angle:.6f}"
    
    rpy_xacro = [angle_to_xacro(a) for a in rpy]
    print(f'\nXacro format:')
    print(f'<origin xyz="{xyz[0]:.6f} {xyz[1]:.6f} {xyz[2]:.6f}" rpy="{rpy_xacro[0]} {rpy_xacro[1]} {rpy_xacro[2]}" />')


def verify_result(parent_xyz, parent_rpy, joint_xyz, joint_rpy, target_xyz, target_R=None):
    """验证计算结果 / Verify calculation results"""
    print("\n" + "=" * 80)
    print("Verification:")
    print("=" * 80)
    
    # 正向验证：使用计算出的joint值，确认能得到目标位置
    T_base_parent = tf_to_transform_matrix(parent_xyz, parent_rpy)
    T_parent_child = tf_to_transform_matrix(joint_xyz, joint_rpy)
    T_base_child = T_base_parent @ T_parent_child
    
    verify_xyz, verify_rpy = transform_matrix_to_tf(T_base_child)
    verify_R = T_base_child[:3, :3]
    
    print_transform("Computed child frame in base_link", verify_xyz, verify_rpy)
    print_rotation_matrix("Child frame", verify_R)
    
    # 计算位置误差
    xyz_error = np.linalg.norm(verify_xyz - np.array(target_xyz))
    print(f"\nPosition error: {xyz_error * 1000:.3f} mm")
    
    # 如果有目标旋转矩阵，计算旋转误差
    if target_R is not None:
        R_error = np.linalg.norm(verify_R - target_R, 'fro')
        print(f"Rotation error: {R_error:.6f}")
    
    if xyz_error < 0.001:
        print("\n[OK] Position verified!")
    else:
        print("\n[WARN] Position error too large, check inputs.")
    
    return verify_xyz, verify_rpy, verify_R


def camera_frame_example():
    """
    camera_frame计算示例
    
    目标：计算camera_frame（相机光学坐标系）的joint参数
    - 位置：[0.146, 0, 0.038] - 在base_link前方146mm，高度38mm，Y=0对中
    - 姿态：Z轴指向base_link的X方向（前方），相机略微下俯10度
    
    ROS相机光学坐标系标准：
    - Z轴：光轴（拍摄方向）
    - X轴：图像右方
    - Y轴：图像下方
    """
    print("=" * 80)
    print("TF Calculator - camera_frame Example")
    print("=" * 80)
    
    # ============= 1. 定义目标 =============
    
    # 目标位置（在base_link中）
    target_xyz = [0.146, 0.000, 0.038]
    
    # 目标姿态：Z轴指向base_link的X方向（前方）
    # 使用轴方向定义法 - 更直观
    # 对于相机光学坐标系，我们希望：
    #   Z轴 -> base_link X（前方，拍摄方向）
    #   X轴 -> base_link Z（上方，图像右侧）
    #   Y轴 -> base_link -Y（右侧，图像下方）
    target_R = rotation_matrix_from_axes(
        x_axis=[0, 0, 1],    # camera X -> base Z (up)
        y_axis=[0, -1, 0],   # camera Y -> base -Y (right)
        z_axis=[1, 0, 0]     # camera Z -> base X (forward)
    )
    
    print("\nTarget Definition:")
    print(f"  Position: {target_xyz}")
    print_rotation_matrix("Target orientation", target_R)
    
    # ============= 2. 获取父坐标系TF =============
    
    # 从 `ros2 run tf2_ros tf2_echo base_link Camera-Model-v3` 获取
    # 或者从URDF变换链计算
    parent_xyz = [0.106367, 0.014, 0.038319]
    parent_rpy = [-np.pi, np.deg2rad(-80), np.pi/2]  # [-180, -80, 90] deg
    
    print("\nParent Frame (Camera-Model-v3) TF in base_link:")
    print_transform("Camera-Model-v3", parent_xyz, parent_rpy)
    
    R_parent = rotation_matrix_from_rpy(*parent_rpy)
    print_rotation_matrix("Camera-Model-v3", R_parent)
    
    # ============= 3. 计算joint参数 =============
    
    result = calculate_joint_transform_from_axes(
        target_xyz, target_R,
        parent_xyz, parent_rpy
    )
    
    print("\n" + "=" * 80)
    print("Result - camera_frame_joint Origin:")
    print("=" * 80)
    print_transform("camera_frame_joint", result['xyz'], result['rpy'])
    print_urdf_origin(result['xyz'], result['rpy'])
    
    # ============= 4. 验证 =============
    
    verify_result(parent_xyz, parent_rpy, 
                  result['xyz'], result['rpy'],
                  target_xyz, target_R)
    
    # ============= 5. 额外：搜索法验证 =============
    
    print("\n" + "=" * 80)
    print("Search Method Verification (find RPY that aligns Z-axis to base_link X):")
    print("=" * 80)
    
    # 使用搜索法找到最佳RPY
    best_rpy, error = search_rpy_for_axis_alignment(
        parent_rpy, 
        target_axis='Z', 
        target_direction='X',
        pitch_offset=np.deg2rad(10),  # 10 deg pitch
        search_step=90
    )
    
    print(f"\nSearch Result:")
    print(f"  Best RPY: [{np.rad2deg(best_rpy[0]):.1f}, {np.rad2deg(best_rpy[1]):.1f}, {np.rad2deg(best_rpy[2]):.1f}] deg")
    print(f"  Direction error: {error:.6f}")
    
    return result


def interactive_mode():
    """
    交互式模式 - 引导用户输入参数 / Interactive mode - guide user input parameters
    """
    print("=" * 80)
    print("TF Calculator - Interactive Mode")
    print("=" * 80)
    
    print("\nThis tool helps you calculate URDF joint xyz and rpy values.")
    print("You need to provide:")
    print("  1. Target position of child frame in base_link")
    print("  2. Target axis directions of child frame in base_link")
    print("  3. Parent frame TF in base_link (from tf2_echo)")
    
    try:
        # 1. 输入目标位置
        print("\n" + "-" * 40)
        print("Step 1: Target position in base_link")
        print("-" * 40)
        target_x = float(input("  X (m): "))
        target_y = float(input("  Y (m): "))
        target_z = float(input("  Z (m): "))
        target_xyz = [target_x, target_y, target_z]
        
        # 2. 输入目标姿态
        print("\n" + "-" * 40)
        print("Step 2: Target axis directions in base_link")
        print("Options: X, -X, Y, -Y, Z, -Z or custom vector like 1,0,0")
        print("-" * 40)
        
        def parse_direction(s):
            s = s.strip().upper()
            direction_map = {
                'X': [1,0,0], '+X': [1,0,0], '-X': [-1,0,0],
                'Y': [0,1,0], '+Y': [0,1,0], '-Y': [0,-1,0],
                'Z': [0,0,1], '+Z': [0,0,1], '-Z': [0,0,-1],
            }
            if s in direction_map:
                return direction_map[s]
            else:
                parts = s.replace(' ', '').split(',')
                return [float(p) for p in parts]
        
        print("\n  Which direction should child X-axis point to in base_link?")
        x_dir = parse_direction(input("  X-axis direction: "))
        
        print("  Which direction should child Y-axis point to in base_link?")
        y_dir = parse_direction(input("  Y-axis direction: "))
        
        print("  Which direction should child Z-axis point to in base_link?")
        z_dir = parse_direction(input("  Z-axis direction: "))
        
        target_R = rotation_matrix_from_axes(x_dir, y_dir, z_dir)
        
        # 3. 输入父坐标系TF
        print("\n" + "-" * 40)
        print("Step 3: Parent frame TF in base_link")
        print("Hint: Use `ros2 run tf2_ros tf2_echo base_link <parent_frame>` to get it")
        print("-" * 40)
        
        parent_x = float(input("  Parent X (m): "))
        parent_y = float(input("  Parent Y (m): "))
        parent_z = float(input("  Parent Z (m): "))
        parent_xyz = [parent_x, parent_y, parent_z]
        
        print("\n  Parent RPY (degrees):")
        parent_roll = np.deg2rad(float(input("  Roll (deg): ")))
        parent_pitch = np.deg2rad(float(input("  Pitch (deg): ")))
        parent_yaw = np.deg2rad(float(input("  Yaw (deg): ")))
        parent_rpy = [parent_roll, parent_pitch, parent_yaw]
        
        # 4. 计算
        print("\n" + "=" * 80)
        print("Calculating...")
        print("=" * 80)
        
        result = calculate_joint_transform_from_axes(
            target_xyz, target_R,
            parent_xyz, parent_rpy
        )
        
        print("\nResult - Joint Origin:")
        print_transform("Joint", result['xyz'], result['rpy'])
        print_urdf_origin(result['xyz'], result['rpy'])
        
        # 5. 验证
        verify_result(parent_xyz, parent_rpy,
                      result['xyz'], result['rpy'],
                      target_xyz, target_R)
        
    except ValueError as e:
        print(f"\nError: Invalid input - {e}")
    except KeyboardInterrupt:
        print("\n\nCancelled.")


def quick_mode(args):
    """
    快速模式 - 命令行参数直接计算 / Quick mode - direct calculation with command line arguments
    
    用法: python tf_calculator.py -q <target_xyz> <z_axis_dir> <parent_xyz> <parent_rpy>
    示例: python tf_calculator.py -q "0.146,0,0.038" "X" "0.106,0.014,0.038" "-180,-80,90"
    Usage: python tf_calculator.py -q <target_xyz> <z_axis_dir> <parent_xyz> <parent_rpy_deg>
    Example: python tf_calculator.py -q '0.146,0,0.038' 'X' '0.106,0.014,0.038' '-180,-80,90'
    """
    if len(args) < 4:
        print("Usage: python tf_calculator.py -q <target_xyz> <z_axis_dir> <parent_xyz> <parent_rpy_deg>")
        print("Example: python tf_calculator.py -q '0.146,0,0.038' 'X' '0.106,0.014,0.038' '-180,-80,90'")
        return
    
    try:
        # 解析参数
        target_xyz = [float(x) for x in args[0].split(',')]
        z_dir = args[1]
        parent_xyz = [float(x) for x in args[2].split(',')]
        parent_rpy_deg = [float(x) for x in args[3].split(',')]
        parent_rpy = [np.deg2rad(x) for x in parent_rpy_deg]
        
        # 构建目标旋转矩阵
        target_R = build_rotation_from_z_axis(z_dir)
        
        # 计算
        result = calculate_joint_transform_from_axes(
            target_xyz, target_R,
            parent_xyz, parent_rpy
        )
        
        print("Joint Origin:")
        print_transform("Result", result['xyz'], result['rpy'])
        print_urdf_origin(result['xyz'], result['rpy'])
        
    except Exception as e:
        print(f"Error: {e}")


def main():
    """主函数 / Main function"""
    if len(sys.argv) > 1:
        if sys.argv[1] == "-i":
            interactive_mode()
        elif sys.argv[1] == "-q":
            quick_mode(sys.argv[2:])
        elif sys.argv[1] == "-h" or sys.argv[1] == "--help":
            print("TF Coordinate Transformation Calculator")
            print("\nUsage:")
            print("  python tf_calculator.py        # Run camera_frame example")
            print("  python tf_calculator.py -i     # Interactive mode")
            print("  python tf_calculator.py -q ... # Quick mode")
            print("  python tf_calculator.py -h     # Show help")
        else:
            camera_frame_example()
    else:
        camera_frame_example()
        print("\n" + "=" * 80)
        print("Hints:")
        print("  python tf_calculator.py -i    # Interactive mode")
        print("  python tf_calculator.py -h    # Show help")
        print("=" * 80)


if __name__ == "__main__":
    main()
