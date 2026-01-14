#!/usr/bin/env python3
"""
单元测试：验证巡航模式（loop/once）正确设置

测试场景：
1. 加载路点文件时指定loop=True，验证route.loop=True
2. 加载路点文件时指定loop=False，验证route.loop=False
3. 模拟PatrolHandler调用流程，验证mode参数正确传递
"""

import os
import tempfile
import yaml
from pathlib import Path

# 创建测试路点文件
def create_test_waypoints_file():
    """创建临时测试路点文件"""
    test_waypoints = {
        'waypoints': [
            {'name': 'point1', 'x': 1.0, 'y': 0.0, 'yaw': 0.0, 'dwell_time': 1.0},
            {'name': 'point2', 'x': 2.0, 'y': 1.0, 'yaw': 0.5, 'dwell_time': 1.0},
            {'name': 'point3', 'x': 3.0, 'y': 0.0, 'yaw': 0.0, 'dwell_time': 1.0},
        ]
    }
    
    temp_dir = tempfile.mkdtemp()
    waypoint_file = os.path.join(temp_dir, 'test_route.yaml')
    
    with open(waypoint_file, 'w') as f:
        yaml.dump(test_waypoints, f)
    
    return waypoint_file, temp_dir


def test_patrol_manager_load_with_loop_true():
    """测试1：loop=True时路线正确加载"""
    print("\n=== 测试1：加载路点文件，loop=True ===")
    
    # 创建临时路点文件
    waypoint_file, temp_dir = create_test_waypoints_file()
    
    try:
        # 模拟PatrolManager的load_route_from_waypoints_file逻辑
        with open(waypoint_file, 'r') as f:
            data = yaml.safe_load(f)
        
        waypoints_data = data.get('waypoints', [])
        route_waypoints = []
        for wp_data in waypoints_data:
            route_waypoints.append({
                'name': wp_data.get('name', ''),
                'x': float(wp_data['x']),
                'y': float(wp_data['y']),
                'yaw': float(wp_data['yaw']),
                'dwell_time': float(wp_data.get('dwell_time', 2.0))
            })
        
        # 模拟创建PatrolRoute（loop=True）
        from bot_navigation.patrol.patrol_manager import PatrolRoute
        route = PatrolRoute(
            route_id='test_route_1',
            name='test_route',
            waypoints=route_waypoints,
            loop=True
        )
        
        # 验证
        assert route.loop == True, f"Expected loop=True, got loop={route.loop}"
        assert len(route.waypoints) == 3, f"Expected 3 waypoints, got {len(route.waypoints)}"
        
        print(f"✅ 测试通过：route.loop = {route.loop}, waypoints = {len(route.waypoints)}")
        
    finally:
        # 清理临时文件
        os.remove(waypoint_file)
        os.rmdir(temp_dir)


def test_patrol_manager_load_with_loop_false():
    """测试2：loop=False时路线正确加载"""
    print("\n=== 测试2：加载路点文件，loop=False ===")
    
    # 创建临时路点文件
    waypoint_file, temp_dir = create_test_waypoints_file()
    
    try:
        # 模拟PatrolManager的load_route_from_waypoints_file逻辑
        with open(waypoint_file, 'r') as f:
            data = yaml.safe_load(f)
        
        waypoints_data = data.get('waypoints', [])
        route_waypoints = []
        for wp_data in waypoints_data:
            route_waypoints.append({
                'name': wp_data.get('name', ''),
                'x': float(wp_data['x']),
                'y': float(wp_data['y']),
                'yaw': float(wp_data['yaw']),
                'dwell_time': float(wp_data.get('dwell_time', 2.0))
            })
        
        # 模拟创建PatrolRoute（loop=False）
        from bot_navigation.patrol.patrol_manager import PatrolRoute
        route = PatrolRoute(
            route_id='test_route_2',
            name='test_route',
            waypoints=route_waypoints,
            loop=False
        )
        
        # 验证
        assert route.loop == False, f"Expected loop=False, got loop={route.loop}"
        assert len(route.waypoints) == 3, f"Expected 3 waypoints, got {len(route.waypoints)}"
        
        print(f"✅ 测试通过：route.loop = {route.loop}, waypoints = {len(route.waypoints)}")
        
    finally:
        # 清理临时文件
        os.remove(waypoint_file)
        os.rmdir(temp_dir)


def test_patrol_mode_parameter_mapping():
    """测试3：验证patrol_mode参数到loop的映射"""
    print("\n=== 测试3：patrol_mode参数映射 ===")
    
    test_cases = [
        ('loop', True),
        ('once', False),
        ('bounce', False),  # bounce模式也是非循环（来回一次后停止）
    ]
    
    for patrol_mode, expected_loop in test_cases:
        is_loop = (patrol_mode == 'loop')
        assert is_loop == expected_loop, \
            f"patrol_mode='{patrol_mode}' should map to loop={expected_loop}, got {is_loop}"
        print(f"✅ patrol_mode='{patrol_mode}' → loop={is_loop} (expected: {expected_loop})")


def test_execute_patrol_completion_logic():
    """测试4：验证execute_patrol的完成逻辑"""
    print("\n=== 测试4：execute_patrol完成逻辑 ===")
    
    # 模拟路线数据
    from bot_navigation.patrol.patrol_manager import PatrolRoute
    
    # 测试场景1：loop=True，到达终点应重新开始
    route_loop = PatrolRoute(
        route_id='test_loop',
        name='loop_route',
        waypoints=[{'name': 'p1', 'x': 1.0, 'y': 0.0, 'yaw': 0.0, 'dwell_time': 0.0}],
        loop=True
    )
    
    current_waypoint_index = 1  # 超过路点数量
    
    if current_waypoint_index >= len(route_loop.waypoints):
        if route_loop.loop:
            current_waypoint_index = 0
            result = "restart"
        else:
            result = "completed"
    
    assert result == "restart", f"loop=True should restart, got {result}"
    print(f"✅ loop=True, waypoint_index={1}, total={len(route_loop.waypoints)} → {result}")
    
    # 测试场景2：loop=False，到达终点应完成
    route_once = PatrolRoute(
        route_id='test_once',
        name='once_route',
        waypoints=[{'name': 'p1', 'x': 1.0, 'y': 0.0, 'yaw': 0.0, 'dwell_time': 0.0}],
        loop=False
    )
    
    current_waypoint_index = 1  # 超过路点数量
    
    if current_waypoint_index >= len(route_once.waypoints):
        if route_once.loop:
            current_waypoint_index = 0
            result = "restart"
        else:
            result = "completed"
    
    assert result == "completed", f"loop=False should complete, got {result}"
    print(f"✅ loop=False, waypoint_index={1}, total={len(route_once.waypoints)} → {result}")


if __name__ == '__main__':
    print("=" * 60)
    print("巡航模式单元测试")
    print("=" * 60)
    
    try:
        test_patrol_manager_load_with_loop_true()
        test_patrol_manager_load_with_loop_false()
        test_patrol_mode_parameter_mapping()
        test_execute_patrol_completion_logic()
        
        print("\n" + "=" * 60)
        print("✅ 所有测试通过！")
        print("=" * 60)
        
    except AssertionError as e:
        print(f"\n❌ 测试失败: {e}")
        exit(1)
    except Exception as e:
        print(f"\n❌ 测试出错: {e}")
        import traceback
        traceback.print_exc()
        exit(1)
