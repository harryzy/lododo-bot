#!/usr/bin/env python3
"""
测试任务清除服务

演示如何使用 /mission/clear_tasks 服务清除任务
"""

import sys
import rclpy
from rclpy.node import Node
from bot_navigation_msgs.srv import ClearTasks, CreateTask, ListTasks


def clear_tasks_by_ids(node, task_ids, clear_history=False):
    """按任务ID清除任务"""
    client = node.create_client(ClearTasks, '/mission/clear_tasks')
    
    if not client.wait_for_service(timeout_sec=2.0):
        node.get_logger().error('Clear tasks service not available')
        return False
    
    request = ClearTasks.Request()
    request.task_ids = task_ids
    request.states = []
    request.clear_history = clear_history
    
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future, timeout_sec=5.0)
    
    if future.result() is not None:
        response = future.result()
        node.get_logger().info(
            f"Clear by IDs: {response.message}\n"
            f"  Cleared: {response.cleared_count} tasks\n"
            f"  Task IDs: {response.cleared_task_ids}"
        )
        return response.success
    else:
        node.get_logger().error('Service call failed')
        return False


def clear_tasks_by_states(node, states, clear_history=False):
    """按状态清除任务"""
    client = node.create_client(ClearTasks, '/mission/clear_tasks')
    
    if not client.wait_for_service(timeout_sec=2.0):
        node.get_logger().error('Clear tasks service not available')
        return False
    
    request = ClearTasks.Request()
    request.task_ids = []
    request.states = states
    request.clear_history = clear_history
    
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future, timeout_sec=5.0)
    
    if future.result() is not None:
        response = future.result()
        node.get_logger().info(
            f"Clear by states {states}: {response.message}\n"
            f"  Cleared: {response.cleared_count} tasks\n"
            f"  Task IDs: {response.cleared_task_ids}"
        )
        return response.success
    else:
        node.get_logger().error('Service call failed')
        return False


def clear_all_tasks(node, clear_history=False):
    """清除所有任务"""
    return clear_tasks_by_states(node, ['all'], clear_history)


def list_tasks(node):
    """列出所有任务"""
    client = node.create_client(ListTasks, '/mission/list_tasks')
    
    if not client.wait_for_service(timeout_sec=2.0):
        node.get_logger().error('List tasks service not available')
        return
    
    request = ListTasks.Request()
    request.filter = 'all'
    
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future, timeout_sec=5.0)
    
    if future.result() is not None:
        response = future.result()
        node.get_logger().info(f"\n{'='*60}")
        node.get_logger().info(f"Current tasks: {len(response.task_ids)}")
        node.get_logger().info(f"{'='*60}")
        for i, task_id in enumerate(response.task_ids):
            node.get_logger().info(
                f"  [{i+1}] {task_id}\n"
                f"      Type: {response.task_types[i]}\n"
                f"      State: {response.states[i]}\n"
                f"      Priority: {response.priorities[i]}"
            )
        node.get_logger().info(f"{'='*60}\n")
    else:
        node.get_logger().error('Service call failed')


def main():
    rclpy.init()
    node = Node('clear_tasks_test')
    
    print("\n" + "="*60)
    print("任务清除服务测试")
    print("="*60)
    
    if len(sys.argv) < 2:
        print("\n使用方法:")
        print("  1. 按状态清除:")
        print("     ros2 run bot_navigation clear_tasks_test states completed failed canceled")
        print("  2. 按ID清除:")
        print("     ros2 run bot_navigation clear_tasks_test ids task_001 task_002")
        print("  3. 清除所有:")
        print("     ros2 run bot_navigation clear_tasks_test all")
        print("  4. 清除并删除历史:")
        print("     ros2 run bot_navigation clear_tasks_test all --clear-history")
        print("\n可用状态: pending, running, completed, failed, canceled, timeout, blocked")
        print("="*60 + "\n")
        node.destroy_node()
        rclpy.shutdown()
        return
    
    # 先列出当前任务
    print("\n--- 清除前的任务列表 ---")
    list_tasks(node)
    
    command = sys.argv[1].lower()
    clear_history = '--clear-history' in sys.argv
    
    if command == 'all':
        # 清除所有任务
        print(f"\n--- 清除所有任务 (clear_history={clear_history}) ---")
        clear_all_tasks(node, clear_history)
        
    elif command == 'states':
        # 按状态清除
        if len(sys.argv) < 3:
            print("错误: 请指定至少一个状态")
            node.destroy_node()
            rclpy.shutdown()
            return
        
        states = [s for s in sys.argv[2:] if not s.startswith('--')]
        print(f"\n--- 按状态清除: {states} (clear_history={clear_history}) ---")
        clear_tasks_by_states(node, states, clear_history)
        
    elif command == 'ids':
        # 按ID清除
        if len(sys.argv) < 3:
            print("错误: 请指定至少一个任务ID")
            node.destroy_node()
            rclpy.shutdown()
            return
        
        task_ids = [s for s in sys.argv[2:] if not s.startswith('--')]
        print(f"\n--- 按ID清除: {task_ids} (clear_history={clear_history}) ---")
        clear_tasks_by_ids(node, task_ids, clear_history)
        
    else:
        print(f"错误: 未知命令 '{command}'")
        node.destroy_node()
        rclpy.shutdown()
        return
    
    # 再次列出任务
    print("\n--- 清除后的任务列表 ---")
    list_tasks(node)
    
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
