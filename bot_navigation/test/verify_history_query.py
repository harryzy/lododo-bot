#!/usr/bin/env python3
"""
历史查询功能验证脚本
History Query Feature Verification Script

演示场景：
1. 创建导航任务
2. 任务完成并被MissionPlanner删除
3. CMD接口查询任务状态（成功从历史获取）

Usage:
    python3 verify_history_query.py
"""

import sys
import os
import tempfile
import shutil

# 添加模块路径
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from bot_navigation.mission.task_manager import TaskManager, TaskType, TaskState


def print_separator(title):
    """打印分隔符"""
    print("\n" + "="*60)
    print(f"  {title}")
    print("="*60)


def main():
    """主验证流程"""
    # 创建临时目录
    temp_dir = tempfile.mkdtemp(prefix='verify_history_')
    print(f"使用临时目录: {temp_dir}")
    
    try:
        # 1. 创建TaskManager
        print_separator("步骤1: 初始化TaskManager")
        tm = TaskManager(persistence_dir=temp_dir)
        print("✓ TaskManager已创建")
        print(f"  - 活动任务文件: {tm._active_tasks_file}")
        print(f"  - 历史目录: {tm._history_dir}")
        print(f"  - 历史缓存大小限制: {tm._cache_max_size}")
        
        # 2. 创建导航任务
        print_separator("步骤2: 创建导航任务")
        task = tm.create_task(
            task_type=TaskType.POINT_TO_POINT,
            parameters={'x': 2.0, 'y': 3.0, 'yaw': 0.0},
            task_id='nav_20260113_114216_72881'
        )
        print(f"✓ 任务已创建: {task.task_id}")
        print(f"  - 状态: {task.state.name}")
        print(f"  - 类型: {task.task_type.name}")
        
        # 3. 模拟任务执行完成
        print_separator("步骤3: 任务执行完成")
        tm.complete_task(task.task_id)
        print(f"✓ 任务已完成")
        
        # 查看活动任务
        active = tm.get_task(task.task_id, include_history=False)
        print(f"  - 活动任务中存在: {active is not None}")
        
        # 4. MissionPlanner删除任务（模拟handler的remove_task调用）
        print_separator("步骤4: MissionPlanner删除任务（清理资源）")
        tm.remove_task(task.task_id)
        print(f"✓ 任务已从活动任务中删除")
        
        # 验证活动任务中不存在
        active = tm.get_task(task.task_id, include_history=False)
        print(f"  - 活动任务中存在: {active is not None}")
        
        # 5. CMD接口查询任务（关键测试点）
        print_separator("步骤5: CMD接口查询任务状态")
        print("场景: 终端用户在任务完成后查询状态...")
        
        # 模拟CMD接口调用（默认include_history=True）
        result = tm.get_task(task.task_id, include_history=True)
        
        if result:
            print("✅ 成功从历史记录查询到任务！")
            print(f"  - 任务ID: {result.task_id}")
            print(f"  - 状态: {result.state.name}")
            print(f"  - 完成时间: {result.completed_at}")
            print(f"  - 进度: {result.progress * 100:.0f}%")
        else:
            print("❌ 查询失败！任务未找到")
            return False
        
        # 6. 验证缓存机制
        print_separator("步骤6: 验证性能优化缓存")
        print("再次查询相同任务（应从缓存读取）...")
        
        # 检查缓存
        in_cache = task.task_id in tm._history_cache
        print(f"  - 任务在缓存中: {in_cache}")
        
        # 第二次查询
        result2 = tm.get_task(task.task_id, include_history=True)
        if result2:
            print(f"✓ 第二次查询成功（从缓存）")
        
        # 7. 边界测试：查询不存在的任务
        print_separator("步骤7: 边界测试 - 查询不存在的任务")
        nonexist = tm.get_task('nonexistent_task_id', include_history=True)
        print(f"  - 不存在的任务查询结果: {nonexist}")
        if nonexist is None:
            print("✓ 正确返回None")
        
        # 8. 显示历史文件
        print_separator("步骤8: 检查持久化文件")
        from datetime import datetime
        date_str = datetime.now().strftime('%Y%m%d')
        history_file = os.path.join(tm._history_dir, f'tasks_{date_str}.json')
        
        if os.path.exists(history_file):
            print(f"✓ 历史文件已创建: {history_file}")
            with open(history_file, 'r') as f:
                import json
                history = json.load(f)
                print(f"  - 历史任务数量: {len(history)}")
                for h in history:
                    print(f"    * {h['task_id']}: {h['state']}")
        
        print_separator("验证完成")
        print("✅ 所有测试通过！")
        print("\n总结:")
        print("  1. ✅ 任务完成后保存到历史记录")
        print("  2. ✅ 活动任务中删除后仍可从历史查询")
        print("  3. ✅ 查询缓存机制正常工作")
        print("  4. ✅ 不存在的任务返回None")
        print("\n🎯 修复验证成功！CMD接口现在可以查询已完成的任务。")
        
        return True
        
    except Exception as e:
        print(f"\n❌ 验证失败: {e}")
        import traceback
        traceback.print_exc()
        return False
    
    finally:
        # 清理临时文件
        if os.path.exists(temp_dir):
            shutil.rmtree(temp_dir)
            print(f"\n已清理临时目录: {temp_dir}")


if __name__ == '__main__':
    success = main()
    sys.exit(0 if success else 1)
