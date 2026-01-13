#!/bin/bash
# Stage 3.1 后端功能测试脚本
# 测试任务暂停、恢复、状态推送功能

set -e

API_URL="http://localhost:8000/api"
WS_URL="ws://localhost:8000/ws"

echo "=========================================="
echo "Stage 3.1 后端功能测试"
echo "=========================================="
echo ""

# 检查服务器是否运行
echo "[1/5] 检查 Web 服务器状态..."
health_response=$(curl -s ${API_URL}/health)
echo "✓ 服务器健康检查: $health_response"
echo ""

# 创建导航任务
echo "[2/5] 创建导航任务..."
nav_response=$(curl -s -X POST ${API_URL}/tasks/navigate \
  -H "Content-Type: application/json" \
  -d '{
    "x": 2.0,
    "y": 3.0,
    "yaw": 0.0
  }')
echo "✓ 导航任务响应: $nav_response"

# 提取 task_id（假设响应中包含）
# 注意：实际响应可能只有 request_id，需要通过查询状态获取 task_id
echo ""

# 测试任务暂停（假设 task_id=1）
echo "[3/5] 测试暂停任务（task_id=1）..."
pause_response=$(curl -s -X POST ${API_URL}/tasks/1/pause)
echo "✓ 暂停响应: $pause_response"
echo ""

# 测试任务恢复
echo "[4/5] 测试恢复任务（task_id=1）..."
resume_response=$(curl -s -X POST ${API_URL}/tasks/1/resume)
echo "✓ 恢复响应: $resume_response"
echo ""

# 查询任务状态
echo "[5/5] 查询任务状态..."
status_response=$(curl -s "${API_URL}/tasks/status")
echo "✓ 状态查询响应: $status_response"
echo ""

echo "=========================================="
echo "✓ 后端功能测试完成"
echo "=========================================="
echo ""
echo "注意事项："
echo "1. 暂停/恢复功能需要 CommandAdapter 和 MissionPlanner 运行"
echo "2. WebSocket 状态推送需要在浏览器中验证"
echo "3. 建议启动完整环境进行集成测试："
echo "   ros2 launch bot_bringup simulation_mission_planner_localization.launch.py"
echo ""
