#!/bin/bash
# Web控制界面启动脚本
# 用途: 启动 FastAPI 后端服务器

set -e

# 颜色输出
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

echo -e "${GREEN}======================================"
echo -e "  LeKiwi Robot Web Server"
echo -e "======================================${NC}"
echo ""

# 获取项目根目录
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
TELEOP_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
PROJECT_ROOT="$(cd "$TELEOP_DIR/../.." && pwd)"

# 激活虚拟环境
if [ -d "$PROJECT_ROOT/venv_ros2" ]; then
    echo -e "${GREEN}[1/4]${NC} 激活虚拟环境..."
    source "$PROJECT_ROOT/venv_ros2/bin/activate"
else
    echo -e "${YELLOW}警告: 虚拟环境未找到，使用系统 Python${NC}"
fi

# Source ROS2
echo -e "${GREEN}[2/4]${NC} 加载 ROS2 环境..."
source /opt/ros/humble/setup.bash
source "$PROJECT_ROOT/install/setup.bash"

# 初始化 ROS2
echo -e "${GREEN}[3/4]${NC} 初始化 ROS2..."
if ! pgrep -x "ros2" > /dev/null; then
    # 确保 ROS2 daemon 运行
    ros2 daemon start 2>/dev/null || true
fi

# 启动 Web 服务器
echo -e "${GREEN}[4/4]${NC} 启动 Web 服务器..."
echo ""
echo -e "${GREEN}访问地址:${NC}"
echo -e "  http://localhost:8000      - REST API"
echo -e "  http://localhost:3000      - Web UI (需要单独启动前端)"
echo ""
echo -e "${YELLOW}按 Ctrl+C 停止服务器${NC}"
echo ""

cd "$TELEOP_DIR"

# 使用 uvicorn 启动（使用虚拟环境中的 python 绝对路径）
"$PROJECT_ROOT/venv_ros2/bin/python" -m uvicorn web.backend.web_server:app \
    --host 0.0.0.0 \
    --port 8000 \
    --reload \
    --log-level info
