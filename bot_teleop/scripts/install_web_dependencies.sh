#!/bin/bash
# Web控制界面依赖安装脚本
# 用途: 一键安装所有Python、Node.js和ROS2依赖
# 版本: v1.0.0
# 日期: 2026-01-08

set -e  # 遇到错误立即退出

# 颜色输出
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# 日志函数
log_info() {
    echo -e "${GREEN}[INFO]${NC} $1"
}

log_warn() {
    echo -e "${YELLOW}[WARN]${NC} $1"
}

log_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

# 获取项目根目录
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
TELEOP_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
PROJECT_ROOT="$(cd "$TELEOP_DIR/../.." && pwd)"

log_info "项目根目录: $PROJECT_ROOT"
log_info "bot_teleop目录: $TELEOP_DIR"

echo ""
echo "======================================"
echo "  Web控制界面依赖安装"
echo "======================================"
echo ""

# ============================================
# 阶段 1: Python 后端依赖
# ============================================
echo ""
log_info "【阶段 1/4】安装 Python 后端依赖..."
echo ""

# 检查是否已有虚拟环境
if [ -d "$PROJECT_ROOT/venv_ros2" ]; then
    log_info "检测到现有虚拟环境: $PROJECT_ROOT/venv_ros2"
    VENV_PATH="$PROJECT_ROOT/venv_ros2"
else
    log_warn "未检测到虚拟环境，将创建新环境: $PROJECT_ROOT/venv_web"
    python3 -m venv "$PROJECT_ROOT/venv_web"
    VENV_PATH="$PROJECT_ROOT/venv_web"
fi

# 激活虚拟环境
log_info "激活虚拟环境..."
source "$VENV_PATH/bin/activate"

# 升级 pip
log_info "升级 pip..."
pip install --upgrade pip

# 创建临时 requirements 文件
TMP_REQ="/tmp/web_requirements_$$.txt"
cat > "$TMP_REQ" << 'EOF'
# Web服务器框架
fastapi==0.104.1
uvicorn[standard]==0.24.0

# WebSocket支持
websockets==12.0

# 数据验证
pydantic==2.5.0

# 配置文件解析
pyyaml==6.0.1

# 文件上传支持
python-multipart==0.0.6

# CORS支持（已包含在FastAPI中，但明确列出）
# starlette (FastAPI依赖，包含CORS)
EOF

log_info "安装 Python 依赖包..."
pip install -r "$TMP_REQ"

# 验证安装
log_info "验证 Python 依赖..."
python3 << 'PYEOF'
import sys
try:
    import fastapi
    print(f"✓ FastAPI {fastapi.__version__}")
    
    import uvicorn
    print(f"✓ uvicorn {uvicorn.__version__}")
    
    import websockets
    print(f"✓ websockets {websockets.__version__}")
    
    import pydantic
    print(f"✓ pydantic {pydantic.__version__}")
    
    import yaml
    print(f"✓ PyYAML (yaml 模块)")
    
    print("\n✓ 所有 Python 依赖验证通过!")
except ImportError as e:
    print(f"✗ 导入失败: {e}", file=sys.stderr)
    sys.exit(1)
PYEOF

if [ $? -eq 0 ]; then
    log_info "Python 依赖安装完成 ✓"
else
    log_error "Python 依赖验证失败"
    exit 1
fi

rm -f "$TMP_REQ"

# ============================================
# 阶段 2: ROS2 依赖
# ============================================
echo ""
log_info "【阶段 2/4】验证 ROS2 依赖..."
echo ""

# Source ROS2
if [ -f "/opt/ros/humble/setup.bash" ]; then
    source /opt/ros/humble/setup.bash
else
    log_error "ROS2 Humble 未安装"
    exit 1
fi

# 验证 bot_cmd_interface
log_info "验证 bot_cmd_interface..."
if ros2 pkg list | grep -q "bot_cmd_interface"; then
    log_info "✓ bot_cmd_interface 已存在"
else
    log_error "bot_cmd_interface 包未找到，请先编译项目"
    exit 1
fi

# 检查 rosbridge_server
log_info "检查 rosbridge_server..."
if ros2 pkg list | grep -q "rosbridge_server"; then
    log_info "✓ rosbridge_server 已安装"
else
    log_warn "rosbridge_server 未安装，开始安装..."
    sudo apt update
    sudo apt install -y ros-humble-rosbridge-server
    
    if ros2 pkg list | grep -q "rosbridge_server"; then
        log_info "✓ rosbridge_server 安装成功"
    else
        log_error "rosbridge_server 安装失败"
        exit 1
    fi
fi

# 检查 web_video_server（可选）
log_info "检查 web_video_server（可选）..."
if ros2 pkg list | grep -q "web_video_server"; then
    log_info "✓ web_video_server 已安装"
else
    log_warn "web_video_server 未安装（可选组件）"
    read -p "是否安装 web_video_server? (y/N): " -n 1 -r
    echo
    if [[ $REPLY =~ ^[Yy]$ ]]; then
        sudo apt install -y ros-humble-web-video-server
        if ros2 pkg list | grep -q "web_video_server"; then
            log_info "✓ web_video_server 安装成功"
        else
            log_warn "web_video_server 安装失败（可跳过）"
        fi
    else
        log_warn "跳过 web_video_server 安装"
    fi
fi

log_info "ROS2 依赖验证完成 ✓"

# ============================================
# 阶段 3: Node.js 和前端依赖
# ============================================
echo ""
log_info "【阶段 3/4】安装前端依赖..."
echo ""

# 检查 Node.js
log_info "检查 Node.js..."
if command -v node &> /dev/null; then
    NODE_VERSION=$(node --version)
    log_info "✓ Node.js 已安装: $NODE_VERSION"
    
    # 检查版本是否 >= 18
    NODE_MAJOR=$(node --version | cut -d'v' -f2 | cut -d'.' -f1)
    if [ "$NODE_MAJOR" -lt 18 ]; then
        log_warn "Node.js 版本过低（需要 >= 18），当前: $NODE_VERSION"
        log_warn "请手动升级 Node.js: https://nodejs.org/"
        exit 1
    fi
else
    log_error "Node.js 未安装"
    log_info "安装指南:"
    log_info "  Ubuntu/Debian: curl -fsSL https://deb.nodesource.com/setup_18.x | sudo -E bash - && sudo apt install -y nodejs"
    log_info "  或访问: https://nodejs.org/"
    exit 1
fi

# 检查 npm
if command -v npm &> /dev/null; then
    NPM_VERSION=$(npm --version)
    log_info "✓ npm 已安装: $NPM_VERSION"
else
    log_error "npm 未安装（通常随Node.js一起安装）"
    exit 1
fi

# 创建 package.json（临时，后续代码生成会覆盖）
FRONTEND_DIR="$TELEOP_DIR/web_frontend"
mkdir -p "$FRONTEND_DIR"

# 由于我们还没有生成前端代码，这里先创建一个临时的 package.json 进行依赖预安装
if [ ! -f "$FRONTEND_DIR/package.json" ]; then
    log_info "创建临时 package.json..."
    cat > "$FRONTEND_DIR/package.json" << 'EOF'
{
  "name": "lododo-web-ui",
  "private": true,
  "version": "1.0.0",
  "type": "module",
  "scripts": {
    "dev": "vite",
    "build": "tsc && vite build",
    "preview": "vite preview"
  },
  "dependencies": {
    "react": "^18.2.0",
    "react-dom": "^18.2.0",
    "antd": "^5.11.5",
    "roslibjs": "^1.3.0",
    "zustand": "^4.4.7",
    "react-i18next": "^13.5.0",
    "i18next": "^23.7.8",
    "axios": "^1.6.2"
  },
  "devDependencies": {
    "@types/react": "^18.2.43",
    "@types/react-dom": "^18.2.17",
    "@vitejs/plugin-react": "^4.2.1",
    "typescript": "^5.3.3",
    "vite": "^5.0.8"
  }
}
EOF
fi

# 安装前端依赖
log_info "安装前端依赖（这可能需要几分钟）..."
cd "$FRONTEND_DIR"

# 使用 npm ci 如果有 package-lock.json，否则使用 npm install
if [ -f "package-lock.json" ]; then
    npm ci
else
    npm install
fi

# 验证关键包
log_info "验证前端依赖..."
npm list --depth=0 | grep -E "react|antd|roslibjs" || true

if [ -d "$FRONTEND_DIR/node_modules" ]; then
    log_info "前端依赖安装完成 ✓"
else
    log_error "前端依赖安装失败"
    exit 1
fi

# ============================================
# 阶段 4: 完整性测试
# ============================================
echo ""
log_info "【阶段 4/4】依赖完整性测试..."
echo ""

cd "$PROJECT_ROOT"

# 激活虚拟环境
source "$VENV_PATH/bin/activate"

# 测试脚本
TEST_RESULT=0

log_info "测试 Python 依赖..."
python3 << 'PYEOF'
import sys
try:
    from fastapi import FastAPI
    from uvicorn import run
    import websockets
    from pydantic import BaseModel
    import yaml
    print("✓ Python 依赖完整")
except ImportError as e:
    print(f"✗ Python 依赖缺失: {e}", file=sys.stderr)
    sys.exit(1)
PYEOF
[ $? -ne 0 ] && TEST_RESULT=1

log_info "测试 ROS2 依赖..."
source /opt/ros/humble/setup.bash
ros2 pkg list | grep -q "bot_cmd_interface" && echo "✓ bot_cmd_interface" || { echo "✗ bot_cmd_interface"; TEST_RESULT=1; }
ros2 pkg list | grep -q "rosbridge_server" && echo "✓ rosbridge_server" || { echo "✗ rosbridge_server"; TEST_RESULT=1; }

log_info "测试前端依赖..."
[ -d "$FRONTEND_DIR/node_modules" ] && echo "✓ node_modules 存在" || { echo "✗ node_modules 缺失"; TEST_RESULT=1; }

# ============================================
# 总结
# ============================================
echo ""
echo "======================================"
if [ $TEST_RESULT -eq 0 ]; then
    log_info "✓ 所有依赖安装成功！"
    echo ""
    log_info "下一步操作:"
    log_info "  1. 生成 Web 控制界面代码"
    log_info "  2. 运行测试: bash scripts/start_web_server.sh"
    echo ""
    log_info "环境信息:"
    log_info "  虚拟环境: $VENV_PATH"
    log_info "  前端目录: $FRONTEND_DIR"
    echo ""
else
    log_error "✗ 部分依赖安装失败，请检查上述错误"
    exit 1
fi
echo "======================================"
