#!/bin/bash
# 前端开发服务器启动脚本

set -e

GREEN='\033[0;32m'
NC='\033[0m'

echo -e "${GREEN}======================================"
echo -e "  Lododo Robot Web UI (Dev)"
echo -e "======================================${NC}"
echo ""

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
TELEOP_DIR="$(cd "$SCRIPT_DIR/.." && pwd)"
FRONTEND_DIR="$TELEOP_DIR/web_frontend"

if [ ! -d "$FRONTEND_DIR/node_modules" ]; then
    echo -e "${GREEN}安装前端依赖...${NC}"
    cd "$FRONTEND_DIR"
    npm install
fi

echo -e "${GREEN}启动前端开发服务器...${NC}"
echo ""
echo -e "${GREEN}访问地址:${NC} http://localhost:3000"
echo ""

cd "$FRONTEND_DIR"
npm run dev
