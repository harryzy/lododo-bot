#!/bin/bash
# Universal script: run perception nodes in virtual environment / 通用脚本：在虚拟环境中运行感知节点

# 获取工作空间根目录
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

# Locate workspace root directory from install directory / 从 install 目录定位工作空间根目录
# Script location: install/bot_perception/share/bot_perception/scripts/run_with_venv.sh / 脚本位于: install/bot_perception/share/bot_perception/scripts/run_with_venv.sh
# Need to go back 5 levels: scripts -> bot_perception -> share -> bot_perception -> install -> workspace_root / 需要返回5级: scripts -> bot_perception -> share -> bot_perception -> install -> workspace_root
WS_DIR="$( cd "$SCRIPT_DIR/../../../../.." && pwd )"

# Virtual environment path (in workspace root directory) / 虚拟环境路径（在工作空间根目录）
VENV_PATH="$WS_DIR/venv_ros2/bin/activate"

# Activate virtual environment and set Python path / 激活虚拟环境并设置Python路径
if [ -f "$VENV_PATH" ]; then
    source "$VENV_PATH"
    # 将虚拟环境的site-packages添加到PYTHONPATH开头（优先级最高）
    export PYTHONPATH="$WS_DIR/venv_ros2/lib/python3.10/site-packages:$PYTHONPATH"
else
    echo "Error: Virtual environment not found at $VENV_PATH" >&2
    echo "Workspace: $WS_DIR" >&2
    echo "Script location: $SCRIPT_DIR" >&2
    exit 1
fi

# Execute passed command (usually ros2 node) / 执行传入的命令（通常是ros2节点）
exec "$@"
