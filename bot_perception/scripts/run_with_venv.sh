#!/bin/bash
# 通用脚本：在虚拟环境中运行感知节点

# 获取工作空间根目录
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"

# 从 install 目录定位工作空间根目录
# 脚本位于: install/bot_perception/share/bot_perception/scripts/run_with_venv.sh
# 需要返回5级: scripts -> bot_perception -> share -> bot_perception -> install -> workspace_root
WS_DIR="$( cd "$SCRIPT_DIR/../../../../.." && pwd )"

# 虚拟环境路径（在工作空间根目录）
VENV_PATH="$WS_DIR/venv_ros2/bin/activate"

# 激活虚拟环境并设置Python路径
if [ -f "$VENV_PATH" ]; then
    source "$VENV_PATH"
    # 将虚拟环境的site-packages添加到PYTHONPATH开头（优先级最高）
    export PYTHONPATH="$WS_DIR/venv_ros2/lib/python3.10/site-packages:$PYTHONPATH"
else
    echo "错误: 虚拟环境未找到于 $VENV_PATH" >&2
    echo "工作空间: $WS_DIR" >&2
    echo "脚本位置: $SCRIPT_DIR" >&2
    exit 1
fi

# 执行传入的命令（通常是ros2节点）
exec "$@"
