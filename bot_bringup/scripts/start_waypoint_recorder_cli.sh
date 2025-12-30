#!/bin/bash
##############################################################################
# 航点录制CLI启动脚本 / Waypoint Recorder CLI Launch Script
#
# 功能 / Features:
#   - 启动交互式航点录制CLI
#   - 使用 waypoint_recorder 节点的 CLI 模式
#   - 提供友好的用户界面
#
# 使用方法 / Usage:
#   ./start_waypoint_recorder_cli.sh
#   ./start_waypoint_recorder_cli.sh --persistence-dir ~/my_waypoints
#
# 注意 / Note:
#   - 需要先启动导航系统（simulation_mission_planner.launch.py 或类似）
#   - CLI 模式提供交互式命令界面
#   - 录制的航点会保存到指定目录
#
# Author: LeKiwi Bot Development Team
# Date: 2025-12-30
##############################################################################

set -e  # 遇到错误立即退出

# ============================================================================
# 颜色定义 / Color Definitions
# ============================================================================
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
CYAN='\033[0;36m'
NC='\033[0m' # No Color

# ============================================================================
# 配置 / Configuration
# ============================================================================
DEFAULT_PERSISTENCE_DIR="${HOME}/lododo_bot/waypoints"
PERSISTENCE_DIR="${DEFAULT_PERSISTENCE_DIR}"

# ============================================================================
# 解析命令行参数 / Parse Command Line Arguments
# ============================================================================
while [[ $# -gt 0 ]]; do
    case $1 in
        --persistence-dir)
            PERSISTENCE_DIR="$2"
            shift 2
            ;;
        -h|--help)
            echo "Usage: $0 [OPTIONS]"
            echo ""
            echo "Options:"
            echo "  --persistence-dir DIR    指定航点保存目录 (默认: ~/lododo_bot/waypoints)"
            echo "  -h, --help              显示此帮助信息"
            echo ""
            echo "Examples:"
            echo "  $0"
            echo "  $0 --persistence-dir ~/my_waypoints"
            exit 0
            ;;
        *)
            echo -e "${RED}Unknown option: $1${NC}"
            echo "Use -h or --help for usage information"
            exit 1
            ;;
    esac
done

# ============================================================================
# 创建目录 / Create Directory
# ============================================================================
mkdir -p "${PERSISTENCE_DIR}"

# ============================================================================
# 打印启动信息 / Print Startup Information
# ============================================================================
echo -e "${CYAN}╔════════════════════════════════════════════════════════════════╗${NC}"
echo -e "${CYAN}║                                                                ║${NC}"
echo -e "${CYAN}║            🗺️  航点录制CLI / Waypoint Recorder CLI             ║${NC}"
echo -e "${CYAN}║                                                                ║${NC}"
echo -e "${CYAN}╚════════════════════════════════════════════════════════════════╝${NC}"
echo ""
echo -e "${BLUE}📁 航点保存目录 / Waypoint Directory:${NC}"
echo -e "   ${GREEN}${PERSISTENCE_DIR}${NC}"
echo ""
echo -e "${BLUE}🎮 CLI 命令 / CLI Commands:${NC}"
echo -e "   ${YELLOW}record${NC}              - 录制当前位置为航点"
echo -e "   ${YELLOW}list${NC}                - 列出所有已录制航点"
echo -e "   ${YELLOW}delete <index>${NC}      - 删除指定索引的航点"
echo -e "   ${YELLOW}save <filename>${NC}     - 保存航点到文件"
echo -e "   ${YELLOW}load <filename>${NC}     - 从文件加载航点"
echo -e "   ${YELLOW}clear${NC}               - 清除所有航点"
echo -e "   ${YELLOW}quit / exit${NC}         - 退出录制器"
echo ""
echo -e "${BLUE}⚠️  注意事项 / Important Notes:${NC}"
echo -e "   1. 确保导航系统已启动（定位正常）"
echo -e "   2. 使用键盘或手柄移动机器人到目标位置"
echo -e "   3. 在CLI中输入 ${YELLOW}record${NC} 录制当前位置"
echo -e "   4. 录制完成后输入 ${YELLOW}save <filename>${NC} 保存"
echo ""
echo -e "${CYAN}════════════════════════════════════════════════════════════════${NC}"
echo ""

# ============================================================================
# 检查 ROS2 环境 / Check ROS2 Environment
# ============================================================================
if [ -z "$ROS_DISTRO" ]; then
    echo -e "${RED}❌ 错误: ROS2 环境未配置${NC}"
    echo -e "${YELLOW}请先运行: source /opt/ros/humble/setup.bash${NC}"
    exit 1
fi

echo -e "${GREEN}✓ ROS2 环境已配置: $ROS_DISTRO${NC}"
echo ""

# ============================================================================
# 检查必要的话题 / Check Required Topics
# ============================================================================
echo -e "${BLUE}🔍 检查导航系统状态...${NC}"

# 检查定位话题
if timeout 3 ros2 topic info /rtabmap/localization_pose > /dev/null 2>&1; then
    echo -e "${GREEN}✓ RTABMap 定位话题可用${NC}"
    POSE_TOPIC="/rtabmap/localization_pose"
elif timeout 3 ros2 topic info /odometry/filtered > /dev/null 2>&1; then
    echo -e "${GREEN}✓ EKF 定位话题可用${NC}"
    POSE_TOPIC="/odometry/filtered"
else
    echo -e "${YELLOW}⚠️  警告: 定位话题不可用，请确保导航系统已启动${NC}"
    echo -e "${YELLOW}   建议先运行: ros2 launch bot_bringup simulation_mission_planner.launch.py${NC}"
    echo ""
    read -p "是否继续启动？(y/N): " -n 1 -r
    echo
    if [[ ! $REPLY =~ ^[Yy]$ ]]; then
        echo -e "${RED}已取消${NC}"
        exit 0
    fi
    POSE_TOPIC="/rtabmap/localization_pose"
fi

echo ""

# ============================================================================
# 启动航点录制器 / Launch Waypoint Recorder
# ============================================================================
echo -e "${CYAN}════════════════════════════════════════════════════════════════${NC}"
echo -e "${GREEN}🚀 启动航点录制CLI...${NC}"
echo -e "${CYAN}════════════════════════════════════════════════════════════════${NC}"
echo ""

# 启动 waypoint_recorder 节点（CLI 模式）
ros2 run bot_navigation waypoint_recorder \
    --ros-args \
    -p persistence_dir:="${PERSISTENCE_DIR}" \
    -p pose_topic:="${POSE_TOPIC}"

# ============================================================================
# 退出清理 / Exit Cleanup
# ============================================================================
echo ""
echo -e "${CYAN}════════════════════════════════════════════════════════════════${NC}"
echo -e "${YELLOW}📝 航点录制器已退出${NC}"
echo -e "${CYAN}════════════════════════════════════════════════════════════════${NC}"
echo ""
echo -e "${BLUE}录制的航点保存在:${NC}"
echo -e "   ${GREEN}${PERSISTENCE_DIR}${NC}"
echo ""
echo -e "${BLUE}使用巡航任务测试航点:${NC}"
echo -e "   ${YELLOW}ros2 service call /mission/start_patrol \\${NC}"
echo -e "   ${YELLOW}  bot_navigation_msgs/srv/StartPatrol \\${NC}"
echo -e "   ${YELLOW}  \"{waypoint_file: '${PERSISTENCE_DIR}/your_route.yaml', mode: 'loop'}\"${NC}"
echo ""
