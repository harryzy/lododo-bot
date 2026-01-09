# Web控制界面文档导航

**项目状态**: ✅ 框架代码已生成，进入测试阶段  
**最后更新**: 2026-01-08

---

## 📖 文档清单（7个核心文档）

### 1. [WEB_DESIGN.md](WEB_DESIGN.md) - 完整设计文档 ⭐ **必读**
包含77项决策的完整设计方案，重点章节：
- **§ 2.2.1**: bot_cmd_interface 统一接口架构（核心设计原则）⚠️
- 系统架构、技术栈选型
- 功能模块设计、通信协议设计
- 实施路线图

### 2. [ARCHITECTURE_NOTES.md](ARCHITECTURE_NOTES.md) - 架构核心要点 ⚠️ **强制阅读**
避免架构偏离的关键要点：
- bot_cmd_interface 统一接口铁律
- 禁止事项清单（直接调用 ROS 服务）
- 正确实现模式（SDK + 话题通信）
- 命令流程图

### 3. [UI_MOCKUP.md](UI_MOCKUP.md) - UI设计规范
详细的界面设计说明：
- 布局设计（三栏结构）
- 组件设计（MapView, TaskControl, WaypointEditor）
- 交互状态、颜色、图标规范

### 4. [API.md](API.md) - API接口文档
完整的REST API和WebSocket协议定义：
- HTTP API端点（任务/地图/路点管理）
- WebSocket消息格式
- rosbridge集成方案

### 5. [IMPLEMENTATION_PLAN.md](IMPLEMENTATION_PLAN.md) - 实施计划
14天开发计划（已提前完成框架）：
- **阶段0**: 依赖安装（必须在测试前完成）⚠️
- 阶段1-4: 按天分解的任务清单
- 当前状态: 已完成代码生成，进入测试阶段

### 6. [QUICKSTART.md](QUICKSTART.md) - 快速启动指南
5分钟启动Web界面：
- 环境要求检查
- 依赖安装（一键脚本）
- 启动完整环境
- 访问界面验证

### 7. [TROUBLESHOOTING.md](TROUBLESHOOTING.md) - 故障排除
常见问题解决方案：
- 依赖安装失败
- ROS连接问题
- 前端构建错误

---

## 🚀 快速开始

### Step 0: 安装依赖（必须先完成）⚠️

```bash
# 1. Python 后端依赖
source ~/lododo_bot/venv_web/bin/activate
pip install -r ~/lododo_bot/src/bot_teleop/requirements_web.txt

# 2. 前端依赖
cd ~/lododo_bot/src/bot_teleop/web_frontend
npm install

# 3. ROS2 依赖
sudo apt install ros-humble-rosbridge-server ros-humble-web-video-server
```

**详细说明**: 见 [IMPLEMENTATION_PLAN.md § 阶段0](IMPLEMENTATION_PLAN.md#阶段0-依赖安装与环境验证)

### Step 1: 启动环境

```bash
ros2 launch bot_bringup simulation_web_full.launch.py \
  slam:=false map_name:=exploration_test gui:=false
```

### Step 2: 访问界面

打开浏览器：http://localhost:8000

---

## ⚠️ 架构核心原则

**所有任务控制命令必须通过 bot_cmd_interface 统一接口**：

```
Web前端 → REST API → WebTerminalNode → /cmd/request → CommandAdapter → MissionPlanner
```

**禁止直接调用**:
- ❌ 在 API 层直接调用 `/mission/start_exploration` 服务
- ❌ 在任何模块中导入 `bot_navigation_msgs` 服务定义
- ❌ 绕过 WebTerminalNode 直接发布到 `/cmd/request`

**详细说明**: 见 [ARCHITECTURE_NOTES.md](ARCHITECTURE_NOTES.md)

---

## 📂 目录结构

```
bot_teleop/
├── docs/                          # 文档目录（当前）
│   ├── WEB_DESIGN.md              # 完整设计文档
│   ├── ARCHITECTURE_NOTES.md      # 架构核心要点 ⚠️
│   ├── UI_MOCKUP.md               # UI设计规范
│   ├── API.md                     # API接口文档
│   ├── IMPLEMENTATION_PLAN.md     # 实施计划
│   ├── QUICKSTART.md              # 快速启动
│   ├── TROUBLESHOOTING.md         # 故障排除
│   └── README.md                  # 本文档
│
├── web/                           # 后端代码
│   └── backend/
│       ├── web_server.py          # FastAPI 主应用
│       ├── websocket_handler.py   # WebSocket 管理器
│       ├── api/                   # REST API 端点
│       │   ├── tasks.py           # 任务管理（已修正架构）
│       │   ├── maps.py            # 地图管理
│       │   ├── waypoints.py       # 路点管理（已修正架构）
│       │   └── settings.py        # 设置管理
│       └── nodes/                 # ROS2 节点
│           ├── web_terminal_node.py  # WebTerminalNode（核心）
│           └── ros_bridge.py      # ROS 话题桥接
│
├── web_frontend/                  # 前端代码
│   ├── src/
│   │   ├── components/            # React 组件
│   │   ├── services/              # API & WebSocket 客户端
│   │   ├── stores/                # 状态管理（zustand）
│   │   └── locales/               # 国际化（中英双语）
│   ├── package.json
│   └── vite.config.ts
│
├── scripts/                       # Shell 脚本
│   ├── setup_web_env.sh           # 环境搭建
│   ├── start_web_server.sh        # 启动服务器
│   └── dev_rebuild.sh             # 快速重建
│
├── launch/                        # Launch 文件
│   └── web_server.launch.py
│
├── config/                        # 配置文件
│   └── web_config.yaml
│
├── requirements_web.txt           # Python 依赖
└── package.xml                    # ROS2 功能包定义
```

---

**重要提示**:
1. ⚠️ **必须先完成依赖安装**（阶段0），否则无法启动
2. ⚠️ **所有控制命令必须通过 bot_cmd_interface**，严禁直接调用服务
3. ⚠️ **开发前阅读 ARCHITECTURE_NOTES.md**，避免架构偏离

---

**项目状态**: 🎉 框架完成，等待依赖安装和测试验证  
**预计完整可用**: 2026-01-15（1周后）
