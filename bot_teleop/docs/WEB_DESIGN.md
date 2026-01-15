# Robot Web控制界面设计方案（最终版）

**功能包**: bot_teleop  
**模块**: Web控制界面  
**版本**: v1.0.0-Final  
**创建日期**: 2026-01-08  
**最终确认**: 2026-01-08  
**依赖**: bot_cmd_interface v1.0.0  
**状态**: ✅ 设计完成，可开始实施

**重要说明**: 本文档整合了4轮需求分析（77项决策）的完整结果，包含所有技术细节、架构设计、实施计划。所有设计矛盾已解决，技术栈已确认，可直接用于代码实施。

---

## 1. 设计目标

### 1.1 核心目标

为机器人提供一个**基于浏览器的可视化控制界面**，使用户能够通过Web浏览器实现：

1. **可视化监控**: 实时查看地图、机器人位置、传感器数据、摄像头视频流
2. **交互式控制**: 点击地图导航（RViz风格Nav Goal Pose）、任务管理、路点编辑
3. **状态监控**: 任务进度、系统状态、历史记录、实时日志
4. **地图管理**: 地图列表、加载、保存、Costmap可视化
5. **国际化支持**: 中英双语界面切换

### 1.2 用户场景

**场景1：日常巡逻管理**
```
用户打开Web界面 → 选择已保存的巡逻路线 → 点击"开始巡逻" → 
实时监控机器人位置和任务进度 → 必要时暂停/恢复/取消任务
```

**场景2：临时导航任务**
```
用户打开地图视图 → 在地图上点击目标位置 → 确认导航 →
监控导航进度 → 机器人到达目标点
```

**场景3：路点录制与编辑**
```
用户启动路点录制模式 → 手动控制机器人移动到各个位置 →
点击"标记路点" → 保存路线 → 在地图上拖拽编辑路点位置
```

**场景4：地图管理**
```
用户查看地图库 → 选择要加载的地图 → 加载地图 →
启动探索建图 → 保存新地图到地图库
```

---

## 2. 系统架构

### 2.1 整体架构图

```
┌─────────────────────────────────────────────────────────────────────┐
│                         Web控制界面架构                               │
├─────────────────────────────────────────────────────────────────────┤
│                                                                     │
│  ┌───────────────────────────────────────────────────────────┐     │
│  │              浏览器端 (Browser Client)                     │     │
│  │  ┌─────────────────────────────────────────────────────┐  │     │
│  │  │  React/Vue.js 前端应用                              │  │     │
│  │  │  ┌─────────────┐  ┌──────────────┐  ┌──────────┐   │  │     │
│  │  │  │ 地图组件    │  │ 任务控制面板 │  │ 状态监控 │   │  │     │
│  │  │  │(Canvas+ROS) │  │              │  │          │   │  │     │
│  │  │  └─────────────┘  └──────────────┘  └──────────┘   │  │     │
│  │  │  ┌─────────────┐  ┌──────────────┐  ┌──────────┐   │  │     │
│  │  │  │ 路点编辑器  │  │ 地图管理     │  │ 日志查看 │   │  │     │
│  │  │  │             │  │              │  │          │   │  │     │
│  │  │  └─────────────┘  └──────────────┘  └──────────┘   │  │     │
│  │  └─────────────────────────────────────────────────────┘  │     │
│  └───────────────────────────────────────────────────────────┘     │
│                            │                                        │
│                   HTTP/WebSocket                                    │
│                            ↓                                        │
│  ┌───────────────────────────────────────────────────────────┐     │
│  │          Web服务器 (Web Server - Python)                  │     │
│  │  ┌─────────────────────────────────────────────────────┐  │     │
│  │  │  FastAPI / Flask                                    │  │     │
│  │  │  ┌──────────────┐  ┌──────────────┐               │  │     │
│  │  │  │ HTTP API     │  │ WebSocket    │               │  │     │
│  │  │  │ - 地图管理   │  │ - 实时通信   │               │  │     │
│  │  │  │ - 路点管理   │  │ - 状态推送   │               │  │     │
│  │  │  │ - 任务管理   │  │ - 事件通知   │               │  │     │
│  │  │  └──────────────┘  └──────────────┘               │  │     │
│  │  └─────────────────────────────────────────────────────┘  │     │
│  │                            │                               │     │
│  │  ┌─────────────────────────────────────────────────────┐  │     │
│  │  │  WebTerminalNode (ROS2节点)                         │  │     │
│  │  │  - 使用 bot_cmd_interface SDK                       │  │     │
│  │  │  - 发布请求到 /cmd/request                          │  │     │
│  │  │  - 订阅响应从 /cmd/response                         │  │     │
│  │  │  - 订阅状态话题 (/map, /rtabmap/localization_pose) │  │     │
│  │  └─────────────────────────────────────────────────────┘  │     │
│  └───────────────────────────────────────────────────────────┘     │
│                            ↓                                        │
│                  ROS2 Topics & Services                            │
│                            ↓                                        │
│  ┌───────────────────────────────────────────────────────────┐     │
│  │              bot_cmd_interface                            │     │
│  │              (统一命令接口层)                              │     │
│  │  /cmd/request  →  CommandAdapter  →  /cmd/response       │     │
│  └───────────────────────────────────────────────────────────┘     │
│                            ↓                                        │
│  ┌───────────────────────────────────────────────────────────┐     │
│  │           MissionPlanner (任务执行层)                      │     │
│  │  - 导航任务                                                │     │
│  │  - 探索任务                                                │     │
│  │  - 巡逻任务                                                │     │
│  └───────────────────────────────────────────────────────────┘     │
│                                                                     │
└─────────────────────────────────────────────────────────────────────┘
```

### 2.2 关键架构决策

#### 2.2.1 bot_cmd_interface 统一接口架构 ⚠️ **核心设计原则**

**🚨 架构铁律：所有控制命令必须通过 bot_cmd_interface 统一接口**

本项目**严格禁止** Web 后端直接调用 MissionPlanner、waypoint_recorder 等 ROS 服务。必须通过 `/cmd/request` 和 `/cmd/response` 话题进行交互。

**❌ 错误架构（禁止）**:
```python
# ❌ 错误：直接调用 ROS 服务
from rclpy.service import Client
mission_client = node.create_client(StartExploration, '/mission/start_exploration')
mission_client.call_async(request)  # 🚨 违反架构设计！
```

**✅ 正确架构（强制）**:
```python
# ✅ 正确：使用 bot_cmd_interface SDK
from bot_cmd_interface.sdk import create_exploration_request

class WebTerminalNode(Node):
    def __init__(self):
        self.cmd_publisher = self.create_publisher(String, '/cmd/request', 10)
        self.cmd_subscriber = self.create_subscription(String, '/cmd/response', self._response_callback, 10)
    
    def start_exploration(self, map_name: str, save_on_completion: bool):
        # 使用 SDK 创建请求
        request = create_exploration_request(
            map_name=map_name,
            save_on_completion=save_on_completion,
            request_id=self._generate_request_id("explore")
        )
        
        # 发布到 /cmd/request 话题
        msg = String()
        msg.data = request.to_json()
        self.cmd_publisher.publish(msg)
```

**命令流程图**:
```
┌─────────────┐
│  Web 前端   │  用户点击"开始导航"
└──────┬──────┘
       │ HTTP POST /api/tasks/navigate
       ↓
┌──────────────────────┐
│ FastAPI (tasks.py)   │  接收 HTTP 请求
└──────┬───────────────┘
       │ node = get_web_terminal_node()
       │ node.navigate_to_pose(x, y, yaw)
       ↓
┌─────────────────────────┐
│ WebTerminalNode (ROS2)  │  发布到 /cmd/request
└──────┬──────────────────┘
       │ JSON: {"action": "navigate_to_pose", "params": {...}}
       ↓
┌─────────────────────────┐
│ CommandAdapter          │  bot_cmd_interface 核心
└──────┬──────────────────┘
       │ 解析请求 → 调用后端服务
       │ client.call_async('/mission/navigate_to_pose')
       ↓
┌─────────────────────────┐
│ MissionPlanner          │  创建任务，发送 Nav2 目标
└──────┬──────────────────┘
       │ 返回 task_id: 1001
       ↓
┌─────────────────────────┐
│ CommandAdapter          │  发布响应到 /cmd/response
└──────┬──────────────────┘
       │ JSON: {"request_id": "web-nav-1", "status": "completed", "result": {"task_id": "task-001"}}
       ↓
┌─────────────────────────┐
│ WebTerminalNode         │  订阅 /cmd/response
└──────┬──────────────────┘
       │ response_callback(response)
       │ asyncio.create_task(self.callback(response))
       ↓
┌──────────────────────┐
│ WebSocket Manager    │  广播到所有连接的前端
└──────┬───────────────┘
       │ ws.send_json({"type": "cmd_response", ...})
       ↓
┌─────────────┐
│  Web 前端   │  更新任务状态："任务 #1001 已创建"
└─────────────┘
```

**SDK 使用规范**:

1. **请求创建（使用 SDK 工厂函数）**:
```python
from bot_cmd_interface.sdk import (
    create_navigate_request,
    create_exploration_request,
    create_patrol_request,
    create_get_status_request,
    create_cancel_task_request,
    create_emergency_stop_request
)

# 导航请求（SDK 自动生成 request_id）
request = create_navigate_request(x=2.0, y=3.0, yaw=0.0)

# 探索请求（SDK 自动生成 request_id）
request = create_exploration_request(map_name="office", save_on_completion=True)

# 巡逻请求（SDK 自动生成 request_id）
request = create_patrol_request(waypoint_file="route1.yaml", mode="loop")

# 状态查询（task_id 必须是字符串）
request = create_get_status_request(task_id="task-001")
```

2. **消息发布**:
```python
msg = String()
msg.data = request.to_json()  # SDK 自动序列化为 JSON
self.cmd_publisher.publish(msg)
```

3. **响应处理**:
```python
from bot_cmd_interface.sdk import CommandResponse

def _response_callback(self, msg: String):
    # SDK 自动反序列化
    response = CommandResponse.from_json(msg.data)
    
    # 检查状态（使用 SDK 提供的方法）
    if response.is_success():
        # 使用 result 字段（不是 data）
        task_id = response.result.get('task_id') if response.result else None
        print(f"Task created: {task_id}")
    elif response.status == ResponseStatus.FAILED:
        error_msg = response.message
        print(f"Command failed: {error_msg}")
```

**架构优势**:
- ✅ **服务隔离**: Web 层不知道 MissionPlanner 的存在
- ✅ **松耦合**: 后端服务可任意替换，Web 层无需修改
- ✅ **统一接口**: 所有用户端（Web/Voice/Mobile）使用同一套 API
- ✅ **异步响应**: 通过 WebSocket 实时推送任务状态
- ✅ **请求追踪**: 使用 `request_id` 匹配请求和响应

**禁止事项清单**:
- ❌ 在 `tasks.py` 中直接 `create_client('/mission/start_exploration')`
- ❌ 在 `waypoints.py` 中直接调用 `/waypoint_recorder/record_waypoint`
- ❌ 在任何 API 模块中导入 `bot_navigation_msgs` 的服务定义
- ❌ 绕过 WebTerminalNode 直接发布到 `/cmd/request`（必须通过节点方法）

**例外情况（允许直接操作）**:
- ✅ 文件读取：地图列表、路点文件加载（只读操作）
- ✅ 话题订阅：`/map`、`/rtabmap/localization_pose`（由 ros_bridge.py 处理，只读）
- ✅ 本地文件保存：路点文件写入 `~/lododo_bot/waypoints/`

---

#### 2.2.2 ROS通信层设计 ✅

**WebSocket双连接架构**:
- **FastAPI WebSocket** (`ws://localhost:8000/ws`): 用于命令发送和响应接收
- **rosbridge WebSocket** (`ws://localhost:9090`): 用于订阅ROS话题（地图、位姿、路径等）

**rosbridge启动策略**:
- ⚠️ **重要**: rosbridge在 `bot_bringup/launch/simulation_web_full.launch.py` 中统一启动
- ✅ **禁止**: 在 `simulation_cmd_interface_test.launch.py` 中启动rosbridge（避免修改现有文件）
- ✅ **禁止**: 在 `bot_teleop/launch/web_server.launch.py` 中启动rosbridge（避免端口冲突）

**Launch文件层级**:
```python
# bot_bringup/launch/simulation_web_full.launch.py
def generate_launch_description():
    return LaunchDescription([
        # 1. 启动基础环境（Gazebo + Nav2 + CommandAdapter）
        IncludeLaunchDescription(...simulation_cmd_interface_test...),
        
        # 2. 启动rosbridge（统一管理，避免冲突）
        ExecuteProcess(
            cmd=['ros2', 'launch', 'rosbridge_server', 
                 'rosbridge_websocket_launch.xml'],
            output='screen'
        ),
        
        # 3. 启动bot_teleop的Web环境（FastAPI + 前端）
        IncludeLaunchDescription(...web_server...),
    ])
```

**并发连接限制**: 
- FastAPI WebSocket: 最多3个连接（对应3个用户）
- rosbridge: 不限制（由rosbridge_server管理）

---

### 2.3 技术栈选型（已确认）

#### 前端技术栈 ✅

**已选方案：React + TypeScript + Ant Design**
```
React 18           - UI框架
TypeScript         - 类型安全
HTML5 Canvas       - 2D地图渲染（原生实现，无第三方依赖）
roslibjs           - ROS通信库（rosbridge WebSocket）
Ant Design v5      - UI组件库（使用默认亮色主题，简约风格）
zustand            - 状态管理（轻量级）
React Query        - 数据获取（可选）
react-i18next      - 国际化（中英双语，前端界面翻译）
Vite               - 构建工具
web_video_server   - 摄像头视频流（端口8080，MJPEG/H.264）
```

**UI主题决策**: 
- ✅ 使用Ant Design默认亮色主题（白色背景 + 蓝色主色调）
- ✅ 简约风格，无需特殊颜色处理
- ❌ 第一阶段不实现暗色主题（节省2.5天开发时间）
- 📌 暗色主题推迟到第二阶段（可选增强功能）

#### 后端技术栈 ✅

**已选方案：FastAPI + Python 3.10**
```
FastAPI            - 异步Web框架（生产模式，服务前端静态文件）
WebSockets         - 实时通信（FastAPI原生支持）
rclpy              - ROS2 Python客户端
bot_cmd_interface  - SDK集成（使用统一命令接口）
pydantic           - 数据验证
uvicorn            - ASGI服务器
```

**虚拟环境管理**:
- 创建独立虚拟环境 `~/lododo_bot/venv_web` （与ROS2环境隔离）
- 通过 `setup_web_env.sh` 脚本一键配置（创建venv、安装依赖、构建前端）
- 通过 `start_web_server.sh` 脚本启动FastAPI（自动激活venv）
- Launch文件调用 `start_web_server.sh` 启动服务

**前端构建工具**:
- `dev_rebuild.sh`: 开发快速重建脚本（npm build + colcon build）
- `build_frontend.sh`: 单独构建前端（输出到 `web_frontend/dist/`）
- ⚠️ **重要**: 前端修改后必须执行 `npm run build` 才能生效
- FastAPI服务前端静态文件（生产模式，无开发服务器）

#### 地图可视化 ✅

**已选方案：原生HTML5 Canvas手动渲染**

**技术决策理由**:
- ❌ **放弃ros2djs**: 该库已6年未更新（最后版本0.10.0, 2018年），与ROS2 Humble存在兼容性问题，在测试中出现无法读取地图origin的错误
- ✅ **采用Canvas原生实现**: 使用HTML5 Canvas API + ROSLIB.js手动渲染所有元素
- ✅ **完全控制**: 实现OccupancyGrid渲染、Costmap叠加、机器人标记、路径显示等所有功能
- ✅ **无依赖风险**: 不依赖过时的第三方库，代码完全可控
- ✅ **性能优化**: 可针对特定场景优化渲染逻辑

**实现内容**:
- **MapRenderer工具类**：处理OccupancyGrid → Canvas图像转换、坐标系转换、变换矩阵管理
- **Canvas地图渲染**：黑（障碍物）/白（自由空间）/灰（未知）三色渲染
- **Costmap叠加层**：半透明渲染膨胀区（橙色）和致命障碍物（红色）
- **机器人位置**：绘制蓝色箭头表示位置和朝向
- **交互控制**：鼠标缩放/平移、重置视图、跟随机器人
- **Nav Goal交互**：点击+拖拽设置导航目标（RViz风格）

**关键决策**: 第一阶段必须实现Costmap显示，否则无法支持导航建图功能

**代码量估算**: ~200行（MapRenderer工具类 + MapView组件），远少于集成ros2djs的复杂度

---

## 3. 功能模块设计

### 3.1 地图可视化模块

#### 3.1.1 功能需求

1. **实时地图显示**
   - 订阅 `/map` 话题（OccupancyGrid）
   - 灰度渲染（黑色=障碍物，白色=自由空间，灰色=未知）
   - 支持缩放、平移
   - **默认更新频率**: 1 Hz（可在 `config/web_config.yaml` 中配置）
   - ⚠️ **配置修改**: 需要重启FastAPI服务器才能生效（第一阶段不支持热重载）

2. **Costmap可视化（⚠️ 第一阶段必须实现）** ✅
   - 订阅 `/local_costmap/costmap` 和 `/global_costmap/costmap`
   - **Canvas手动渲染半透明叠加层**：
     - 使用独立的Canvas图层叠加在地图上
     - 解析OccupancyGrid数据（costmap值0-254）
     - 绘制膨胀区（橙色半透明，costmap值 > 0 且 < 100）：`rgba(255, 165, 0, 0.5)`
     - 绘制致命障碍物（红色半透明，costmap值 >= 100）：`rgba(255, 0, 0, 0.7)`
     - 使用OffscreenCanvas缓存提升性能
   - 图层控制：可切换显示/隐藏Costmap
   - **默认更新频率**: 5 Hz（在 `config/web_config.yaml` 中可配置）
   - **透明度**: 可配置（默认0.5）
   - **关键性**: 无Costmap显示，用户无法判断机器人安全区域，影响导航建图体验

3. **机器人位置追踪**
   - 订阅 `/rtabmap/localization_pose` 或 `/odom/filtered`
   - 显示机器人图标（方向箭头）
   - 实时更新位置
   - **默认更新频率**: 10 Hz（在 `config/web_config.yaml` 中可配置）

4. **路径显示**
   - 订阅 `/plan` 话题（全局路径）
   - 订阅 `/local_plan` 话题（局部路径）
   - 不同颜色区分（全局路径=绿色，局部路径=蓝色）
   - **默认更新频率**: 2 Hz（在 `config/web_config.yaml` 中可配置）

5. **路点显示**
   - 从后端获取路点列表
   - 在地图上标记路点（编号、名称）
   - 路点间连线显示顺序

6. **Nav Goal Pose交互（类似RViz）** ✅
   - **交互流程**：
     1. 点击工具栏"设置导航目标"按钮（图标：🎯）
     2. 鼠标变为十字准星
     3. 在地图上点击并拖拽绘制箭头
     4. 释放鼠标确认目标姿态
     5. 右键取消操作
   - **技术实现**：
     - `mousedown`：记录起点 (x, y)
     - `mousemove`：绘制绿色箭头表示朝向
     - `mouseup`：计算yaw角度，发送导航请求
   - **坐标转换**：像素坐标 → 地图坐标

7. **摄像头视频流显示** ✅
   - 使用`web_video_server`将ROS图像topic转为HTTP/WebSocket流
   - 支持多个图像源：
     - 原始摄像头图像（如`/camera/color/image_raw`）
     - 自定义处理图像（如YOLO标注后的图像）
   - **配置方式**：
     - 配置文件定义默认显示的topic
     - Web界面支持动态添加/删除图像源
     - 自动发现可用图像topic列表
   - **布局**：右下角固定区域，可折叠
   - 图像流质量可配置（分辨率、压缩率）

#### 3.1.2 技术实现

**配置文件格式** (`config/web_config.yaml`):
```yaml
update_rates:
  map: 1.0              # 地图更新频率 (Hz)
  pose: 10.0            # 机器人位姿更新频率 (Hz)
  costmap: 5.0          # Costmap更新频率 (Hz)
  planned_path: 2.0     # 规划路径更新频率 (Hz)

costmap:
  opacity: 0.5          # 透明度 (0.0-1.0)
  inflation_color: "rgba(255, 165, 0, 0.5)"  # 膨胀区颜色
  lethal_color: "rgba(255, 0, 0, 0.5)"       # 致命障碍物颜色

rosbridge:
  url: "ws://localhost:9090"
  reconnect_attempts: 5
  reconnect_interval: 5  # 秒
```

**前端实现示例**:
```typescript
// 地图组件伪代码 - Canvas实现
import * as ROSLIB from 'roslib';

class MapViewer {
  private canvas: HTMLCanvasElement;
  private ctx: CanvasRenderingContext2D;
  private ros: ROSLIB.Ros;
  private mapData: OccupancyGridData | null = null;
  private transform = { scale: 1, offsetX: 0, offsetY: 0 };
  
  constructor(canvasId: string) {
    // 获取Canvas元素
    this.canvas = document.getElementById(canvasId) as HTMLCanvasElement;
    this.ctx = this.canvas.getContext('2d')!;
    
    // 连接rosbridge（从配置文件加载URL）
    this.ros = new ROSLIB.Ros({
      url: 'ws://localhost:9090'  // rosbridge地址
    });
    
    // 订阅地图
    this.subscribeMap();
    this.subscribeRobotPose();
    this.subscribePath();
  }
  
  subscribeMap() {
    const mapTopic = new ROSLIB.Topic({
      ros: this.ros,
      name: '/map',
      messageType: 'nav_msgs/OccupancyGrid'
    });
    
    mapTopic.subscribe((message) => {
      // 渲染地图
      this.viewer.renderMap(message);
    });
  }
  
  // 点击事件
  onClick(x: number, y: number) {
    // 将像素坐标转换为地图坐标
    const mapCoord = this.viewer.pixelToMap(x, y);
    
    // 发送导航请求
    this.sendNavigationRequest(mapCoord.x, mapCoord.y);
  }
}
```

### 3.2 任务控制面板

#### 3.2.1 功能需求

1. **任务创建**
   - 导航任务：输入坐标或点击地图
   - 探索任务：选择地图名称
   - 巡逻任务：选择路点文件、选择模式

2. **任务控制**
   - 开始 / 暂停 / 恢复 / 取消按钮
   - 紧急停止按钮（红色，醒目）

3. **任务状态显示**
   - 当前任务信息（ID、类型、状态）
   - 进度条（如果有进度信息）
   - 预计剩余时间

4. **任务历史**
   - 列表显示最近10个任务
   - 状态：成功 / 失败 / 取消
   - 点击查看详情

#### 3.2.2 UI设计（伪代码）

```typescript
interface TaskControlProps {
  currentTask: Task | null;
  onStartTask: (taskType: TaskType, params: any) => void;
  onPauseTask: (taskId: number) => void;
  onResumeTask: (taskId: number) => void;
  onCancelTask: (taskId: number) => void;
  onEmergencyStop: () => void;
}

function TaskControlPanel({ currentTask, ...handlers }: TaskControlProps) {
  return (
    <div className="task-control-panel">
      {/* 任务创建表单 */}
      <TaskCreationForm onSubmit={handlers.onStartTask} />
      
      {/* 当前任务状态 */}
      {currentTask && (
        <CurrentTaskStatus
          task={currentTask}
          onPause={handlers.onPauseTask}
          onResume={handlers.onResumeTask}
          onCancel={handlers.onCancelTask}
        />
      )}
      
      {/* 紧急停止按钮 */}
      <EmergencyStopButton onClick={handlers.onEmergencyStop} />
      
      {/* 任务历史 */}
      <TaskHistory />
    </div>
  );
}
```

### 3.3 路点管理模块

#### 3.3.1 功能需求

1. **路点列表**
   - 显示所有已保存的路点文件
   - 文件名、路点数量、创建时间
   - 预览、编辑、删除操作

2. **路点录制**
   - 启动录制模式
   - 手动标记路点（记录当前位置）
   - 自动录制（距离/时间触发）
   - 保存到YAML文件

3. **路点编辑**
   - 可视化显示路点
   - 拖拽修改位置
   - 添加/删除路点
   - 修改路点名称、停留时间

4. **路点导入/导出**
   - 从文件导入
   - 导出为JSON/YAML

#### 3.3.2 数据结构

```typescript
interface Waypoint {
  name: string;
  x: number;
  y: number;
  yaw: number;
  dwell_time: number;  // 停留时间（秒）
}

interface WaypointFile {
  filename: string;
  waypoints: Waypoint[];
  created_at: string;
  updated_at: string;
  metadata?: {
    description?: string;
    tags?: string[];
  };
}
```

### 3.4 地图管理模块

#### 3.4.1 功能需求

1. **地图列表**
   - 显示所有保存的地图
   - 地图名称、尺寸、创建时间
   - 缩略图预览

2. **地图操作**
   - ✅ 加载地图（切换定位地图）- **实现状态**: 方案2-增强版（返回launch命令）
   - ✅ 删除地图
   - ✅ 重命名地图
   - ⚠️ **技术限制说明**: RTABMap无法在运行时动态切换地图，必须重启launch文件

3. **地图保存**
   - 探索完成后保存地图
   - 输入地图名称和描述

4. **地图元数据管理**
   - ✅ 添加标签（如：office, warehouse）
   - ✅ 添加描述

#### 3.4.2 API设计

```python
# FastAPI端点

@app.get("/api/maps")
async def list_maps() -> List[MapInfo]:
    """获取地图列表"""
    pass

@app.post("/api/maps/{map_name}/load")
async def load_map(map_name: str):
    """加载地图（返回launch命令）
    
    ⚠️ 技术限制 (2026-01-15):
    - RTABMap的地图加载需要在launch文件启动时指定 map_name:=... 参数
    - 无法在运行时动态切换地图（需要重启rtabmap节点）
    - 当前实现: 返回完整的launch命令供用户在终端执行
    
    🔮 未来改进方向:
    - 研究RTABMap的动态地图加载机制
    - 可能的方案: rosservice call /rtabmap/reset + 重新加载数据库
    - 或者: 实现自动重启rtabmap节点的功能（需要launch文件重构）
    """
    # 返回launch命令而非立即加载
    return {
        "requires_restart": True,
        "launch_command": f"ros2 launch ... map_name:={map_name}",
        "note": "RTABMap limitation: map switching requires restart"
    }

@app.post("/api/maps/{map_name}/save")
async def save_map(map_name: str, metadata: MapMetadata):
    """保存地图"""
    request = create_save_map_request(map_name)
    await terminal_node.send_request(request)
    return {"status": "saving"}

@app.delete("/api/maps/{map_name}")
async def delete_map(map_name: str):
    """删除地图"""
    pass

@app.patch("/api/maps/{map_name}/rename")
async def rename_map(map_name: str, new_name: str):
    """重命名地图（更新目录名和map_library.yaml）"""
    pass

@app.patch("/api/maps/{map_name}/metadata")
async def update_metadata(map_name: str, description: str, tags: List[str]):
    """更新地图元数据"""
    pass
```

### 3.5 状态监控模块

#### 3.5.1 功能需求

1. **机器人状态**
   - 位置（x, y, yaw）
   - 速度（vx, vy, vyaw）
   - 电池电量（如果有传感器）

2. **系统状态**
   - ROS节点状态（绿色=正常，红色=异常）
   - 传感器状态（摄像头、IMU）
   - 网络延迟

3. **任务进度**
   - 当前任务进度百分比
   - 已完成/总数（巡逻路点）
   - 预计剩余时间

4. **日志查看**
   - 实时日志流（WebSocket推送）
   - 日志级别过滤（INFO / WARN / ERROR）
   - 搜索和筛选

---

## 4. 通信协议设计

### 4.1 WebSocket消息格式

#### 4.1.1 客户端 → 服务器（请求）

```json
{
  "type": "command",
  "request_id": "web-20260108-001",
  "timestamp": "2026-01-08T12:00:00Z",
  "command": {
    "action": "navigate_to_pose",
    "params": {
      "x": 2.0,
      "y": 3.0,
      "yaw": 0.0
    }
  }
}
```

#### 4.1.2 服务器 → 客户端（响应）

```json
{
  "type": "response",
  "request_id": "web-20260108-001",
  "timestamp": "2026-01-08T12:00:01Z",
  "status": "completed",
  "data": {
    "task_id": 123,
    "message": "Task created successfully"
  }
}
```

#### 4.1.3 服务器 → 客户端（状态推送）

```json
{
  "type": "status_update",
  "timestamp": "2026-01-08T12:00:05Z",
  "data": {
    "robot_pose": {"x": 1.5, "y": 2.3, "yaw": 0.5},
    "task_status": {
      "task_id": 123,
      "state": "RUNNING",
      "progress": 0.45
    }
  }
}
```

### 4.2 HTTP REST API设计

```
GET    /api/maps                # 获取地图列表
POST   /api/maps/{name}/load    # 加载地图
POST   /api/maps/{name}/save    # 保存地图
DELETE /api/maps/{name}         # 删除地图

GET    /api/waypoints           # 获取路点文件列表
POST   /api/waypoints           # 创建路点文件
GET    /api/waypoints/{name}    # 获取路点详情
PUT    /api/waypoints/{name}    # 更新路点文件
DELETE /api/waypoints/{name}    # 删除路点文件

GET    /api/tasks               # 获取任务历史
GET    /api/tasks/{id}          # 获取任务详情
POST   /api/tasks/{id}/cancel   # 取消任务

GET    /api/status              # 获取系统状态
POST   /api/emergency_stop      # 紧急停止
```

---

## 5. 实现路线图

### 5.1 第一阶段：基础框架（3-4天）

**目标**: 搭建基本的前后端框架，实现简单的导航功能

#### 后端 (1-2天)
- [ ] 创建FastAPI项目结构
- [ ] 实现WebTerminalNode（集成bot_cmd_interface SDK）
- [ ] 实现WebSocket通信
- [ ] 实现基本的HTTP API（地图管理、任务管理）
- [ ] 单元测试

#### 前端 (2天)
- [ ] 创建React项目（Vite）
- [ ] 实现Canvas地图组件（MapRenderer工具类）
- [ ] 实现基本UI布局（地图区域、控制面板）
- [ ] 实现WebSocket连接（rosbridge）
- [ ] 实现点击地图导航（Canvas坐标转换）

### 5.2 第二阶段：核心功能（4-5天）

#### 任务控制面板 (2天)
- [ ] 任务创建表单（导航、探索、巡逻）
- [ ] 任务状态显示（实时更新）
- [ ] 任务控制按钮（暂停、恢复、取消）
- [ ] 任务历史记录

#### 路点管理 (2-3天)
- [ ] 路点列表展示
- [ ] 路点可视化（地图上标记）
- [ ] 路点录制功能
- [ ] 路点编辑（拖拽、修改属性）
- [ ] 路点导入/导出

### 5.3 第三阶段：高级功能（3-4天）

#### 地图管理 (1-2天)
- [ ] 地图列表与预览
- [ ] 地图加载/保存
- [ ] 地图元数据管理

#### 状态监控 (1-2天)
- [ ] 机器人状态面板
- [ ] 实时日志查看
- [ ] 系统健康检查

#### UI优化 (1天)
- [ ] 响应式布局（桌面端优化，暂不支持移动端）
- [ ] 交互动画（按钮点击、页面切换）
- [ ] 国际化完善（中英双语界面）

### 5.4 第四阶段：测试与优化（2-3天）

- [ ] 集成测试（前后端联调）
- [ ] 性能测试（WebSocket延迟、地图渲染）
- [ ] 压力测试（多客户端连接）
- [ ] 用户体验测试
- [ ] 文档编写

---

## 6. 目录结构设计

```
bot_teleop/
├── bot_teleop/
│   ├── __init__.py
│   ├── keyboard_teleop.py         # 现有键盘控制
│   └── web/
│       ├── __init__.py
│       ├── web_server.py          # FastAPI主应用
│       ├── websocket_handler.py   # WebSocket处理
│       ├── api/                   # REST API端点
│       │   ├── __init__.py
│       │   ├── maps.py            # 地图管理API
│       │   ├── waypoints.py       # 路点管理API
│       │   └── tasks.py           # 任务管理API
│       └── nodes/
│           ├── __init__.py
│           └── web_terminal_node.py  # ROS2节点（集成SDK）
├── web_frontend/                  # 前端项目（独立目录）
│   ├── package.json
│   ├── tsconfig.json
│   ├── vite.config.ts
│   ├── public/
│   ├── src/
│   │   ├── main.tsx
│   │   ├── App.tsx
│   │   ├── components/
│   │   │   ├── MapViewer/        # 地图组件
│   │   │   ├── TaskControl/      # 任务控制
│   │   │   ├── WaypointManager/  # 路点管理
│   │   │   ├── MapManager/       # 地图管理
│   │   │   └── StatusMonitor/    # 状态监控
│   │   ├── hooks/                # React Hooks
│   │   ├── services/             # API服务
│   │   ├── types/                # TypeScript类型
│   │   └── utils/                # 工具函数
│   └── dist/                     # 构建输出
├── scripts/
│   ├── setup_web_env.sh          # 环境搭建脚本（首次运行）
│   ├── start_web_server.sh       # 启动FastAPI服务器
│   ├── build_frontend.sh         # 构建前端
│   └── dev_rebuild.sh            # 开发快速重建（npm + colcon）
├── launch/
│   └── web_server.launch.py      # Web服务器启动（调用start_web_server.sh）
├── config/
│   ├── web_config.yaml           # Web服务器配置（更新频率、rosbridge等）
│   └── user_preferences.yaml     # 用户偏好设置（服务端存储）
├── docs/
│   ├── WEB_DESIGN.md             # 本文件
│   ├── WEB_QUESTIONS.md          # 需求明确文档
│   ├── API.md                    # API文档
│   └── FRONTEND_GUIDE.md         # 前端开发指南
├── test/
│   ├── test_web_server.py
│   └── test_websocket.py
├── package.xml
└── setup.py
```

---

## 7. 依赖项

### 7.1 Python依赖

```txt
# requirements_web.txt
fastapi>=0.104.0
uvicorn[standard]>=0.24.0
websockets>=12.0
pydantic>=2.0
python-multipart>=0.0.6
aiofiles>=23.2.1
```

### 7.2 ROS2依赖

```xml
<!-- package.xml -->
<depend>rclpy</depend>
<depend>std_msgs</depend>
<depend>geometry_msgs</depend>
<depend>nav_msgs</depend>
<depend>bot_navigation_msgs</depend>
<depend>bot_cmd_interface</depend>
```

### 7.3 前端依赖

```json
{
  "dependencies": {
    "react": "^18.2.0",
    "react-dom": "^18.2.0",
    "roslibjs": "^1.3.0",
    "antd": "^5.0.0",
    "zustand": "^4.4.0",
    "@tanstack/react-query": "^5.0.0",
    "axios": "^1.6.0"
  },
  "devDependencies": {
    "@types/react": "^18.2.0",
    "typescript": "^5.0.0",
    "vite": "^5.0.0"
  }
}
```

**注意**: 已移除 `ros2djs` 依赖，改用原生Canvas实现地图渲染。

---

## 8. 性能目标

| 指标 | 目标值 | 说明 |
|------|--------|------|
| 地图渲染帧率 | ≥15 FPS | 流畅的地图更新 |
| WebSocket延迟 | <100ms | 实时性要求 |
| 首次加载时间 | <3秒 | 用户体验 |
| 内存占用（浏览器） | <200MB | 资源控制 |
| 并发连接数 | 3个用户 | FastAPI WebSocket限制为3个连接 |
| rosbridge连接 | 不限制 | 由rosbridge_server管理 |

---

## 9. 安全考虑

1. **身份认证**（可选）
   - JWT token认证
   - 会话管理

2. **权限控制**
   - 只读模式（只能查看，不能控制）
   - 管理员模式（完全控制）

3. **输入验证**
   - 坐标范围检查
   - 参数类型验证
   - 防止注入攻击

4. **网络安全**
   - HTTPS（生产环境，可选）
   - WebSocket加密（WSS，生产环境可选）
   - **CORS策略** ⚠️:
     - ✅ **生产模式**: FastAPI服务前端静态文件，前后端同源，**不需要CORS**
     - ✅ **代码保留**: CORS配置在代码中注释掉，方便未来扩展（如需要第三方应用连接）
     - ❌ **禁用原因**: 同源策略更安全，避免跨域攻击
     - 📌 **启用场景**: 如果未来需要从其他域名访问API或从第三方应用连接WebSocket，可解除CORS注释

**CORS配置示例**（默认注释）:
```python
# bot_teleop/web/web_server.py
from fastapi.middleware.cors import CORSMiddleware

# CORS配置（默认禁用）
# 如需启用，取消以下注释并配置允许的源
# origins = [
#     "http://localhost:3000",      # 开发环境前端
#     "http://192.168.1.100:8000",  # 其他设备访问
# ]
# app.add_middleware(
#     CORSMiddleware,
#     allow_origins=origins,
#     allow_credentials=True,
#     allow_methods=["*"],
#     allow_headers=["*"],
# )
```

---

## 10. 第二阶段优化方向（可选）

1. **暗色主题支持** 🌙
   - Ant Design暗色算法
   - 地图canvas深灰背景
   - Costmap颜色调整（适应暗色背景）
   - 预计增加1-2天开发时间

2. **移动端适配** 📱
   - 响应式布局优化（适配平板）
   - 触摸手势支持（地图缩放、拖拽）
   - PWA支持（离线可用）

3. **高级功能** 🚀
   - 录制与回放（轨迹）
   - 多机器人支持（WebSocket命名空间）
   - 数据统计与报表

4. **性能优化**
   - 地图瓦片化（大地图）
   - WebGL渲染（Three.js）
   - 增量更新（减少数据传输）

---

## 附录：技术选型对比

### A1. 前端框架对比

| 特性 | React | Vue.js |
|------|-------|--------|
| 学习曲线 | 陡峭 | 平缓 |
| TypeScript支持 | 优秀 | 良好 |
| 生态系统 | 丰富 | 丰富 |
| 性能 | 优秀 | 优秀 |
| 社区活跃度 | 极高 | 高 |
| ROS集成 | 成熟 | 成熟 |

**推荐**: React（企业级应用首选，TypeScript支持更好）

### A2. 后端框架对比

| 特性 | FastAPI | Flask | Express.js |
|------|---------|-------|------------|
| 异步支持 | 原生 | 需扩展 | 原生 |
| 性能 | 极高 | 中等 | 高 |
| WebSocket | 原生 | 需扩展 | 需库 |
| 类型检查 | Pydantic | 无 | TypeScript |
| 文档生成 | 自动 | 手动 | 手动 |
| ROS2集成 | rclpy | rclpy | rclnodejs |

**推荐**: FastAPI（现代化、高性能、与bot_cmd_interface（Python）集成方便）

### A3. 地图可视化对比

| 特性 | Canvas原生 | ros2djs | Leaflet | Three.js |
|------|-----------|---------|---------|----------|
| 学习成本 | 低 | 低 | 中 | 高 |
| ROS集成 | 手动实现 | 原生（已过时） | 需适配 | 需适配 |
| 2D地图 | 优秀 | 优秀（不维护） | 优秀 | 一般 |
| 3D可视化 | 不支持 | 不支持 | 不支持 | 优秀 |
| 性能 | 极高 | 良好 | 优秀 | 中等 |
| 可维护性 | 优秀（无依赖） | 差（2018年停更） | 优秀 | 中等 |
| 代码量 | ~200行 | 0（依赖库） | 适配层>300行 | 适配层>500行 |

**推荐**: **Canvas原生实现**
- ros2djs已6年未更新，与ROS2 Humble不兼容
- 原生Canvas提供完全控制，无第三方依赖风险
- ~200行代码即可实现OccupancyGrid渲染、坐标转换、缩放平移

---

## 11. 完整决策汇总

### 11.1 核心架构决策（22项）

| 决策点 | 选择 | 理由 |
|--------|------|------|
| 前端框架 | React 18 + TypeScript | 类型安全，生态丰富，企业级应用首选 |
| UI库 | Ant Design v5 | 组件丰富，中文友好，默认亮色主题 |
| 后端框架 | FastAPI | 异步高性能，WebSocket原生支持，Pydantic数据验证 |
| 地图可视化 | Canvas原生 + 自定义Costmap | 完全控制，无第三方依赖，手动渲染Costmap叠加层 |
| 视频流方案 | web_video_server | ROS标准方案，MJPEG/H.264编码，端口8080 |
| 状态管理 | Zustand | 轻量级，TypeScript友好 |
| 国际化 | react-i18next | 成熟方案，支持中英双语 |
| 部署模式 | 生产模式 | FastAPI服务前端静态文件，无开发服务器 |
| 用户认证 | 不需要 | 单用户场景，最多3连接 |
| 移动端支持 | 桌面端优先 | 第一阶段不支持移动端 |
| 开发周期 | 2周MVP | 14天完成核心功能 |

### 11.2 UI/交互细节（17项）

| 决策点 | 选择 | 说明 |
|--------|------|------|
| 视频流配置 | 配置文件默认 + Web界面动态添加 | 灵活性与易用性平衡 |
| 视频流布局 | 右下角固定区域，可折叠 | 不遮挡主地图区域 |
| Costmap显示 | 第一阶段必须实现 | 手动渲染半透明canvas叠加层 |
| Nav Goal Pose | 点击+拖拽箭头（类似RViz） | 工具栏按钮激活，十字准星光标 |
| 国际化切换 | 右上角下拉菜单 | 手动切换，不自动检测 |
| 翻译范围 | 前端界面文本 | 后端数据保留原文 |
| 顶部导航栏高度 | 60px | 标准高度 |
| 右侧面板宽度 | 400px | 适中宽度 |
| 品牌名称 | Robot | 不使用LeKiwi |
| 虚拟环境 | 独立venv | 与ROS2环境隔离 |
| WebSocket重连 | 自动重连，最多5次 | 分层提示策略 |
| 错误提示 | Modal对话框 | 需用户确认 |
| 并发限制 | 拒绝第4个连接 | 前3个正常服务 |
| 性能监控 | 完整功能（不区分模式） | 调试面板右侧抽屉 |

### 11.3 配置与部署（14项）

| 决策点 | 选择 | 说明 |
|--------|------|------|
| 前端部署 | FastAPI静态文件服务 | 生产模式，前后端同源 |
| Launch启动 | bringup统一launch | 引用bot_teleop launch |
| rosbridge启动 | bringup的simulation_web_full.launch.py | 避免修改现有文件 |
| 配置文件格式 | YAML | web_config.yaml |
| 配置加载 | API动态加载 | 启动时读取 |
| 配置修改 | 需重启FastAPI | 第一阶段不支持热重载 |
| 调试面板 | 右侧抽屉500px | 4区域：性能/CMD/日志/ROS |
| ROS状态显示 | 简化（成功/失败） | 不显示topic详情 |
| 断线重连策略 | 分层（<5s静默，5-30s提示，>30s停止） | 地图冻结，按钮禁用 |
| 用户偏好存储 | 服务端存储 | 有保存按钮，主动保存 |
| 操作历史 | 不需要 | 第一阶段不实现 |
| 翻译文件组织 | 按模块（common/map/task/waypoint/debug/error） | 便于维护 |
| 后端数据翻译 | 不翻译 | 保留原文（中英混合） |

### 11.4 技术实现细节（15+6+3项）

| 决策点 | 选择 | 说明 |
|--------|------|------|
| 生产模式 | 仅生产模式 | 无开发模式 |
| 环境搭建 | 手动执行setup脚本 | 非自动化 |
| Costmap渲染 | 基础canvas绘制 | 解析OccupancyGrid，手动叠加 |
| 配置文件格式 | YAML | server端存储 |
| Ant Design版本 | v5 | 最新稳定版 |
| 状态管理库 | Zustand | 轻量级 |
| WebSocket架构 | 双连接 | FastAPI + rosbridge |
| Python依赖管理 | requirements.txt锁定版本 | 确保一致性 |
| 脚本封装 | dev_rebuild.sh | npm build + colcon build |
| 更新频率 | map=1Hz, pose=10Hz, costmap=5Hz, path=2Hz | 可配置 |
| 暗色主题 | 推迟到第二阶段 | 第一阶段使用简约亮色 |
| npm scripts | 标准配置 + build:watch + clean | 满足基本需求 |
| CORS | 不需要 | 生产模式同源，代码中保留注释 |

### 11.5 关键约束

1. **rosbridge启动**: 只在 `bot_bringup/launch/simulation_web_full.launch.py` 中启动，不得修改 `simulation_cmd_interface_test.launch.py`
2. **前端构建**: 每次修改后必须执行 `npm run build`，否则不生效
3. **配置修改**: 需要重启FastAPI服务器才能生效（第一阶段不支持热重载）
4. **并发限制**: FastAPI WebSocket最多3个连接，rosbridge不限制
5. **Costmap**: 第一阶段必须实现，手动canvas渲染，否则影响导航建图体验
6. **暗色主题**: 第一阶段不实现，节省2.5天开发时间
7. **CORS**: 不启用，代码中注释掉，方便未来扩展
8. **移动端**: 第一阶段不支持，桌面端优先

---

**文档结束 - 设计完成，可开始实施！** 🚀

**下一步**: 开始生成7份详细文档和项目脚手架代码
