# Robot Web控制界面实施计划

**版本**: v1.0.0  
**创建日期**: 2026-01-09  
**更新日期**: 2026-01-13  
**项目状态**: 🚧 开发中 - 阶段3已完成  
**预计工期**: 14天 (2周完成MVP)

---

## 🚨 关键架构约束：bot_cmd_interface 统一接口协议

### 核心设计原则

**⚠️ 强制规则 - 所有后端控制命令必须遵守以下架构**：

```
命令流设计（松耦合）:
  Web前端 → FastAPI HTTP API → WebTerminalNode 
    ↓ 使用SDK构造JSON请求
  /cmd/request Topic (std_msgs/String) 
    ↓
  CommandAdapter Node（统一接口层）
    ├─ 请求解析、验证、去重
    ├─ 队列管理（基于request_id）
    ├─ 调用 MissionPlanner 服务
    └─ 获得 task_id 后立即响应
    ↓
  /cmd/response Topic (std_msgs/String)
    ↓
  WebTerminalNode 订阅 → WebSocket → Web前端

关键点：
1. 后端不直接调用 MissionPlanner 服务
2. 通过 bot_cmd_interface SDK 构造请求
3. 使用 request_id 追踪请求-响应关联
4. CommandAdapter 短生命周期（~1秒）
5. 任务执行状态由 MissionPlanner 维护
```

### 禁止的错误模式

```python
# ❌ 严禁：在 Web 后端直接调用 ROS 服务
from bot_navigation_msgs.srv import StartExploration

mission_client = node.create_client(StartExploration, '/mission/start_exploration')
response = mission_client.call_async(request)  # 违反松耦合原则！
```

### 正确的实现模式

```python
# ✅ 正确：使用 bot_cmd_interface SDK

# 步骤1：导入 SDK
from bot_cmd_interface.sdk import (
    CommandRequest,
    CommandResponse,
    ActionType
)

# 步骤2：构造请求（SDK 自动生成 request_id）
request = CommandRequest(
    action=ActionType.START_EXPLORATION,
    params={
        "map_name": "office_floor1",
        "save_on_completion": True
    }
)

# 步骤3：发布到统一接口
msg = String()
msg.data = request.to_json()
self.cmd_publisher.publish(msg)  # 发布到 /cmd/request

# 步骤4：订阅响应（WebTerminalNode 自动处理）
# 响应通过 /cmd/response 返回，包含 task_id
```

### 短生命周期模型

**命令请求的生命周期（约1秒）**：
```
Time 0:    用户点击"开始探索"
Time 0.1:  FastAPI 接收请求
Time 0.2:  WebTerminalNode.start_exploration()
           ├─ 构造 CommandRequest
           └─ 发布到 /cmd/request

Time 0.3:  CommandAdapter 接收
           ├─ 发布 status="queued"
           └─ 加入队列

Time 0.5:  CommandAdapter 处理
           ├─ 发布 status="executing"
           ├─ 调用 /mission/start_exploration 服务
           └─ MissionPlanner 返回 task_id=123

Time 0.8:  CommandAdapter 发布最终响应
           ├─ status="completed"
           ├─ result={'task_id': 123}
           └─ 清理 request_id 状态 ⚠️

Time 1.0:  WebTerminalNode 接收响应
           ├─ 解析 task_id
           └─ 通过 WebSocket 推送到前端

⚠️ 命令请求生命周期结束
⚠️ 此后不再有该 request_id 的任何更新
```

**任务执行状态查询（新请求）**：
```
Time 10:   用户想知道进度 → 前端发起查询
Time 10.1: WebTerminalNode.query_task_status(task_id=123)
           ├─ 构造新的 CommandRequest（新的 request_id）
           ├─ action="get_task_status"
           ├─ params={'task_id': 123}
           └─ 发布到 /cmd/request

Time 10.5: CommandAdapter 处理查询
           ├─ 调用 /mission/get_task_status 服务
           ├─ MissionPlanner 返回: status=RUNNING, progress=0.5
           └─ 发布响应: result={'task_status': 'RUNNING', 'progress': 0.5}

Time 11:   前端接收响应 → 更新进度条显示 50%
```

### SDK 消息格式

**请求格式（/cmd/request）**：
```json
{
  "action": "start_exploration",
  "params": {
    "map_name": "office_floor1",
    "save_on_completion": true
  },
  "request_id": "exploration_20260112_103045",
  "source": "web_terminal",
  "timestamp": 1736659845.123
}
```

**响应格式（/cmd/response）**：
```json
{
  "request_id": "exploration_20260112_103045",
  "status": "completed",
  "message": "Exploration task created",
  "data": {
    "task_id": 123
  },
  "timestamp": 1736659846.456
}
```

### WebTerminalNode 公共接口

**已实现的方法**（位于 `web/backend/nodes/web_terminal_node.py`）：

```python
class WebTerminalNode:
    def navigate_to_pose(x: float, y: float, yaw: float) -> str:
        """返回 request_id"""
        
    def start_exploration(map_name: str, save_on_completion: bool) -> str:
        """返回 request_id"""
        
    def start_patrol(waypoint_file: str, mode: str) -> str:
        """返回 request_id"""
        
    def query_task_status(task_id: str) -> str:
        """返回 request_id（查询响应包含任务进度）"""
        
    def cancel_task(task_id: str) -> str:
        """返回 request_id"""
        
    def emergency_stop() -> str:
        """返回 request_id"""
```

### 暂停/恢复任务的实现方法

**⚠️ 关键发现：SDK 已定义 ActionType，但未提供便捷构造函数**

根据 bot_cmd_interface 文档分析：
- ✅ `ActionType.PAUSE_TASK` 和 `ActionType.RESUME_TASK` 已定义
- ❌ SDK 未提供 `create_pause_task_request()` / `create_resume_task_request()`
- ✅ 可以直接使用 `CommandRequest` 构造

**实现方案（不修改 SDK）**：

```python
# 在 WebTerminalNode 中添加新方法
def pause_task(self, task_id: str) -> str:
    """暂停任务"""
    request = CommandRequest(
        action=ActionType.PAUSE_TASK,
        params={'task_id': task_id},
        priority=2,
        timeout=5.0
    )
    return self._publish_request(request)

def resume_task(self, task_id: str) -> str:
    """恢复任务"""
    request = CommandRequest(
        action=ActionType.RESUME_TASK,
        params={'task_id': task_id},
        priority=2,
        timeout=5.0
    )
    return self._publish_request(request)
```

### 开发检查清单

在实现任何后端功能前，必须确认：

- [ ] ✅ 使用 `bot_cmd_interface.sdk` 导入 `CommandRequest` / `ActionType`
- [ ] ✅ 通过 WebTerminalNode 公共方法调用（不直接访问 ROS 服务）
- [ ] ✅ FastAPI 端点返回 `request_id`（不等待任务完成）
- [ ] ❌ 禁止导入 `bot_navigation_msgs.srv.*`
- [ ] ❌ 禁止使用 `node.create_client()`
- [ ] ❌ 禁止在响应中返回任务进度（进度通过查询获得）

---

## 项目概览

### 当前进度评估

**✅ 已完成部分**:
1. 基础项目结构创建
2. 依赖项安装 (Python: FastAPI/uvicorn/websockets, Node.js: React/Ant Design/TypeScript)
3. 后端基础框架 (FastAPI启动、WebTerminalNode骨架、基础API端点)
4. 前端基础框架 (React App、基础Layout、空壳组件)
5. Launch文件配置 (simulation_web_full.launch.py)
6. 启动脚本 (start_web_server.sh)

**❌ 缺失的核心功能**:
1. **地图可视化** - 没有ros2djs集成，没有真实ROS地图加载
2. **Costmap叠加** - 完全缺失（设计要求第一阶段必须实现）
3. **交互式导航** - 没有RViz风格的点击地图设置Nav Goal Pose
4. **rosbridge集成** - 前端没有连接rosbridge，无法订阅ROS话题
5. **地图管理** - 地图列表、加载、保存功能缺失
6. **路点管理** - 路点显示、编辑、录制功能缺失
7. **WebSocket实时推送** - 后端有WebSocket Handler但前端未实现订阅
8. **任务状态监控** - 没有实时任务进度显示

### 核心问题分析

**问题1**: 当前MapView只是静态canvas网格，没有任何ROS数据
- **原因**: 未集成ros2djs，未连接rosbridge
- **影响**: 地图无法显示，用户无法看到机器人位置和环境
- **解决**: 阶段1优先级最高

**问题2**: 没有Costmap可视化
- **原因**: 设计文档要求手动渲染，但未实现
- **影响**: 用户无法判断机器人安全区域，影响导航建图体验
- **解决**: 阶段2必须完成

**问题3**: 导航功能只有输入框，无法在地图上点击
- **原因**: 缺少Nav Goal Pose交互实现
- **影响**: 用户体验差，不符合RViz使用习惯
- **解决**: 阶段2必须完成

**问题4**: 任务控制面板功能不完整
- **原因**: 只有基础表单，没有状态显示、进度监控
- **影响**: 用户不知道任务是否成功执行
- **解决**: 阶段3完成

---

## 实施阶段划分

### 阶段0: 环境验证与修复 (0.5天)

**目标**: 确保现有代码可以正常启动，修复已知问题

**任务清单**:
- [ ] 0.1 验证依赖完整性
  - 检查 `package.json` 是否包含 `roslib`（正确的npm包名）
  - 检查 Python venv 是否包含所有依赖
  - 验证 rosbridge_server 已安装

- [ ] 0.2 修复后端启动问题
  - ✅ 已修复：添加 `rclpy.init()` 到 web_server.py
  - 测试 WebTerminalNode 是否正常创建
  - 测试 `/cmd/request` 和 `/cmd/response` 话题连接

- [ ] 0.3 验证前端构建
  - 测试 `npm run build` 是否成功
  - 测试 FastAPI 是否能服务静态文件
  - 验证 http://localhost:8000 可访问

- [ ] 0.4 测试完整启动流程
  ```bash
  # 终端1: 启动ROS环境
  ros2 launch bot_bringup simulation_web_full.launch.py slam:=false map_name:=exploration_test
  
  # 终端2: 启动Web后端
  bash src/bot_teleop/scripts/start_web_server.sh
  
  # 浏览器: 访问 http://localhost:8000
  ```

**验收标准**:
- ✅ 后端正常启动，WebTerminalNode 初始化成功
- ✅ 前端页面可访问，显示基础UI
- ✅ rosbridge 在 9090 端口运行
- ✅ 无控制台错误

**预计用时**: 4小时

---

### 阶段1: 地图可视化核心 (2天) ✅ 已完成

**目标**: 使用原生Canvas实现地图显示，让用户能看到真实的ROS地图和机器人位置
**实际用时**: 2天 | **完成日期**: 2026-01-12

**关键成果**:
- ✅ MapRenderer类实现完整Canvas渲染引擎（811行）
- ✅ ROSConnection服务封装rosbridge WebSocket
- ✅ 订阅/map、/localization_pose实时数据
- ✅ 机器人位置实时显示，地图缩放/平移交互
- ✅ 高DPI支持，像素级清晰度

**技术方案**: 
- ✅ **不使用ros2djs**（已过时，6年未更新，有兼容性问题）
- ✅ 使用**原生HTML5 Canvas + ROSLIB.js**手动渲染
- ✅ 优势：轻量级、无依赖、完全控制、性能好

#### 1.1 安装和配置ROSLIB (0.5天) ✅

**前端任务**:
- [x] 1.1.1 安装依赖 ✅
  ```bash
  cd web_frontend
  npm install roslib@2.0.1 eventemitter2@6.4.9
  ```

- [x] 1.1.2 创建 ROS 连接服务 ✅
  - 文件：`web_frontend/src/services/rosConnection.ts`
  - 功能：封装 rosbridge WebSocket 连接管理
  - 内容：连接、断线重连、错误处理

- [x] 1.1.3 更新配置文件 ✅
  - 在 `config/web_config.yaml` 中添加 rosbridge URL
  - 添加各话题更新频率配置

**验收标准**:
- ✅ 前端能成功连接 ws://localhost:9090
- ✅ 控制台无连接错误
- ✅ rosConnection.ts 提供统一接口

**实际用时**: 4小时 ✅

#### 1.2 实现Canvas地图渲染 (1天) ✅

**前端任务**:
- [x] 1.2.1 创建 MapRenderer 工具类 ✅
  - 文件：`web_frontend/src/utils/MapRenderer.ts`
  - 功能：
    - OccupancyGrid → Canvas 图像数据转换
    - 坐标系转换（ROS坐标 ↔ 屏幕坐标）
    - 缩放/平移变换矩阵管理

- [x] 1.2.2 重构 MapView 组件 ✅
  - 文件：`web_frontend/src/components/MapView/MapView.tsx`
  - 使用 Canvas 标签：
    ```tsx
    <canvas 
      ref={mapCanvasRef}
      width={800}
      height={600}
      style={{ cursor: 'grab' }}
    />
    ```

- [x] 1.2.3 订阅 /map 话题并渲染 ✅
  ```typescript
  // 伪代码
  const mapTopic = new ROSLIB.Topic({
    ros: rosConnection.getRos(),
    name: '/map',
    messageType: 'nav_msgs/OccupancyGrid'
  });

  mapTopic.subscribe((message) => {
    const imageData = convertOccupancyGridToImage(message);
    renderToCanvas(imageData, mapCanvasRef.current);
  });
  
  function convertOccupancyGridToImage(grid) {
    const width = grid.info.width;
    const height = grid.info.height;
    const imageData = new ImageData(width, height);
    
    for (let i = 0; i < grid.data.length; i++) {
      const value = grid.data[i];
      const pixelIndex = i * 4;
      
      if (value === -1) {
        // 未知区域：灰色
        imageData.data[pixelIndex] = 128;
        imageData.data[pixelIndex + 1] = 128;
        imageData.data[pixelIndex + 2] = 128;
      } else if (value === 0) {
        // 空闲：白色
        imageData.data[pixelIndex] = 255;
        imageData.data[pixelIndex + 1] = 255;
        imageData.data[pixelIndex + 2] = 255;
      } else {
        // 占用：黑色
        imageData.data[pixelIndex] = 0;
        imageData.data[pixelIndex + 1] = 0;
        imageData.data[pixelIndex + 2] = 0;
      }
      imageData.data[pixelIndex + 3] = 255; // Alpha
    }
    
    return imageData;
  }
  ```

- [x] 1.2.4 添加地图控制功能 ✅
  - 鼠标滚轮缩放（Ctrl+Wheel）
  - 鼠标拖拽平移
  - 重置视图按钮
  - 实现平滑缩放动画

**验收标准**:
- ✅ 地图正确显示（黑色=障碍物，白色=空闲，灰色=未知）
- ✅ 可以用鼠标缩放、平移地图
- ✅ 地图数据来自真实的ROS `/map` 话题
- ✅ 地图实时更新（1 Hz）
- ✅ 无第三方地图库依赖

**实际用时**: 8小时 ✅

#### 1.3 实现机器人位置显示 (0.5天) ✅

**前端任务**:
- [x] 1.3.1 添加机器人位姿监听 ✅
  ```typescript
  const poseTopic = new ROSLIB.Topic({
    ros: rosConnection.getRos(),
    name: '/rtabmap/localization_pose',
    messageType: 'geometry_msgs/PoseWithCovarianceStamped'
  });
  
  poseTopic.subscribe((message) => {
    updateRobotMarker(message.pose.pose);
  });
  ```

- [x] 1.3.2 在Canvas上绘制机器人 ✅
  ```typescript
  function drawRobot(ctx, x, y, theta) {
    // 转换到屏幕坐标
    const screenPos = rosToScreen(x, y);
    
    ctx.save();
    ctx.translate(screenPos.x, screenPos.y);
    ctx.rotate(-theta); // Canvas坐标系Y轴向下
    
    // 绘制箭头
    ctx.fillStyle = '#1890ff';
    ctx.beginPath();
    ctx.moveTo(20, 0);
    ctx.lineTo(-10, 10);
    ctx.lineTo(-10, -10);
    ctx.closePath();
    ctx.fill();
    
    ctx.restore();
  }
  ```

- [x] 1.3.3 添加机器人追踪功能 ✅
  - "跟随机器人"按钮
  - 自动将视图中心保持在机器人位置

**验收标准**:
- ✅ 机器人位置实时显示（10 Hz）
- ✅ 机器人朝向准确显示（蓝色箭头）
- ✅ "跟随机器人"功能正常工作

**实际用时**: 4小时 ✅

---

### 阶段2: 交互式导航与Costmap (2天) ✅ 已完成

**目标**: 实现RViz风格的点击地图导航，添加Costmap可视化
**实际用时**: 2天 | **完成日期**: 2026-01-12

**关键成果**:
- ✅ 交互式导航目标设置（点击+拖拽朝向，类似RViz 2D Nav Goal）
- ✅ Costmap半透明叠加（局部代价地图粉→红，全局代价地图蓝→绿→黄→红）
- ✅ 增量更新支持（订阅costmap_updates优化性能）
- ✅ MapToolbar工具栏（导航目标、清除、跟随、重置视图、Costmap开关）
- ✅ 发布到/goal_pose话题，与Nav2集成

#### 2.1 Nav Goal Pose交互 (1天) ✅

**前端任务**:
- [x] 2.1.1 添加工具栏 ✅
  - 文件：`web_frontend/src/components/MapView/MapToolbar.tsx`
  - 按钮：设置导航目标（🎯图标）、清除目标、重置视图

- [x] 2.1.2 实现点击+拖拽交互 ✅
  ```typescript
  // 伪代码
  let navGoalMode = false;
  let startPoint = null;
  
  // 点击"设置导航目标"按钮
  function activateNavGoalMode() {
    navGoalMode = true;
    mapCanvas.style.cursor = 'crosshair';
  }
  
  // 鼠标按下
  function onMouseDown(event) {
    if (!navGoalMode) return;
    startPoint = getMapCoordinates(event);
  }
  
  // 鼠标移动
  function onMouseMove(event) {
    if (!navGoalMode || !startPoint) return;
    const currentPoint = getMapCoordinates(event);
    drawArrow(startPoint, currentPoint); // 绘制绿色箭头
  }
  
  // 鼠标释放
  function onMouseUp(event) {
    if (!navGoalMode || !startPoint) return;
    const endPoint = getMapCoordinates(event);
    const yaw = calculateYaw(startPoint, endPoint);
    sendNavigationGoal(startPoint.x, startPoint.y, yaw);
    navGoalMode = false;
    mapCanvas.style.cursor = 'default';
  }
  ```

- [x] 2.1.3 实现坐标转换 ✅
  - 像素坐标 → 地图坐标（考虑地图原点、分辨率、缩放）
  - MapRenderer类提供rosToScreen/screenToRos方法

- [x] 2.1.4 发送导航请求 ✅
  - 发布到`/goal_pose`话题（geometry_msgs/PoseStamped）
  - 在地图上显示目标标记

**后端任务**:
- [x] 2.1.5 验证坐标有效性 ✅
  - 前端实时计算朝向并显示箭头
  - 发布到ROS话题，Nav2自动处理

**验收标准**:
- ✅ 点击工具栏按钮后鼠标变为十字准星
- ✅ 可以点击+拖拽绘制箭头
- ✅ 释放鼠标后发送导航请求
- ✅ 机器人开始导航到目标点

**实际用时**: 8小时 ✅

#### 2.2 Costmap可视化 (1天) ✅

**前端任务**:
- [x] 2.2.1 订阅Costmap话题 ✅
  - 订阅 `/local_costmap/costmap` 和 `/global_costmap/costmap`
  - 订阅增量更新 `/local_costmap/costmap_updates` 和 `/global_costmap/costmap_updates`

- [x] 2.2.2 实现手动渲染 ✅
  ```typescript
  // 伪代码
  function renderCostmap(costmapData, canvas, opacity) {
    const ctx = canvas.getContext('2d');
    const imageData = ctx.createImageData(costmapData.info.width, costmapData.info.height);
    
    for (let i = 0; i < costmapData.data.length; i++) {
      const cost = costmapData.data[i];
      const pixelIndex = i * 4;
      
      if (cost === 0) {
        // 自由空间 - 透明
        imageData.data[pixelIndex + 3] = 0;
      } else if (cost < 100) {
        // 膨胀区 - 橙色半透明
        imageData.data[pixelIndex] = 255;     // R
        imageData.data[pixelIndex + 1] = 165; // G
        imageData.data[pixelIndex + 2] = 0;   // B
        imageData.data[pixelIndex + 3] = opacity * 255;
      } else {
        // 致命障碍物 - 红色半透明
        imageData.data[pixelIndex] = 255;     // R
        imageData.data[pixelIndex + 1] = 0;   // G
        imageData.data[pixelIndex + 2] = 0;   // B
        imageData.data[pixelIndex + 3] = opacity * 255;
      }
    }
    
    ctx.putImageData(imageData, 0, 0);
  }
  ```

- [x] 2.2.3 叠加到地图上 ✅
  - MapRenderer在render()方法中渲染Costmap层
  - 使用半透明ImageData绘制
  - 自动同步缩放和平移

- [x] 2.2.4 添加图层控制 ✅
  - 切换按钮：显示/隐藏Costmap
  - MapToolbar中的Costmap开关
  - 可调透明度（默认50%）

**验收标准**:
- ✅ Costmap 半透明叠加在地图上
- ✅ 局部代价地图粉→红渐变，全局代价地图蓝→绿→黄→红
- ✅ 可以切换显示/隐藏图层
- ✅ Costmap 跟随地图缩放和平移
- ✅ 增量更新性能优化

**实际用时**: 8小时 ✅

---

### 阶段3: 任务管理完整功能 (2天) ✅ 已完成 (2026-01-14)

**目标**: 完善任务控制面板，实现任务状态实时监控

#### 3.1 任务状态实时显示 (1天) ✅

**前端任务**:
- [x] ✅ 3.1.1 重构 TaskControl 组件
  - 文件：`web_frontend/src/components/TaskControl/TaskControl.tsx`
  - 分离为子组件：TaskList组件实现活跃任务和历史任务管理

- [x] ✅ 3.1.2 实现 WebSocket 订阅
  - 连接 FastAPI WebSocket (ws://localhost:8000/ws)
  - 监听任务响应消息
  - 监听任务状态更新

- [x] ✅ 3.1.3 创建 TaskList 组件
  - 显示活跃任务信息（ID、类型、状态、进度）
  - 显示历史任务列表
  - 控制按钮：查询、暂停、恢复、取消

**后端任务**:
- [x] ✅ 3.1.4 实现任务状态推送
  - 修改 `web/backend/websocket_handler.py`
  - TaskManager监听/cmd/response自动推送
  - 广播任务进度更新到所有连接的客户端

- [x] ✅ 3.1.5 实现任务控制端点
  - `POST /api/tasks/{task_id}/pause` - 暂停任务 ✅
  - `POST /api/tasks/{task_id}/resume` - 恢复任务 ✅
  - `POST /api/tasks/{task_id}/cancel` - 取消任务 ✅
  - `POST /api/tasks/{task_id}/query` - 查询任务状态 ✅
  - 通过 WebTerminalNode 调用 bot_cmd_interface SDK

**关键修复**:
- [x] ✅ 修复task_id类型不一致问题（int→str）
  - pause/resume/query API统一使用str类型
  - 支持字符串格式task_id（如nav_20260113_xxx）

- [x] ✅ 修复导航参数格式兼容性
  - ServiceAdapter支持SDK标准格式（嵌套goal_pose）
  - ServiceAdapter支持扁平格式（向后兼容）
  - Web端正确使用create_navigate_request() SDK函数

- [x] ✅ 修复已完成任务查询功能
  - TaskManager.get_task()支持历史查询
  - 添加_get_task_from_history()方法
  - 实现LRU缓存优化性能（缓存100个历史任务）
  - 13个单元测试全部通过

- [x] ✅ 修复任务状态迁移逻辑
  - 已取消/失败任务自动移到历史列表
  - updateLocalTask()正确处理终态任务
  - 支持WebSocket推送和手动查询触发迁移

- [x] ✅ 修复取消状态显示问题
  - Web端TaskManager支持CANCELED和CANCELLED拼写
  - 正确映射ROS后端的canceled状态
  - 取消任务正确显示"已取消"并归档

**验收标准**:
- ✅ 创建任务后立即显示任务信息
- ✅ 任务状态实时更新（通过WebSocket推送）
- ✅ 进度条准确显示任务进度
- ✅ 可以暂停、恢复、取消任务
- ✅ 任务完成/失败/取消后自动移到历史列表
- ✅ 已完成任务可以从历史记录查询

**预计用时**: 8小时 → **实际用时**: 2天（包含问题修复）

#### 3.2 任务历史与探索/巡逻面板 (1天) ✅ 已完成

**前端任务**:
- [x] ✅ 3.2.1 实现 TaskHistory 组件
  - 显示最近10个任务
  - 列表项：任务ID、类型、状态、开始时间、耗时
  - 点击查看详情（Modal对话框）
  - 从localStorage加载历史数据

- [x] ✅ 3.2.2 完善探索任务面板
  - ExplorationPanel组件实现
  - 输入地图名称
  - 选择是否保存地图
  - 显示探索进度（百分比）
  - 实时监听当前探索任务状态

- [x] ✅ 3.2.3 完善巡逻任务面板
  - PatrolPanel组件实现
  - 选择路点文件（从后端获取列表）
  - 选择模式：单次/循环/往返
  - 显示当前路点进度（1/5）
  - 集成waypoints API

**后端任务**:
- [x] ✅ 3.2.4 路点文件列表API
  - `GET /api/waypoints` - 获取路点文件列表
  - 返回文件名、路点数量、创建时间
  - waypoints.py已实现完整CRUD接口

- [x] ✅ 3.2.5 探索/巡逻API（已存在）
  - `POST /api/tasks/exploration` - 创建探索任务
  - `POST /api/tasks/patrol` - 创建巡逻任务
  - 通过 WebTerminalNode 调用 bot_cmd_interface SDK

**验收标准**:
- ✅ 任务历史列表正确显示
- ✅ 可以创建探索任务并监控进度
- ✅ 可以创建巡逻任务并监控进度
- ✅ 所有任务记录保存到历史

**关键实现**:
- TaskHistory组件 - 历史任务展示和详情查看
- ExplorationPanel组件 - 探索任务创建和进度监控
- PatrolPanel组件 - 巡逻任务创建，路点文件选择
- apiService.waypoints.list() - 路点文件列表API
- Tabs界面布局 - 活跃任务、导航、探索、巡逻、历史
- 中英双语完整支持

**实际用时**: 1天 ✅ (2026-01-14)

---

### 阶段4: 地图管理功能 (1.5天)

**目标**: 实现地图列表、加载、保存功能

#### 4.1 地图列表与加载 (1天)

**前端任务**:
- [ ] 4.1.1 创建 MapManager 组件
  - 文件：`web_frontend/src/components/MapManager/MapManager.tsx`
  - 顶部导航栏"地图管理"菜单项

- [ ] 4.1.2 实现地图列表
  - 表格显示：地图名称、尺寸、创建时间、操作
  - 缩略图预览（可选）
  - 搜索和过滤

- [ ] 4.1.3 实现地图加载
  - 点击"加载"按钮
  - 确认对话框
  - 调用后端 API 加载地图

**后端任务**:
- [ ] 4.1.4 实现地图管理 API
  - `GET /api/maps` - 获取地图列表
  - 扫描 `~/lododo_bot/maps/` 目录
  - 读取 metadata.yaml（如果存在）

- [ ] 4.1.5 实现地图加载功能
  - `POST /api/maps/{map_name}/load` - 加载地图
  - 通过 bot_cmd_interface 发送加载地图请求
  - ⚠️ 注意：需要在 CommandAdapter 中添加对应的 action 支持

**验收标准**:
- ✅ 地图列表正确显示所有保存的地图
- ✅ 可以点击加载地图
- ✅ 加载后机器人定位到新地图

**预计用时**: 8小时

#### 4.2 地图保存与删除 (0.5天)

**前端任务**:
- [ ] 4.2.1 实现地图保存对话框
  - 输入地图名称
  - 输入描述（可选）
  - 添加标签（可选）

- [ ] 4.2.2 实现地图删除
  - 删除确认对话框
  - 调用后端 API

**后端任务**:
- [ ] 4.2.3 实现地图保存 API
  - `POST /api/maps/{map_name}/save` - 保存地图
  - 通过 bot_cmd_interface 触发地图保存
  - 保存 metadata.yaml

- [ ] 4.2.4 实现地图删除 API
  - `DELETE /api/maps/{map_name}` - 删除地图
  - 删除整个地图目录

**验收标准**:
- ✅ 探索完成后可以保存地图
- ✅ 可以为地图添加描述和标签
- ✅ 可以删除不需要的地图

**预计用时**: 4小时

---

### 阶段5: 路点管理功能 (2天)

**目标**: 实现路点录制、显示、编辑功能

#### 5.1 路点列表与显示 (1天)

**前端任务**:
- [ ] 5.1.1 创建 WaypointManager 组件
  - 文件：`web_frontend/src/components/WaypointManager/WaypointManager.tsx`
  - 顶部导航栏"路点管理"菜单项

- [ ] 5.1.2 实现路点列表
  - 表格显示：文件名、路点数量、创建时间、操作
  - 操作：查看、编辑、删除、导出

- [ ] 5.1.3 在地图上显示路点
  - 创建 WaypointMarkers 组件
  - 订阅路点数据
  - 在地图上绘制路点（编号、名称）
  - 路点间连线显示顺序

**后端任务**:
- [ ] 5.1.4 实现路点管理 API
  - `GET /api/waypoints` - 获取路点文件列表
  - `GET /api/waypoints/{filename}` - 获取路点详情
  - 扫描 `~/lododo_bot/waypoints/` 目录

**验收标准**:
- ✅ 路点列表正确显示
- ✅ 路点在地图上可视化
- ✅ 可以查看路点详细信息

**预计用时**: 8小时

#### 5.2 路点录制与编辑 (1天)

**前端任务**:
- [ ] 5.2.1 实现路点录制模式
  - "开始录制"按钮
  - 手动标记路点按钮（记录当前机器人位置）
  - "停止录制"并保存

- [ ] 5.2.2 实现路点编辑
  - 可拖拽修改路点位置
  - 修改路点名称
  - 修改停留时间
  - 添加/删除路点

- [ ] 5.2.3 实现路点导入/导出
  - 上传 YAML 文件
  - 下载为 YAML 文件

**后端任务**:
- [ ] 5.2.4 实现路点录制 API
  - `POST /api/waypoints/recording/start` - 开始录制
  - `POST /api/waypoints/recording/mark` - 标记路点
  - `POST /api/waypoints/recording/stop` - 停止并保存
  - 通过 bot_cmd_interface 调用 waypoint_recorder 服务

- [ ] 5.2.5 实现路点编辑 API
  - `PUT /api/waypoints/{filename}` - 更新路点文件
  - `DELETE /api/waypoints/{filename}` - 删除路点文件

**验收标准**:
- ✅ 可以录制路点
- ✅ 可以在地图上拖拽编辑路点位置
- ✅ 可以修改路点属性
- ✅ 可以导入/导出路点文件

**预计用时**: 8小时

---

### 阶段6: 状态监控与日志 (1.5天)

**目标**: 实现机器人状态监控、系统状态检查、实时日志

#### 6.1 机器人状态监控 (0.5天)

**前端任务**:
- [ ] 6.1.1 创建 StatusMonitor 组件
  - 文件：`web_frontend/src/components/StatusMonitor/StatusMonitor.tsx`
  - 布局：右侧面板或底部面板

- [ ] 6.1.2 实现状态卡片
  - 位置信息（x, y, yaw）
  - 速度信息（vx, vy, vyaw）
  - 电池电量（如果有）

**后端任务**:
- [ ] 6.1.3 实现状态推送
  - 通过 WebSocket 推送机器人状态
  - 订阅 `/rtabmap/localization_pose`、`/cmd_vel`
  - 频率：2 Hz

**验收标准**:
- ✅ 状态面板实时显示机器人信息
- ✅ 数据准确无误

**预计用时**: 4小时

#### 6.2 系统状态检查 (0.5天)

**前端任务**:
- [ ] 6.2.1 添加系统状态指示器
  - ROS 节点状态（绿色=正常，红色=异常）
  - 传感器状态（摄像头、IMU）
  - 网络延迟显示

**后端任务**:
- [ ] 6.2.2 实现健康检查 API
  - `GET /api/status` - 获取系统状态
  - 检查关键节点是否运行
  - 检查话题是否有数据

**验收标准**:
- ✅ 系统状态指示器准确显示
- ✅ 异常时显示警告

**预计用时**: 4小时

#### 6.3 实时日志查看 (0.5天)

**前端任务**:
- [ ] 6.3.1 创建日志面板
  - 底部抽屉或独立标签页
  - 实时滚动显示
  - 日志级别过滤（INFO/WARN/ERROR）
  - 搜索功能

**后端任务**:
- [ ] 6.3.2 实现日志推送
  - 通过 WebSocket 推送日志
  - 订阅 `/rosout` 话题
  - 日志级别过滤

**验收标准**:
- ✅ 实时显示 ROS 日志
- ✅ 可以按级别过滤
- ✅ 可以搜索关键词

**预计用时**: 4小时

---

### 阶段7: 国际化与UI优化 (1.5天)

**目标**: 完善中英双语支持，优化用户体验

#### 7.1 国际化完善 (1天)

**前端任务**:
- [ ] 7.1.1 完善翻译文件
  - 文件：`web_frontend/src/locales/zh-CN.json`、`en-US.json`
  - 按模块组织：common、map、task、waypoint、debug、error

- [ ] 7.1.2 添加语言切换
  - 右上角下拉菜单
  - 切换后立即生效
  - 保存用户偏好（localStorage）

- [ ] 7.1.3 翻译所有界面文本
  - 菜单、按钮、标签
  - 提示信息、错误信息
  - 占位符文本

**验收标准**:
- ✅ 所有界面文本支持中英切换
- ✅ 切换语言后无遗漏
- ✅ 用户偏好保存

**预计用时**: 8小时

#### 7.2 UI优化 (0.5天)

**前端任务**:
- [ ] 7.2.1 优化布局
  - 响应式设计（桌面端 1920x1080、1366x768）
  - 调整间距和对齐
  - 优化颜色和字体

- [ ] 7.2.2 添加交互动画
  - 按钮点击效果
  - 页面切换动画
  - 加载骨架屏

- [ ] 7.2.3 优化性能
  - React 组件优化（useMemo、useCallback）
  - 地图渲染优化
  - WebSocket 消息去重

**验收标准**:
- ✅ 界面美观、统一
- ✅ 动画流畅
- ✅ 无明显卡顿

**预计用时**: 4小时

---

### 阶段8: 测试与文档 (2天)

**目标**: 全面测试，编写用户文档

#### 8.1 功能测试 (1天)

**测试任务**:
- [ ] 8.1.1 地图可视化测试
  - 地图正确显示
  - Costmap 叠加正确
  - 机器人位置准确

- [ ] 8.1.2 交互式导航测试
  - 点击地图导航功能
  - 目标点设置准确
  - 机器人正确导航

- [ ] 8.1.3 任务管理测试
  - 创建各类任务
  - 任务状态实时更新
  - 任务控制（暂停/恢复/取消）

- [ ] 8.1.4 地图管理测试
  - 地图列表、加载、保存、删除
  - 地图元数据管理

- [ ] 8.1.5 路点管理测试
  - 路点录制、显示、编辑
  - 路点导入/导出

- [ ] 8.1.6 状态监控测试
  - 机器人状态显示
  - 系统状态检查
  - 实时日志

**验收标准**:
- ✅ 所有功能正常工作
- ✅ 无明显BUG

**预计用时**: 8小时

#### 8.2 性能测试 (0.5天)

**测试任务**:
- [ ] 8.2.1 地图渲染性能
  - 测试帧率（目标 ≥15 FPS）
  - 大地图加载时间

- [ ] 8.2.2 WebSocket 延迟
  - 测试命令响应延迟（目标 <100ms）
  - 状态更新延迟

- [ ] 8.2.3 并发连接测试
  - 测试3个客户端同时连接
  - 测试第4个连接被拒绝

**验收标准**:
- ✅ 性能指标满足要求
- ✅ 无内存泄漏

**预计用时**: 4小时

#### 8.3 文档编写 (0.5天)

**文档任务**:
- [ ] 8.3.1 用户手册
  - 文件：`docs/WEB_USER_GUIDE.md`
  - 安装、启动、使用说明
  - 功能介绍、截图

- [ ] 8.3.2 API 文档
  - 文件：`docs/WEB_API.md`
  - REST API 端点说明
  - WebSocket 消息格式

- [ ] 8.3.3 开发者文档
  - 文件：`docs/WEB_DEVELOPER_GUIDE.md`
  - 项目结构、代码规范
  - 如何添加新功能

**验收标准**:
- ✅ 文档完整、清晰
- ✅ 包含足够示例

**预计用时**: 4小时

---

## 关键里程碑

| 阶段 | 完成日期 | 里程碑 | 验收标准 |
|------|---------|--------|---------|
| 阶段0 | Day 0.5 | 环境验证 | 后端正常启动，前端可访问 |
| ✅ 阶段1 | 2026-01-12 | 地图可视化 | 地图正确显示，机器人位置准确 |
| ✅ 阶段2 | 2026-01-12 | 交互导航+Costmap | 点击地图导航，Costmap 叠加 |
| ✅ 阶段3 | 2026-01-14 | 任务管理 | 任务状态实时监控，探索/巡逻面板 |
| 阶段4 | Day 8 | 地图管理 | 地图列表、加载、保存 |
| 阶段5 | Day 10 | 路点管理 | 路点录制、编辑 |
| 阶段6 | Day 11.5 | 状态监控 | 机器人状态、日志查看 |
| 阶段7 | Day 13 | UI优化 | 国际化、交互优化 |
| 阶段8 | Day 14 | 测试发布 | 全功能测试通过 |

---

## 风险评估

### 高风险项

1. **ros2djs 集成复杂度**
   - 风险：ros2djs 文档不完善，可能需要大量调试
   - 缓解：阶段1优先级最高，及时发现问题

2. **Costmap 手动渲染**
   - 风险：性能可能不达标，渲染逻辑复杂
   - 缓解：使用 canvas 离屏渲染，优化算法

3. **WebSocket 并发管理**
   - 风险：多客户端连接可能导致消息混乱
   - 缓解：使用连接ID区分客户端，消息路由机制

### 中风险项

1. **坐标转换准确性**
   - 风险：像素坐标转地图坐标可能有误差
   - 缓解：使用 ros2djs 提供的 API，充分测试

2. **任务状态同步**
   - 风险：前端显示的任务状态可能与后端不一致
   - 缓解：定期轮询，WebSocket 实时推送

---

## 资源需求

### 人力
- 全栈开发者 1人（React + Python + ROS2）
- 每天工作 8小时

### 硬件
- 开发机器人（Gazebo 仿真）
- 测试地图（exploration_test）

### 软件
- ROS2 Humble
- Node.js 18+
- Python 3.10+
- 现代浏览器（Chrome/Firefox）

---

## 下一步行动

### 立即开始（阶段0）

1. **验证当前环境**
   ```bash
   # 检查依赖
   cd ~/workDisk/lododo_bot/src/bot_teleop/web_frontend
   npm list roslib eventemitter2
   
   # 检查 Python 依赖
   source ~/workDisk/lododo_bot/venv_ros2/bin/activate
   pip list | grep -E "fastapi|uvicorn|websockets"
   ```

2. **测试完整启动流程**
   ```bash
   # 终端1: ROS环境
   ros2 launch bot_bringup simulation_web_full.launch.py slam:=false map_name:=exploration_test
   
   # 终端2: Web后端
   bash src/bot_teleop/scripts/start_web_server.sh
   
   # 浏览器: http://localhost:8000
   ```

3. **确认阶段0验收标准**
   - [ ] 后端启动无错误
   - [ ] 前端页面可访问
   - [ ] rosbridge 运行在 9090 端口
   - [ ] WebTerminalNode 初始化成功

### 准备阶段1

1. **安装 ros2djs 相关依赖**
   ```bash
   cd web_frontend
   npm install roslib@2.0.1 eventemitter2@6.4.9
   ```

2. **创建 rosConnection.ts 服务**

3. **备份现有 MapView 组件**

---

## 附录

### A. 文件清单

**需要创建的文件（20+）**:
- `web_frontend/src/services/rosConnection.ts`
- `web_frontend/src/components/MapView/MapToolbar.tsx`
- `web_frontend/src/components/MapView/CostmapLayer.tsx`
- `web_frontend/src/components/MapView/WaypointMarkers.tsx`
- `web_frontend/src/components/TaskControl/TaskCreationForm.tsx`
- `web_frontend/src/components/TaskControl/CurrentTaskStatus.tsx`
- `web_frontend/src/components/TaskControl/TaskHistory.tsx`
- `web_frontend/src/components/MapManager/MapManager.tsx`
- `web_frontend/src/components/WaypointManager/WaypointManager.tsx`
- `web_frontend/src/components/StatusMonitor/StatusMonitor.tsx`
- `web/backend/api/maps.py` （完善）
- `web/backend/api/waypoints.py` （完善）
- `web/backend/api/tasks.py` （完善）
- `config/web_config.yaml` （完善）
- `docs/WEB_USER_GUIDE.md`
- `docs/WEB_API.md`
- `docs/WEB_DEVELOPER_GUIDE.md`

**需要修改的文件（10+）**:
- `web_frontend/src/components/MapView/MapView.tsx` （重构）
- `web_frontend/src/components/TaskControl/TaskControl.tsx` （重构）
- `web_frontend/src/components/Layout/MainLayout.tsx` （添加菜单）
- `web_frontend/src/services/websocket.ts` （完善）
- `web/backend/web_server.py` （添加静态文件服务）
- `web/backend/websocket_handler.py` （完善）
- `web/backend/nodes/web_terminal_node.py` （添加更多方法）
- `web_frontend/src/locales/zh-CN.json` （完善翻译）
- `web_frontend/src/locales/en-US.json` （完善翻译）

### B. 技术参考

- ros2djs: http://robotwebtools.org/jsdoc/ros2d/
- roslibjs: http://robotwebtools.org/jsdoc/roslibjs/
- Ant Design: https://ant.design/
- FastAPI: https://fastapi.tiangolo.com/
- bot_cmd_interface SDK: `~/workDisk/lododo_bot/src/bot_cmd_interface/bot_cmd_interface/sdk/`

---

**实施计划完成 - 准备开始阶段0！** 🚀
