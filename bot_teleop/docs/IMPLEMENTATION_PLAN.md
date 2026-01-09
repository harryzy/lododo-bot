# Robot Web控制界面实施计划

**版本**: v1.0.0  
**创建日期**: 2026-01-09  
**项目状态**: 📋 计划阶段  
**预计工期**: 14天 (2周完成MVP)

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

### 阶段1: 地图可视化核心 (2天)

**目标**: 使用原生Canvas实现地图显示，让用户能看到真实的ROS地图和机器人位置

**技术方案**: 
- ✅ **不使用ros2djs**（已过时，6年未更新，有兼容性问题）
- ✅ 使用**原生HTML5 Canvas + ROSLIB.js**手动渲染
- ✅ 优势：轻量级、无依赖、完全控制、性能好

#### 1.1 安装和配置ROSLIB (0.5天)

**前端任务**:
- [ ] 1.1.1 安装依赖
  ```bash
  cd web_frontend
  npm install roslib@2.0.1 eventemitter2@6.4.9
  ```

- [ ] 1.1.2 创建 ROS 连接服务
  - 文件：`web_frontend/src/services/rosConnection.ts`
  - 功能：封装 rosbridge WebSocket 连接管理
  - 内容：连接、断线重连、错误处理

- [ ] 1.1.3 更新配置文件
  - 在 `config/web_config.yaml` 中添加 rosbridge URL
  - 添加各话题更新频率配置

**验收标准**:
- ✅ 前端能成功连接 ws://localhost:9090
- ✅ 控制台无连接错误
- ✅ rosConnection.ts 提供统一接口

**预计用时**: 4小时

#### 1.2 实现Canvas地图渲染 (1天)

**前端任务**:
- [ ] 1.2.1 创建 MapRenderer 工具类
  - 文件：`web_frontend/src/utils/MapRenderer.ts`
  - 功能：
    - OccupancyGrid → Canvas 图像数据转换
    - 坐标系转换（ROS坐标 ↔ 屏幕坐标）
    - 缩放/平移变换矩阵管理

- [ ] 1.2.2 重构 MapView 组件
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

- [ ] 1.2.3 订阅 /map 话题并渲染
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

- [ ] 1.2.4 添加地图控制功能
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

**预计用时**: 8小时

#### 1.3 实现机器人位置显示 (0.5天)

**前端任务**:
- [ ] 1.3.1 添加机器人位姿监听
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

- [ ] 1.3.2 在Canvas上绘制机器人
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

- [ ] 1.3.3 添加机器人追踪功能
  - "跟随机器人"按钮
  - 自动将视图中心保持在机器人位置

**验收标准**:
- ✅ 机器人位置实时显示（10 Hz）
- ✅ 机器人朝向准确显示（蓝色箭头）
- ✅ "跟随机器人"功能正常工作

**预计用时**: 4小时

---

### 阶段2: 交互式导航与Costmap (2天)

**目标**: 实现RViz风格的点击地图导航，添加Costmap可视化

#### 2.1 Nav Goal Pose交互 (1天)

**前端任务**:
- [ ] 2.1.1 添加工具栏
  - 文件：`web_frontend/src/components/MapView/MapToolbar.tsx`
  - 按钮：设置导航目标（🎯图标）、清除目标、重置视图

- [ ] 2.1.2 实现点击+拖拽交互
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

- [ ] 2.1.3 实现坐标转换
  - 像素坐标 → 地图坐标（考虑地图原点、分辨率、缩放）
  - 使用 ros2djs 的 Viewer API

- [ ] 2.1.4 发送导航请求
  - 调用 FastAPI `/api/tasks/navigate` 端点
  - 显示确认对话框（可选）
  - 在地图上显示目标标记

**后端任务**:
- [ ] 2.1.5 验证坐标有效性
  - 检查目标点是否在地图范围内
  - 检查目标点是否在障碍物上
  - 返回错误信息（如果无效）

**验收标准**:
- ✅ 点击工具栏按钮后鼠标变为十字准星
- ✅ 可以点击+拖拽绘制箭头
- ✅ 释放鼠标后发送导航请求
- ✅ 右键取消操作
- ✅ 机器人开始导航到目标点

**预计用时**: 8小时

#### 2.2 Costmap可视化 (1天)

**前端任务**:
- [ ] 2.2.1 创建 CostmapLayer 组件
  - 文件：`web_frontend/src/components/MapView/CostmapLayer.tsx`
  - 订阅 `/local_costmap/costmap` 和 `/global_costmap/costmap`

- [ ] 2.2.2 实现手动渲染
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

- [ ] 2.2.3 叠加到地图上
  - 创建独立的 canvas 层
  - 使用绝对定位叠加在地图 canvas 上
  - 同步缩放和平移

- [ ] 2.2.4 添加图层控制
  - 切换按钮：显示/隐藏 Local Costmap
  - 切换按钮：显示/隐藏 Global Costmap
  - 透明度滑块（0.3-0.8）

**验收标准**:
- ✅ Costmap 半透明叠加在地图上
- ✅ 膨胀区显示为橙色，致命障碍物显示为红色
- ✅ 可以切换显示/隐藏图层
- ✅ Costmap 跟随地图缩放和平移
- ✅ 更新频率 5 Hz

**预计用时**: 8小时

---

### 阶段3: 任务管理完整功能 (2天)

**目标**: 完善任务控制面板，实现任务状态实时监控

#### 3.1 任务状态实时显示 (1天)

**前端任务**:
- [ ] 3.1.1 重构 TaskControl 组件
  - 文件：`web_frontend/src/components/TaskControl/TaskControl.tsx`
  - 分离为子组件：TaskCreationForm、CurrentTaskStatus、TaskHistory

- [ ] 3.1.2 实现 WebSocket 订阅
  - 连接 FastAPI WebSocket (ws://localhost:8000/ws)
  - 监听任务响应消息
  - 监听任务状态更新

- [ ] 3.1.3 创建 CurrentTaskStatus 组件
  - 显示当前任务信息（ID、类型、状态）
  - 显示进度条（从 task_status 数据）
  - 显示预计剩余时间
  - 控制按钮：暂停、恢复、取消

**后端任务**:
- [ ] 3.1.4 实现任务状态推送
  - 修改 `web/backend/websocket_handler.py`
  - 定期查询 MissionPlanner 任务状态（通过 bot_cmd_interface）
  - 广播任务进度更新到所有连接的客户端

- [ ] 3.1.5 实现任务控制端点
  - `POST /api/tasks/{task_id}/pause` - 暂停任务
  - `POST /api/tasks/{task_id}/resume` - 恢复任务
  - `POST /api/tasks/{task_id}/cancel` - 取消任务
  - 通过 WebTerminalNode 调用 bot_cmd_interface SDK

**验收标准**:
- ✅ 创建任务后立即显示任务信息
- ✅ 任务状态实时更新（2秒刷新一次）
- ✅ 进度条准确显示任务进度
- ✅ 可以暂停、恢复、取消任务
- ✅ 任务完成后显示成功/失败消息

**预计用时**: 8小时

#### 3.2 任务历史与探索/巡逻面板 (1天)

**前端任务**:
- [ ] 3.2.1 实现 TaskHistory 组件
  - 显示最近10个任务
  - 列表项：任务ID、类型、状态、开始时间、耗时
  - 点击查看详情（Modal对话框）

- [ ] 3.2.2 完善探索任务面板
  - 输入地图名称
  - 选择是否保存地图
  - 显示探索进度（百分比）

- [ ] 3.2.3 完善巡逻任务面板
  - 选择路点文件（从后端获取列表）
  - 选择模式：单次/循环
  - 显示当前路点进度（1/5）

**后端任务**:
- [ ] 3.2.4 实现任务历史 API
  - `GET /api/tasks` - 获取任务历史
  - 从 MissionPlanner 查询任务记录
  - 返回 JSON 格式

- [ ] 3.2.5 实现探索/巡逻 API
  - `POST /api/tasks/exploration` - 创建探索任务
  - `POST /api/tasks/patrol` - 创建巡逻任务
  - 通过 WebTerminalNode 调用 bot_cmd_interface SDK

**验收标准**:
- ✅ 任务历史列表正确显示
- ✅ 可以创建探索任务并监控进度
- ✅ 可以创建巡逻任务并监控进度
- ✅ 所有任务记录保存到历史

**预计用时**: 8小时

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
| 阶段1 | Day 2.5 | 地图可视化 | 地图正确显示，机器人位置准确 |
| 阶段2 | Day 4.5 | 交互导航 | 点击地图导航，Costmap 叠加 |
| 阶段3 | Day 6.5 | 任务管理 | 任务状态实时监控，任务控制 |
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
