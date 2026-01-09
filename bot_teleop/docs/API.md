# Web控制界面 API协议文档

**版本**: v1.0.0  
**协议类型**: REST API + WebSocket  
**基础URL**: `http://localhost:8000`  
**WebSocket URL**: `ws://localhost:8000/ws`  
**rosbridge URL**: `ws://localhost:9090`

---

## 1. 认证与授权

**当前版本**: 无需认证（单用户模式）  
**未来版本**: JWT Token认证（可选）

---

## 2. REST API端点

### 2.1 系统状态

#### GET `/api/status`
获取Web服务器状态

**请求**:
```http
GET /api/status HTTP/1.1
Host: localhost:8000
```

**响应**:
```json
{
  "status": "ok",
  "timestamp": 1704700800.123,
  "version": "1.0.0",
  "ros_connected": true,
  "active_tasks": 1
}
```

---

### 2.2 任务管理

#### POST `/api/tasks/navigate`
创建导航任务

**请求**:
```json
{
  "x": 2.3,
  "y": 1.5,
  "yaw": 0.0,
  "timeout": 300.0
}
```

**响应**:
```json
{
  "task_id": 123,
  "status": "queued",
  "message": "Navigation task created",
  "request_id": "nav-001"
}
```

---

#### POST `/api/tasks/explore`
创建探索建图任务

**请求**:
```json
{
  "map_name": "office_floor1",
  "save_on_completion": true,
  "timeout": 1800.0
}
```

**响应**:
```json
{
  "task_id": 124,
  "status": "queued",
  "message": "Exploration task created"
}
```

---

#### POST `/api/tasks/patrol`
创建巡逻任务

**请求**:
```json
{
  "waypoint_file": "patrol_route1.yaml",
  "mode": "loop",
  "loops": 3
}
```

**响应**:
```json
{
  "task_id": 125,
  "status": "queued",
  "message": "Patrol task created"
}
```

---

#### GET `/api/tasks`
获取任务历史

**请求**:
```http
GET /api/tasks?limit=10&offset=0 HTTP/1.1
```

**响应**:
```json
{
  "tasks": [
    {
      "task_id": 123,
      "type": "navigation",
      "status": "completed",
      "created_at": "2026-01-08T10:23:45Z",
      "completed_at": "2026-01-08T10:25:30Z",
      "result": "success"
    }
  ],
  "total": 50,
  "limit": 10,
  "offset": 0
}
```

---

#### GET `/api/tasks/{task_id}`
获取任务详情

**响应**:
```json
{
  "task_id": 123,
  "type": "navigation",
  "status": "running",
  "progress": 0.75,
  "created_at": "2026-01-08T10:23:45Z",
  "params": {
    "x": 2.3,
    "y": 1.5,
    "yaw": 0.0
  },
  "estimated_remaining": 30.0
}
```

---

#### POST `/api/tasks/{task_id}/pause`
暂停任务

**响应**:
```json
{
  "status": "paused",
  "message": "Task paused successfully"
}
```

---

#### POST `/api/tasks/{task_id}/resume`
恢复任务

**响应**:
```json
{
  "status": "running",
  "message": "Task resumed successfully"
}
```

---

#### POST `/api/tasks/{task_id}/cancel`
取消任务

**响应**:
```json
{
  "status": "cancelled",
  "message": "Task cancelled successfully"
}
```

---

### 2.3 地图管理

#### GET `/api/maps`
获取地图列表

**响应**:
```json
{
  "maps": [
    {
      "name": "office_floor1",
      "size": 2621440,
      "created_at": "2026-01-08T10:23:45Z",
      "width": 50.0,
      "height": 40.0,
      "resolution": 0.05,
      "metadata": {
        "description": "办公室一楼地图",
        "tags": ["office", "floor1"]
      }
    }
  ]
}
```

---

#### POST `/api/maps/{map_name}/load`
加载地图

**响应**:
```json
{
  "status": "loading",
  "message": "Map loading initiated"
}
```

---

#### POST `/api/maps/{map_name}/save`
保存地图

**请求**:
```json
{
  "description": "办公室一楼地图",
  "tags": ["office", "floor1", "verified"]
}
```

**响应**:
```json
{
  "status": "saved",
  "filepath": "/home/user/lododo_bot/maps/office_floor1/"
}
```

---

#### DELETE `/api/maps/{map_name}`
删除地图

**响应**:
```json
{
  "status": "deleted",
  "message": "Map deleted successfully"
}
```

---

### 2.4 路点管理

#### GET `/api/waypoints`
获取路点文件列表

**响应**:
```json
{
  "waypoint_files": [
    {
      "filename": "patrol_route1.yaml",
      "waypoint_count": 5,
      "created_at": "2026-01-08T10:23:45Z",
      "updated_at": "2026-01-08T12:30:00Z"
    }
  ]
}
```

---

#### GET `/api/waypoints/{filename}`
获取路点详情

**响应**:
```json
{
  "filename": "patrol_route1.yaml",
  "waypoints": [
    {
      "name": "point_1",
      "x": 2.3,
      "y": 1.5,
      "yaw": 0.0,
      "dwell_time": 2.0
    }
  ]
}
```

---

#### POST `/api/waypoints`
创建路点文件

**请求**:
```json
{
  "filename": "new_route.yaml",
  "waypoints": [
    {
      "name": "point_1",
      "x": 2.3,
      "y": 1.5,
      "yaw": 0.0,
      "dwell_time": 2.0
    }
  ]
}
```

**响应**:
```json
{
  "status": "created",
  "filepath": "/home/user/lododo_bot/waypoints/new_route.yaml"
}
```

---

#### PUT `/api/waypoints/{filename}`
更新路点文件

**请求**: 同POST

**响应**:
```json
{
  "status": "updated",
  "message": "Waypoint file updated successfully"
}
```

---

#### DELETE `/api/waypoints/{filename}`
删除路点文件

**响应**:
```json
{
  "status": "deleted",
  "message": "Waypoint file deleted successfully"
}
```

---

### 2.5 设置管理

#### GET `/api/settings`
获取用户设置

**响应**:
```json
{
  "language": "zh-CN",
  "brand_name": "Robot",
  "display": {
    "show_robot": true,
    "show_path": true,
    "show_costmap": true,
    "costmap_opacity": 0.5
  },
  "performance": {
    "map_update_rate": 1.0,
    "pose_update_rate": 10.0,
    "costmap_update_rate": 5.0,
    "path_update_rate": 2.0
  },
  "network": {
    "fastapi_url": "http://localhost:8000",
    "rosbridge_url": "ws://localhost:9090",
    "reconnect_attempts": 5,
    "reconnect_interval": 5
  }
}
```

---

#### POST `/api/settings`
保存用户设置

**请求**: 同GET响应

**响应**:
```json
{
  "status": "saved",
  "message": "Settings saved successfully"
}
```

---

### 2.6 紧急操作

#### POST `/api/emergency_stop`
紧急停止

**响应**:
```json
{
  "status": "stopped",
  "message": "Emergency stop activated",
  "timestamp": 1704700800.123
}
```

---

## 3. WebSocket协议（FastAPI）

### 3.1 连接
```
ws://localhost:8000/ws
```

### 3.2 消息格式

#### 客户端 → 服务器（命令请求）
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

#### 服务器 → 客户端（命令响应）
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

#### 服务器 → 客户端（状态推送）
```json
{
  "type": "status_update",
  "timestamp": "2026-01-08T12:00:05Z",
  "data": {
    "robot_pose": {
      "x": 1.5,
      "y": 2.3,
      "yaw": 0.5
    },
    "task_status": {
      "task_id": 123,
      "state": "RUNNING",
      "progress": 0.45
    }
  }
}
```

#### 心跳
```json
{
  "type": "ping",
  "timestamp": "2026-01-08T12:00:10Z"
}
```

---

## 4. WebSocket协议（rosbridge）

### 4.1 连接
```
ws://localhost:9090
```

### 4.2 订阅话题

#### 订阅地图
```json
{
  "op": "subscribe",
  "topic": "/map",
  "type": "nav_msgs/OccupancyGrid",
  "throttle_rate": 1000
}
```

#### 订阅机器人位姿
```json
{
  "op": "subscribe",
  "topic": "/rtabmap/localization_pose",
  "type": "geometry_msgs/PoseStamped",
  "throttle_rate": 100
}
```

#### 订阅Costmap
```json
{
  "op": "subscribe",
  "topic": "/local_costmap/costmap",
  "type": "nav_msgs/OccupancyGrid",
  "throttle_rate": 200
}
```

#### 订阅路径
```json
{
  "op": "subscribe",
  "topic": "/plan",
  "type": "nav_msgs/Path",
  "throttle_rate": 500
}
```

### 4.3 取消订阅
```json
{
  "op": "unsubscribe",
  "topic": "/map"
}
```

---

## 5. 错误码定义

| 错误码 | 名称 | 描述 |
|--------|------|------|
| 0 | SUCCESS | 操作成功 |
| 1000 | INVALID_REQUEST | 请求格式错误 |
| 1001 | MISSING_PARAMETER | 缺少必需参数 |
| 1002 | INVALID_PARAMETER | 参数值无效 |
| 2000 | ROS_NOT_CONNECTED | ROS未连接 |
| 2001 | TOPIC_NOT_AVAILABLE | ROS话题不可用 |
| 2002 | SERVICE_CALL_FAILED | ROS服务调用失败 |
| 3000 | TASK_NOT_FOUND | 任务不存在 |
| 3001 | TASK_ALREADY_RUNNING | 任务已在运行 |
| 3002 | TASK_CREATION_FAILED | 任务创建失败 |
| 4000 | MAP_NOT_FOUND | 地图不存在 |
| 4001 | MAP_LOAD_FAILED | 地图加载失败 |
| 5000 | FILE_NOT_FOUND | 文件不存在 |
| 5001 | FILE_READ_ERROR | 文件读取错误 |
| 5002 | FILE_WRITE_ERROR | 文件写入错误 |
| 9000 | INTERNAL_SERVER_ERROR | 服务器内部错误 |

---

## 6. 数据模型

### 6.1 Waypoint（路点）
```typescript
interface Waypoint {
  name: string;         // 路点名称
  x: number;            // X坐标（米）
  y: number;            // Y坐标（米）
  yaw: number;          // 朝向（弧度）
  dwell_time: number;   // 停留时间（秒）
}
```

### 6.2 Task（任务）
```typescript
interface Task {
  task_id: number;                // 任务ID
  type: 'navigation' | 'exploration' | 'patrol';  // 任务类型
  status: 'queued' | 'running' | 'paused' | 'completed' | 'failed' | 'cancelled';
  progress: number;               // 进度（0.0-1.0）
  created_at: string;             // 创建时间（ISO 8601）
  started_at?: string;            // 开始时间
  completed_at?: string;          // 完成时间
  params: Record<string, any>;    // 任务参数
  result?: string;                // 结果
  error_message?: string;         // 错误消息
}
```

### 6.3 MapInfo（地图信息）
```typescript
interface MapInfo {
  name: string;                   // 地图名称
  size: number;                   // 文件大小（字节）
  created_at: string;             // 创建时间
  width: number;                  // 地图宽度（米）
  height: number;                 // 地图高度（米）
  resolution: number;             // 分辨率（米/像素）
  metadata: {
    description?: string;         // 描述
    tags?: string[];              // 标签
  };
}
```

### 6.4 RobotPose（机器人位姿）
```typescript
interface RobotPose {
  x: number;                      // X坐标（米）
  y: number;                      // Y坐标（米）
  yaw: number;                    // 朝向（弧度）
  timestamp: number;              // 时间戳
}
```

---

## 7. 使用示例

### 7.1 Python（后端）
```python
# 使用bot_cmd_interface SDK
from bot_cmd_interface.sdk import create_navigate_request, CommandRequest

# 创建导航请求
request = create_navigate_request(
    x=2.0,
    y=3.0,
    yaw=0.0,
    request_id="nav-001"
)

# 发布到 /cmd/request
cmd_publisher.publish(request)

# 订阅 /cmd/response
def response_callback(msg):
    if msg.header.request_id == "nav-001":
        print(f"Response: {msg.body.data}")
```

### 7.2 TypeScript（前端）
```typescript
// WebSocket客户端
const ws = new WebSocket('ws://localhost:8000/ws');

ws.onopen = () => {
  // 发送导航命令
  ws.send(JSON.stringify({
    type: 'command',
    request_id: 'web-001',
    timestamp: new Date().toISOString(),
    command: {
      action: 'navigate_to_pose',
      params: { x: 2.0, y: 3.0, yaw: 0.0 }
    }
  }));
};

ws.onmessage = (event) => {
  const message = JSON.parse(event.data);
  if (message.type === 'response') {
    console.log('Task ID:', message.data.task_id);
  }
};

// rosbridge客户端
import ROSLIB from 'roslibjs';

const ros = new ROSLIB.Ros({
  url: 'ws://localhost:9090'
});

const mapTopic = new ROSLIB.Topic({
  ros: ros,
  name: '/map',
  messageType: 'nav_msgs/OccupancyGrid'
});

mapTopic.subscribe((message) => {
  console.log('Map received:', message);
});
```

---

**API文档结束**

本文档定义了Web控制界面的完整API协议，包括REST API、WebSocket通信和rosbridge集成。开发时请严格遵循此规范。
