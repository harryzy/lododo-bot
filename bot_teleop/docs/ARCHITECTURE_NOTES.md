# Web控制界面 - 架构核心要点

**⚠️ 此文档是 WEB_DESIGN.md 的关键补充，必读！**

---

## 🚨 架构铁律：bot_cmd_interface 统一接口

### 禁止事项

**❌ 绝对禁止直接调用后端服务**：
```python
# ❌ 错误：在 tasks.py 中直接调用 ROS 服务
from bot_navigation_msgs.srv import StartExploration
client = node.create_client(StartExploration, '/mission/start_exploration')
```

**❌ 绝对禁止在 API 层导入服务定义**：
```python
# ❌ 错误
from bot_navigation_msgs.srv import StartExploration, StartPatrol
from bot_navigation_msgs.srv import NavigateToPose
```

---

### 正确实现

**✅ 必须通过 WebTerminalNode + SDK**：
```python
# ✅ 正确：web/backend/api/tasks.py
from ..web_server import get_web_terminal_node

@router.post("/navigate")
async def create_navigation_task(req: NavigateRequest, node = Depends(get_node)):
    # ✅ 通过节点方法发送
    request_id = node.navigate_to_pose(x=req.x, y=req.y, yaw=req.yaw)
    
    # 返回 request_id，实际任务状态通过 WebSocket 推送
    return {"success": True, "data": {"request_id": request_id}}
```

**✅ WebTerminalNode 使用 SDK**：
```python
# ✅ 正确：web/backend/nodes/web_terminal_node.py
from bot_cmd_interface.sdk import create_navigate_request

class WebTerminalNode(Node):
    def navigate_to_pose(self, x, y, yaw):
        # ✅ 使用 SDK 创建请求
        request = create_navigate_request(x=x, y=y, yaw=yaw, request_id=request_id)
        
        # ✅ 发布到 /cmd/request 话题
        self.cmd_publisher.publish(String(data=request.to_json()))
```

---

## 命令流程图

```
用户点击地图
    ↓ HTTP POST /api/tasks/navigate
FastAPI (tasks.py)
    ↓ node.navigate_to_pose(x, y, yaw)
WebTerminalNode
    ↓ create_navigate_request() → 发布到 /cmd/request
CommandAdapter (bot_cmd_interface)
    ↓ 解析请求 → 调用 /mission/navigate_to_pose 服务
MissionPlanner
    ↓ 创建任务 → 返回 task_id
CommandAdapter
    ↓ 发布到 /cmd/response
WebTerminalNode
    ↓ response_callback() → WebSocket broadcast
Web前端
    ↓ 显示任务状态
```

---

## SDK 使用要点

### 1. 导入SDK

```python
from bot_cmd_interface.sdk import (
    create_navigate_request,
    create_exploration_request,
    create_patrol_request,
    create_get_status_request,
    CommandResponse
)
```

### 2. 创建请求

```python
# 导航请求（SDK 自动生成 UUID）
request = create_navigate_request(x=2.0, y=3.0, yaw=0.0)

# 探索请求
request = create_exploration_request(
    map_name="office",
    save_on_completion=True
)
```

### 3. 发布请求

```python
msg = String()
msg.data = request.to_json()  # SDK 自动序列化
self.cmd_publisher.publish(msg)
```

### 4. 解析响应

```python
def _response_callback(self, msg: String):
    response = CommandResponse.from_json(msg.data)  # SDK 自动反序列化
    
    if response.is_success():
        task_id = response.result.get('task_id') if response.result else None
        print(f"Task created: {task_id}")
```

---

## 例外情况（允许直接操作）

以下操作**不需要**通过 bot_cmd_interface：

1. **文件读取**（只读操作）：
   - 地图文件列表：`GET /api/maps/list`
   - 路点文件加载：`GET /api/waypoints/load`

2. **话题订阅**（由 ros_bridge.py 处理，只读）：
   - `/map` - 地图数据
   - `/rtabmap/localization_pose` - 机器人位姿
   - `/global_costmap/costmap` - 代价地图

3. **本地文件保存**：
   - 路点文件写入：`~/lododo_bot/waypoints/`

**原则**：只要是**控制类命令**（导航/探索/巡逻/录制路点），必须通过 `/cmd/request`。

---

## 参考文档

- **SDK 详细说明**：[WEB_DESIGN.md § 4.3](WEB_DESIGN.md#43-bot_cmd_interface-sdk-使用指南-)
- **实现示例**：[web/backend/nodes/web_terminal_node.py](../web/backend/nodes/web_terminal_node.py)
- **bot_cmd_interface**：[../../bot_cmd_interface/README.md](../../bot_cmd_interface/README.md)

---

**最后更新**：2026-01-08  
**重要性**：⚠️ **强制遵守，违反将导致架构错误**
