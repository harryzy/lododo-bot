# Web控制界面 - 架构设计修正报告

**修正日期**: 2026-01-08  
**状态**: ✅ 架构设计已完全修正，符合 bot_cmd_interface SDK v1.0.0 实际接口

---

## 📋 修正总结

### 1. 已删除的生成代码

- ✅ 删除 `web/` 目录（后端代码）
- ✅ 删除 `web_frontend/` 目录（前端代码）
- ✅ 删除 `requirements_web.txt`
- ✅ 删除 `config/web_config.yaml`
- ✅ 删除 `scripts/` 中的生成脚本
- ✅ 删除 `launch/web_server.launch.py`

### 2. SDK 接口审核完成

**创建文档**: [SDK_INTERFACE_AUDIT.md](SDK_INTERFACE_AUDIT.md)

**审核内容**:
- ✅ 导出的工厂函数清单
- ✅ 每个函数的完整签名
- ✅ 参数类型和默认值
- ✅ CommandRequest/CommandResponse 类结构
- ✅ ResponseStatus 和 ErrorCode 常量
- ✅ ActionType 动作类型定义

### 3. 发现的设计错误

#### 错误 1: 函数名错误 ❌
```python
# 设计文档中的错误
from bot_cmd_interface.sdk import create_task_query_request

# SDK 实际接口
from bot_cmd_interface.sdk import create_get_status_request
```

#### 错误 2: 参数类型错误 ❌
```python
# 设计文档中的错误：task_id 用 int
request = create_get_status_request(task_id=1001)

# SDK 实际接口：task_id 是 Optional[str]
request = create_get_status_request(task_id="task-001")
```

#### 错误 3: 响应字段名错误 ❌
```python
# 设计文档中的错误：使用 body.data
task_id = response.body.data.get('task_id')

# SDK 实际接口：使用 result
task_id = response.result.get('task_id') if response.result else None
```

#### 错误 4: 状态检查方法错误 ❌
```python
# 设计文档中的错误
if response.is_completed():
    ...

# SDK 实际接口
if response.is_success():
    ...
```

#### 错误 5: request_id 参数错误 ❌
```python
# 设计文档中的错误：工厂函数接受 request_id
request = create_navigate_request(x=2.0, y=3.0, request_id="web-nav-1")

# SDK 实际接口：工厂函数不接受 request_id，SDK 自动生成 UUID
request = create_navigate_request(x=2.0, y=3.0)
# request._header["request_id"] 会被自动赋值为 UUID
```

---

## ✅ 已修正的文档

### 修正 1: WEB_DESIGN.md

**修正章节**:
- § 2.2.1 "SDK 使用规范" - 所有代码示例
- 命令流程图中的响应格式

**修正内容** (7处替换):
1. ✅ 导入语句：`create_task_query_request` → `create_get_status_request`
2. ✅ 查询请求：`task_id=1001` → `task_id="task-001"`
3. ✅ 导航请求：移除 `request_id` 参数
4. ✅ 探索请求：移除 `request_id` 参数
5. ✅ 巡逻请求：移除 `request_id` 参数
6. ✅ 响应处理：`is_completed()` → `is_success()`，`body.data` → `result`
7. ✅ 流程图：`"data": {"task_id": 1001}` → `"result": {"task_id": "task-001"}`

### 修正 2: ARCHITECTURE_NOTES.md

**修正内容** (4处替换):
1. ✅ 导入语句：添加 `create_get_status_request`
2. ✅ 导航请求：移除 `request_id` 参数
3. ✅ 探索请求：移除 `request_id` 参数
4. ✅ 响应处理：修正字段和方法

---

## 🎯 正确的架构设计

### 核心原则

1. **所有控制命令通过 `/cmd/request` 话题**
   ```
   Web → WebTerminalNode → /cmd/request → CommandAdapter → MissionPlanner
   ```

2. **使用 SDK 工厂函数创建请求**
   ```python
   from bot_cmd_interface.sdk import create_navigate_request
   request = create_navigate_request(x=2.0, y=3.0, yaw=0.0)
   # SDK 自动生成 request_id（UUID）
   ```

3. **SDK 自动处理序列化**
   ```python
   msg = String()
   msg.data = request.to_json()  # SDK 序列化
   self.cmd_publisher.publish(msg)
   ```

4. **响应解析使用 SDK**
   ```python
   response = CommandResponse.from_json(msg.data)  # SDK 反序列化
   
   if response.is_success():  # 使用 SDK 方法
       task_id = response.result.get('task_id')  # 使用 result 字段
   ```

### WebTerminalNode 标准实现模式

```python
from rclpy.node import Node
from std_msgs.msg import String
from bot_cmd_interface.sdk import (
    CommandResponse,
    ResponseStatus,
    create_navigate_request,
    create_exploration_request,
    create_patrol_request,
    create_get_status_request,
)

class WebTerminalNode(Node):
    def __init__(self, response_callback=None):
        super().__init__('web_terminal_node')
        self.response_callback = response_callback
        
        # ✅ 发布到 /cmd/request
        self.cmd_publisher = self.create_publisher(String, '/cmd/request', 10)
        
        # ✅ 订阅 /cmd/response
        self.cmd_subscriber = self.create_subscription(
            String, '/cmd/response', self._response_callback, 10
        )
    
    def _response_callback(self, msg: String):
        """处理响应"""
        try:
            # ✅ 使用 SDK 反序列化
            response = CommandResponse.from_json(msg.data)
            
            self.get_logger().info(
                f"Response: request_id={response.request_id}, "
                f"status={response.status}, code={response.code}"
            )
            
            # 异步推送到 WebSocket
            if self.response_callback:
                asyncio.create_task(self.response_callback(response))
                
        except Exception as e:
            self.get_logger().error(f"Failed to process response: {e}")
    
    def navigate_to_pose(self, x: float, y: float, yaw: float = 0.0) -> str:
        """发送导航命令"""
        # ✅ 使用 SDK 工厂函数（自动生成 request_id）
        request = create_navigate_request(x=x, y=y, yaw=yaw)
        
        # ✅ SDK 序列化并发布
        msg = String()
        msg.data = request.to_json()
        self.cmd_publisher.publish(msg)
        
        # ✅ 返回 SDK 生成的 request_id
        return request._header["request_id"]
    
    def start_exploration(self, map_name: str = None, save_on_completion: bool = True) -> str:
        """发送探索命令"""
        request = create_exploration_request(
            map_name=map_name,
            save_on_completion=save_on_completion
        )
        
        msg = String()
        msg.data = request.to_json()
        self.cmd_publisher.publish(msg)
        
        return request._header["request_id"]
    
    def start_patrol(self, waypoint_file: str, mode: str = "loop") -> str:
        """发送巡逻命令"""
        request = create_patrol_request(waypoint_file=waypoint_file, mode=mode)
        
        msg = String()
        msg.data = request.to_json()
        self.cmd_publisher.publish(msg)
        
        return request._header["request_id"]
    
    def query_task_status(self, task_id: str) -> str:  # ✅ str 类型
        """查询任务状态"""
        # ✅ 使用正确的函数名
        request = create_get_status_request(task_id=task_id)
        
        msg = String()
        msg.data = request.to_json()
        self.cmd_publisher.publish(msg)
        
        return request._header["request_id"]
```

### FastAPI 集成标准模式

```python
from fastapi import APIRouter, HTTPException, Depends
from pydantic import BaseModel
from typing import Optional

router = APIRouter(prefix="/api/tasks")

# 依赖注入
def get_node():
    from ..web_server import get_web_terminal_node
    node = get_web_terminal_node()
    if not node:
        raise HTTPException(503, "ROS node not initialized")
    return node

class NavigateRequest(BaseModel):
    x: float
    y: float
    yaw: float = 0.0

class TaskResponse(BaseModel):
    success: bool
    message: str
    request_id: str                     # SDK 生成的 UUID
    task_id: Optional[str] = None       # ✅ str 类型

@router.post("/navigate", response_model=TaskResponse)
async def create_navigation_task(req: NavigateRequest, node = Depends(get_node)):
    """创建导航任务"""
    try:
        # ✅ 通过 WebTerminalNode 发送
        request_id = node.navigate_to_pose(x=req.x, y=req.y, yaw=req.yaw)
        
        return TaskResponse(
            success=True,
            message="Navigation command sent",
            request_id=request_id
        )
    except Exception as e:
        raise HTTPException(500, str(e))
```

---

## 🔍 验证清单

### 设计文档验证 ✅

- [x] WEB_DESIGN.md § 2.2.1 SDK 使用规范
- [x] WEB_DESIGN.md 命令流程图
- [x] ARCHITECTURE_NOTES.md SDK 使用要点
- [x] 所有 `create_task_query_request` 已改为 `create_get_status_request`
- [x] 所有 `task_id: int` 已改为 `task_id: str`
- [x] 所有 `response.body.data` 已改为 `response.result`
- [x] 所有 `response.is_completed()` 已改为 `response.is_success()`
- [x] 所有工厂函数调用中移除了 `request_id` 参数

### 即将生成的代码验证 ⏳

当用户确认设计文档后，生成代码时需确保：

- [ ] `WebTerminalNode` 所有方法签名正确
- [ ] 响应处理使用 `response.result` 和 `response.is_success()`
- [ ] 所有 API 模型中 `task_id` 使用 `Optional[str]`
- [ ] 依赖注入正确传递 `WebTerminalNode` 实例
- [ ] 不直接调用任何 ROS 服务

---

## 📚 参考文档

- [SDK_INTERFACE_AUDIT.md](SDK_INTERFACE_AUDIT.md) - SDK 完整接口审核
- [WEB_DESIGN.md](WEB_DESIGN.md) - 完整设计文档（已修正）
- [ARCHITECTURE_NOTES.md](ARCHITECTURE_NOTES.md) - 架构核心要点（已修正）
- [bot_cmd_interface SDK](../../bot_cmd_interface/bot_cmd_interface/sdk/) - SDK 源码

---

**修正完成时间**: 2026-01-08  
**修正状态**: ✅ 所有设计文档已符合 SDK 实际接口  
**下一步**: 等待用户确认后重新生成代码

---

## ⚠️ 重要提示

**请用户仔细review以下修正点**：

1. **函数名变更**: `create_task_query_request` → `create_get_status_request`
2. **参数类型**: `task_id` 从 `int` 改为 `str`
3. **响应字段**: `response.body.data` → `response.result`
4. **状态检查**: `response.is_completed()` → `response.is_success()`
5. **request_id**: SDK 自动生成，工厂函数不接受此参数

确认这些修正符合实际需求后，我们将重新生成完全正确的代码。
