# bot_cmd_interface SDK 接口审核报告

**审核日期**: 2026-01-08  
**审核目的**: 确保 WEB_DESIGN.md 中的架构设计完全符合 bot_cmd_interface SDK 实际接口

---

## ✅ SDK 实际接口（v1.0.0）

### 1. 导出的工厂函数

**位置**: `bot_cmd_interface/sdk/__init__.py`

```python
from .message import (
    CommandRequest,
    CommandResponse,
    ResponseStatus,
    ErrorCode,
    create_navigate_request,          # ✅ 导航
    create_patrol_request,             # ✅ 巡逻
    create_exploration_request,        # ✅ 探索
    create_emergency_stop_request,     # ✅ 紧急停止
    create_get_status_request,         # ✅ 查询状态（注意不是 create_task_query_request！）
    create_cancel_task_request,        # ✅ 取消任务
)
```

---

### 2. 工厂函数签名（完整版）

#### 2.1 `create_navigate_request`

```python
def create_navigate_request(
    x: float,
    y: float,
    yaw: Optional[float] = None,      # ✅ 可选参数
    timeout: Optional[float] = None,
    priority: int = 3
) -> CommandRequest:
```

**参数结构**:
```json
{
  "action": "navigate_to_pose",
  "params": {
    "goal_pose": {
      "position": {"x": 1.5, "y": 2.0},
      "orientation": {"yaw": 0.785}    // 可选
    }
  }
}
```

#### 2.2 `create_exploration_request`

```python
def create_exploration_request(
    map_name: Optional[str] = None,         # ✅ 可选参数
    save_on_completion: bool = True,
    timeout: Optional[float] = None,
    priority: int = 3
) -> CommandRequest:
```

**默认超时**: 3600.0 秒（1小时）

#### 2.3 `create_patrol_request`

```python
def create_patrol_request(
    waypoint_file: str,                     # ✅ 必需参数
    mode: str = "loop",                     # "loop" 或 "once"
    timeout: Optional[float] = None,
    priority: int = 3
) -> CommandRequest:
```

**默认超时**: 3600.0 秒（1小时）

#### 2.4 `create_get_status_request` ⚠️ **关键名称**

```python
def create_get_status_request(
    task_id: Optional[str] = None,          # ✅ 注意是 str 不是 int！
    request_id: Optional[str] = None,
    timeout: Optional[float] = None,
    priority: int = 3
) -> CommandRequest:
```

**重要**:
- ❌ 设计文档中错误使用了 `create_task_query_request`
- ✅ 正确名称是 `create_get_status_request`
- ❌ `task_id` 不是 `int` 类型，是 `Optional[str]`
- ✅ 至少提供 `task_id` 或 `request_id` 之一

**默认超时**: 5.0 秒

#### 2.5 `create_cancel_task_request`

```python
def create_cancel_task_request(
    task_id: Optional[str] = None,          # ✅ str 类型
    request_id: Optional[str] = None,
    timeout: Optional[float] = None,
    priority: int = 2
) -> CommandRequest:
```

**默认超时**: 10.0 秒

#### 2.6 `create_emergency_stop_request`

```python
def create_emergency_stop_request(
    force_immediate: bool = False,
    priority: int = 1                       # 最高优先级
) -> CommandRequest:
```

**固定超时**: 5.0 秒

---

### 3. CommandRequest 类

```python
class CommandRequest:
    def __init__(
        self,
        action: str,                         # 动作类型
        params: dict,                        # 参数字典
        request_id: Optional[str] = None,    # ✅ 可选，自动生成 UUID
        priority: int = 3,                   # 1-5，默认3
        timeout: float = 300.0               # 超时时间（秒）
    ):
```

**方法**:
- `to_json() -> str`: 序列化为 JSON 字符串
- `to_dict() -> dict`: 转换为字典
- `@classmethod from_json(json_str: str) -> CommandRequest`: 从 JSON 反序列化

**消息格式**:
```json
{
  "header": {
    "request_id": "uuid-string",           // SDK 自动生成
    "timestamp": "2026-01-08T12:00:00Z",   // SDK 自动生成
    "priority": 3
  },
  "body": {
    "action": "navigate_to_pose",
    "params": {...},
    "timeout": 300.0
  }
}
```

---

### 4. CommandResponse 类 ⚠️ **关键结构**

```python
class CommandResponse:
    def __init__(
        self,
        request_id: str,
        status: str,                         # ResponseStatus 常量
        message: str,
        code: int = 0,                       # ErrorCode 常量
        result: Optional[dict] = None,       # ✅ 注意是 result 不是 data！
        progress: Optional[float] = None,    # 0.0-1.0
        warnings: Optional[List[str]] = None
    ):
```

**方法**:
- `is_success() -> bool`: 检查是否成功（⚠️ 不是 `is_completed()`！）
- `is_final() -> bool`: 检查是否为最终响应
- `to_json() -> str`: 序列化
- `@classmethod from_json(json_str: str) -> CommandResponse`: 反序列化

**响应格式**:
```json
{
  "header": {
    "request_id": "uuid-string",
    "timestamp": "2026-01-08T12:00:01Z",
    "status": "completed"                  // queued | executing | completed | failed | cancelled
  },
  "body": {
    "message": "Task created successfully",
    "code": 0,                              // 0 = 成功
    "result": {                             // ✅ 注意字段名是 result
      "task_id": "task-001"                 // ✅ 任务ID是字符串
    },
    "progress": 0.0,                        // 可选
    "warnings": []                          // 可选
  }
}
```

---

### 5. ResponseStatus 常量

```python
class ResponseStatus:
    QUEUED = "queued"
    EXECUTING = "executing"
    COMPLETED = "completed"
    FAILED = "failed"
    CANCELLED = "cancelled"
```

---

### 6. ErrorCode 常量

```python
class ErrorCode:
    SUCCESS = 0
    BAD_REQUEST = 400
    NOT_FOUND = 404
    CONFLICT = 409
    CLIENT_CLOSED = 499
    INTERNAL_ERROR = 500
    SERVICE_UNAVAILABLE = 503
    GATEWAY_TIMEOUT = 504
```

---

### 7. ActionType 常量

```python
class ActionType:
    NAVIGATE_TO_POSE = "navigate_to_pose"
    START_EXPLORATION = "start_exploration"
    START_PATROL = "start_patrol"
    STOP_PATROL = "stop_patrol"
    PAUSE_TASK = "pause_task"
    RESUME_TASK = "resume_task"
    CANCEL_TASK = "cancel_task"
    GET_TASK_STATUS = "get_task_status"       # ✅ 查询状态动作
    GET_ROBOT_STATUS = "get_robot_status"
    EMERGENCY_STOP = "emergency_stop"
    SAVE_MAP = "save_map"
    LOAD_MAP = "load_map"
```

---

## ❌ 设计文档中发现的错误

### 错误 1: 函数名错误

**WEB_DESIGN.md 第 174 行**:
```python
# ❌ 错误
from bot_cmd_interface.sdk import create_task_query_request

# ✅ 正确
from bot_cmd_interface.sdk import create_get_status_request
```

**修正位置**:
- WEB_DESIGN.md § 2.2.1
- ARCHITECTURE_NOTES.md

---

### 错误 2: 参数类型错误

**WEB_DESIGN.md 示例代码**:
```python
# ❌ 错误：task_id 使用 int 类型
request = create_get_status_request(task_id=1001)

# ✅ 正确：task_id 是字符串
request = create_get_status_request(task_id="task-001")
```

**影响范围**:
- 所有 API 端点的 task_id 参数必须是字符串
- 数据库/缓存中的 task_id 也应该用字符串

---

### 错误 3: 响应字段名错误

**WEB_DESIGN.md 示例代码**:
```python
# ❌ 错误：使用 data 字段
task_id = response.body.data.get('task_id')

# ✅ 正确：使用 result 字段
task_id = response.result.get('task_id')
```

---

### 错误 4: 状态检查方法错误

**WEB_DESIGN.md 示例代码**:
```python
# ❌ 错误
if response.is_completed():
    ...

# ✅ 正确
if response.is_success():
    ...

# 或者检查状态常量
if response.status == ResponseStatus.COMPLETED:
    ...
```

---

### 错误 5: request_id 生成逻辑

**SDK 行为**:
- 如果不提供 `request_id`，SDK 自动生成 UUID
- 工厂函数**不接受** `request_id` 参数！

**WEB_DESIGN.md 示例代码**:
```python
# ❌ 错误：工厂函数不接受 request_id 参数
request = create_navigate_request(x=2.0, y=3.0, request_id="web-nav-1")

# ✅ 正确：手动设置（需要先创建再修改）
request = create_navigate_request(x=2.0, y=3.0)
request._header["request_id"] = "web-nav-1"  # 不推荐

# ✅ 推荐：使用 CommandRequest 类直接构造
from bot_cmd_interface.sdk import CommandRequest, ActionType
request = CommandRequest(
    action=ActionType.NAVIGATE_TO_POSE,
    params={"goal_pose": {"position": {"x": 2.0, "y": 3.0}}},
    request_id="web-nav-1"
)
```

**注意**: 工厂函数签名中没有 `request_id` 参数！

---

## ✅ 修正建议

### 1. WEB_DESIGN.md 需要修正的章节

- ❌ § 2.2.1 "SDK 使用规范" - 函数名、参数类型、响应字段
- ❌ § 4.3.2 "WebTerminalNode 实现示例" - 所有代码示例
- ❌ § 4.3.3 "FastAPI 集成示例" - task_id 类型
- ❌ § 4.3.4 "错误处理" - 状态检查方法

### 2. ARCHITECTURE_NOTES.md 需要修正

- ❌ "SDK 使用要点" - 所有示例代码
- ❌ "创建请求" - 函数名和参数

### 3. 代码生成时的注意事项

**WebTerminalNode 实现**:
```python
class WebTerminalNode(Node):
    def query_task_status(self, task_id: str) -> str:  # ✅ str 类型
        """查询任务状态"""
        # ✅ 使用正确的函数名
        request = create_get_status_request(task_id=task_id)
        
        msg = String()
        msg.data = request.to_json()
        self.cmd_publisher.publish(msg)
        
        return request._header["request_id"]  # 返回 SDK 生成的 request_id
```

**响应处理**:
```python
def _response_callback(self, msg: String):
    response = CommandResponse.from_json(msg.data)
    
    # ✅ 使用正确的状态检查
    if response.is_success():
        # ✅ 使用 result 字段
        task_id = response.result.get('task_id') if response.result else None
        self.get_logger().info(f"Task created: {task_id}")
```

**FastAPI 模型**:
```python
class TaskResponse(BaseModel):
    success: bool
    message: str
    task_id: Optional[str] = None  # ✅ str 类型
    request_id: str
```

---

## 📋 修正检查清单

- [ ] WEB_DESIGN.md § 2.2.1 SDK 使用规范代码
- [ ] WEB_DESIGN.md § 4.3 全部示例代码
- [ ] ARCHITECTURE_NOTES.md 全部示例代码
- [ ] 确认所有 task_id 使用字符串类型
- [ ] 确认所有响应解析使用 `response.result`
- [ ] 确认所有状态检查使用 `response.is_success()`
- [ ] 确认函数名使用 `create_get_status_request`
- [ ] 确认 request_id 生成策略（SDK 自动生成）

---

**审核结论**: 设计文档存在多处与 SDK 实际接口不符的错误，必须全面修正后再生成代码。

**修正优先级**: 🔴 高优先级 - 必须在代码生成前完成
