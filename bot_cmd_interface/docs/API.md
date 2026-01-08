# API 参考文档 - bot_cmd_interface

**版本**: v1.0.0  
**最后更新**: 2026-01-08

本文档提供 `bot_cmd_interface` 包的完整 API 参考。

---

## 目录

1. [SDK 模块](#sdk-模块)
   - [ActionType](#actiontype)
   - [CommandRequest](#commandrequest)
   - [CommandResponse](#commandresponse)
   - [便捷构造函数](#便捷构造函数)
   - [验证器](#验证器)
2. [核心组件](#核心组件)
   - [RequestQueue](#requestqueue)
   - [ServiceAdapter](#serviceadapter)
   - [ResponsePublisher](#responsepublisher)
3. [ROS2 接口](#ros2-接口)
   - [Topics](#topics)
   - [Parameters](#parameters)
4. [消息格式](#消息格式)
   - [请求消息格式](#请求消息格式)
   - [响应消息格式](#响应消息格式)

---

## SDK 模块

### ActionType

**位置**: `bot_cmd_interface.sdk.action_types`

动作类型常量定义。

#### 类属性

```python
class ActionType:
    """命令动作类型常量 / Command action type constants"""
    
    # 导航类 / Navigation
    NAVIGATE_TO_POSE = "navigate_to_pose"
    NAVIGATE_TO_LOCATION = "navigate_to_location"
    
    # 任务类 / Task
    START_EXPLORATION = "start_exploration"
    START_PATROL = "start_patrol"
    PAUSE_TASK = "pause_task"
    RESUME_TASK = "resume_task"
    CANCEL_TASK = "cancel_task"
    
    # 控制类 / Control
    EMERGENCY_STOP = "emergency_stop"
    
    # 查询类 / Query
    GET_ROBOT_STATUS = "get_robot_status"
    GET_TASK_STATUS = "get_task_status"
    
    # 地图类 / Map
    LIST_SAVED_MAPS = "list_saved_maps"
    LOAD_MAP = "load_map"
    SAVE_MAP = "save_map"
    
    ALL_ACTIONS = [...]  # 所有动作的列表
```

#### 类方法

##### `is_valid_action(action: str) -> bool`

检查动作是否有效。

**参数**:
- `action` (str): 动作类型字符串

**返回**:
- `bool`: 如果是有效动作返回 True

**示例**:
```python
from bot_cmd_interface.sdk import ActionType

if ActionType.is_valid_action("navigate_to_pose"):
    print("Valid action")
```

##### `get_category(action: str) -> str`

获取动作的分类。

**参数**:
- `action` (str): 动作类型字符串

**返回**:
- `str`: 分类名称（"navigation", "task", "query", "control", "map"）

**示例**:
```python
category = ActionType.get_category(ActionType.NAVIGATE_TO_POSE)
# 返回: "navigation"
```

---

### CommandRequest

**位置**: `bot_cmd_interface.sdk.message`

请求消息类。

#### 构造函数

```python
CommandRequest(
    action: str,
    params: Dict[str, Any] = None,
    request_id: str = None,
    source: str = "unknown",
    timestamp: float = None
)
```

**参数**:
- `action` (str): 动作类型（必须是 ActionType 中的常量）
- `params` (dict, optional): 动作参数，默认为 `{}`
- `request_id` (str, optional): 请求ID，默认自动生成
- `source` (str, optional): 请求来源，默认为 "unknown"
- `timestamp` (float, optional): 时间戳，默认为当前时间

**自动生成的 request_id 格式**: `{action}_{YYYYMMDD}_{HHMMSS}`

**示例**:
```python
from bot_cmd_interface.sdk import CommandRequest, ActionType

request = CommandRequest(
    action=ActionType.NAVIGATE_TO_POSE,
    params={
        "goal_pose": {
            "position": {"x": 1.0, "y": 2.0, "z": 0.0},
            "orientation": {"yaw": 0.785}
        }
    },
    source="my_terminal"
)
```

#### 属性

- `action` (str): 动作类型
- `params` (dict): 动作参数
- `request_id` (str): 请求ID
- `source` (str): 请求来源
- `timestamp` (float): 时间戳

#### 方法

##### `to_dict() -> dict`

转换为字典。

**返回**:
- `dict`: 请求的字典表示

##### `to_json(indent: int = None) -> str`

转换为 JSON 字符串。

**参数**:
- `indent` (int, optional): JSON 缩进，None 表示紧凑格式

**返回**:
- `str`: JSON 字符串

**示例**:
```python
json_str = request.to_json()
# {"action": "navigate_to_pose", "params": {...}, ...}
```

##### `from_dict(data: dict) -> CommandRequest` (类方法)

从字典创建请求。

**参数**:
- `data` (dict): 请求字典

**返回**:
- `CommandRequest`: 请求对象

##### `from_json(json_str: str) -> CommandRequest` (类方法)

从 JSON 字符串创建请求。

**参数**:
- `json_str` (str): JSON 字符串

**返回**:
- `CommandRequest`: 请求对象

**示例**:
```python
request = CommandRequest.from_json('{"action": "navigate_to_pose", ...}')
```

---

### CommandResponse

**位置**: `bot_cmd_interface.sdk.message`

响应消息类。

#### 构造函数

```python
CommandResponse(
    request_id: str,
    status: str,
    message: str = "",
    data: Dict[str, Any] = None,
    timestamp: float = None
)
```

**参数**:
- `request_id` (str): 关联的请求ID（必需）
- `status` (str): 响应状态（"queued" | "executing" | "completed" | "failed" | "cancelled"）
- `message` (str, optional): 响应消息，默认为空字符串
- `data` (dict, optional): 附加数据，默认为 `{}`
- `timestamp` (float, optional): 时间戳，默认为当前时间

**示例**:
```python
from bot_cmd_interface.sdk import CommandResponse

response = CommandResponse(
    request_id="nav_20260108_110532",
    status="completed",
    message="Navigation completed successfully",
    data={"result": "success", "distance_traveled": 5.2}
)
```

#### 属性

- `request_id` (str): 关联的请求ID
- `status` (str): 响应状态
- `message` (str): 响应消息
- `data` (dict): 附加数据
- `timestamp` (float): 时间戳

#### 方法

##### `to_dict() -> dict`

转换为字典。

##### `to_json(indent: int = None) -> str`

转换为 JSON 字符串。

##### `from_dict(data: dict) -> CommandResponse` (类方法)

从字典创建响应。

##### `from_json(json_str: str) -> CommandResponse` (类方法)

从 JSON 字符串创建响应。

##### `is_final() -> bool`

判断是否为最终状态（completed/failed/cancelled）。

**返回**:
- `bool`: 如果是最终状态返回 True

**示例**:
```python
if response.is_final():
    print("Task finished")
```

##### `is_success() -> bool`

判断是否成功完成。

**返回**:
- `bool`: 如果状态为 "completed" 返回 True

---

### 便捷构造函数

**位置**: `bot_cmd_interface.sdk.builders`

提供快速创建常用请求的函数。

#### `create_navigate_request(x: float, y: float, yaw: float = 0.0) -> CommandRequest`

创建导航请求。

**参数**:
- `x` (float): 目标 X 坐标
- `y` (float): 目标 Y 坐标
- `yaw` (float, optional): 目标朝向，默认 0.0

**返回**:
- `CommandRequest`: 导航请求对象

**示例**:
```python
from bot_cmd_interface.sdk import create_navigate_request

request = create_navigate_request(1.5, 2.0, 0.785)
```

#### `create_goto_request(location_name: str) -> CommandRequest`

创建前往命名位置的请求。

**参数**:
- `location_name` (str): 位置名称（如 "kitchen", "living_room"）

**返回**:
- `CommandRequest`: 导航请求对象

**示例**:
```python
request = create_goto_request("kitchen")
```

#### `create_exploration_request(map_name: str = "", save_on_completion: bool = False) -> CommandRequest`

创建探索请求。

**参数**:
- `map_name` (str, optional): 地图名称，默认自动生成
- `save_on_completion` (bool, optional): 完成时是否保存，默认 False

**返回**:
- `CommandRequest`: 探索请求对象

**示例**:
```python
request = create_exploration_request("floor_1", save_on_completion=True)
```

#### `create_patrol_request(waypoint_file: str, mode: str = "loop") -> CommandRequest`

创建巡逻请求。

**参数**:
- `waypoint_file` (str): 航点文件路径
- `mode` (str, optional): 巡逻模式（"loop" | "once" | "bounce"），默认 "loop"

**返回**:
- `CommandRequest`: 巡逻请求对象

**示例**:
```python
request = create_patrol_request("/path/to/waypoints.yaml", mode="loop")
```

#### `create_pause_request(task_id: str) -> CommandRequest`

创建暂停任务请求。

**参数**:
- `task_id` (str): 任务ID

**返回**:
- `CommandRequest`: 暂停请求对象

#### `create_resume_request(task_id: str) -> CommandRequest`

创建恢复任务请求。

**参数**:
- `task_id` (str): 任务ID

**返回**:
- `CommandRequest`: 恢复请求对象

#### `create_cancel_request(task_id: str) -> CommandRequest`

创建取消任务请求。

**参数**:
- `task_id` (str): 任务ID

**返回**:
- `CommandRequest`: 取消请求对象

#### `create_emergency_stop_request() -> CommandRequest`

创建紧急停止请求。

**返回**:
- `CommandRequest`: 紧急停止请求对象

**示例**:
```python
request = create_emergency_stop_request()
```

#### `create_robot_status_request() -> CommandRequest`

创建机器人状态查询请求。

**返回**:
- `CommandRequest`: 状态查询请求对象

#### `create_task_status_request(task_id: str) -> CommandRequest`

创建任务状态查询请求。

**参数**:
- `task_id` (str): 任务ID

**返回**:
- `CommandRequest`: 任务状态查询请求对象

**示例**:
```python
request = create_task_status_request("nav_20260108_110532")
```

#### `create_list_maps_request() -> CommandRequest`

创建列出地图请求。

**返回**:
- `CommandRequest`: 列出地图请求对象

#### `create_load_map_request(map_name: str) -> CommandRequest`

创建加载地图请求。

**参数**:
- `map_name` (str): 地图名称

**返回**:
- `CommandRequest`: 加载地图请求对象

#### `create_save_map_request(map_name: str) -> CommandRequest`

创建保存地图请求。

**参数**:
- `map_name` (str): 地图名称

**返回**:
- `CommandRequest`: 保存地图请求对象

---

### 验证器

**位置**: `bot_cmd_interface.sdk.validators`

提供 JSON Schema 验证功能。

#### `validate_request(request_dict: dict) -> tuple[bool, str]`

验证请求消息格式。

**参数**:
- `request_dict` (dict): 请求字典

**返回**:
- `tuple[bool, str]`: (是否有效, 错误消息)

**示例**:
```python
from bot_cmd_interface.sdk import validate_request

is_valid, error_msg = validate_request(request.to_dict())
if not is_valid:
    print(f"Invalid request: {error_msg}")
```

#### `validate_response(response_dict: dict) -> tuple[bool, str]`

验证响应消息格式。

**参数**:
- `response_dict` (dict): 响应字典

**返回**:
- `tuple[bool, str]`: (是否有效, 错误消息)

---

## 核心组件

### RequestQueue

**位置**: `bot_cmd_interface.components.request_queue`

请求队列管理器，负责队列操作和去重。

#### 构造函数

```python
RequestQueue(
    max_size: int = 100,
    timeout_seconds: float = 300.0,
    deduplication_window_seconds: float = 5.0
)
```

**参数**:
- `max_size` (int): 最大队列大小
- `timeout_seconds` (float): 请求超时时间（秒）
- `deduplication_window_seconds` (float): 去重窗口时间（秒）

#### 方法

##### `add(request: CommandRequest) -> tuple[bool, str]`

添加请求到队列。

**参数**:
- `request` (CommandRequest): 请求对象

**返回**:
- `tuple[bool, str]`: (是否成功, 消息)

**可能的失败原因**:
- 队列已满
- 请求重复（在去重窗口内）

##### `get_next() -> CommandRequest | None`

获取下一个待处理请求。

**返回**:
- `CommandRequest | None`: 请求对象或 None

##### `is_empty() -> bool`

检查队列是否为空。

##### `size() -> int`

获取当前队列大小。

##### `clear()`

清空队列。

##### `remove_expired()`

移除过期请求。

---

### ServiceAdapter

**位置**: `bot_cmd_interface.components.service_adapter`

异步服务适配器，将请求转发到后端服务。

#### 构造函数

```python
ServiceAdapter(node: Node)
```

**参数**:
- `node` (Node): ROS2 节点

#### 方法

##### `async call_service(request: CommandRequest) -> CommandResponse`

异步调用服务。

**参数**:
- `request` (CommandRequest): 请求对象

**返回**:
- `CommandResponse`: 响应对象

**注意**: 这是一个异步方法，需要使用 `await` 调用。

---

### ResponsePublisher

**位置**: `bot_cmd_interface.components.response_publisher`

响应发布器，负责发布响应消息到 Topic。

#### 构造函数

```python
ResponsePublisher(node: Node, topic: str = '/cmd/response')
```

**参数**:
- `node` (Node): ROS2 节点
- `topic` (str): 发布的 Topic 名称

#### 方法

##### `publish(response: CommandResponse)`

发布响应消息。

**参数**:
- `response` (CommandResponse): 响应对象

##### `publish_queued(request_id: str, message: str = "Request queued")`

发布 "queued" 状态响应。

**参数**:
- `request_id` (str): 请求ID
- `message` (str): 响应消息

##### `publish_executing(request_id: str, message: str = "Request executing", data: dict = None)`

发布 "executing" 状态响应。

**参数**:
- `request_id` (str): 请求ID
- `message` (str): 响应消息
- `data` (dict, optional): 附加数据（如进度）

##### `publish_completed(request_id: str, message: str = "Request completed", data: dict = None)`

发布 "completed" 状态响应。

**参数**:
- `request_id` (str): 请求ID
- `message` (str): 响应消息
- `data` (dict, optional): 附加数据（如结果）

##### `publish_failed(request_id: str, message: str = "Request failed", data: dict = None)`

发布 "failed" 状态响应。

**参数**:
- `request_id` (str): 请求ID
- `message` (str): 响应消息
- `data` (dict, optional): 附加数据（如错误详情）

---

## ROS2 接口

### Topics

#### `/cmd/request`

**类型**: `std_msgs/String`  
**方向**: 终端 → CommandAdapter  
**内容**: CommandRequest 的 JSON 序列化

**示例**:
```json
{
  "action": "navigate_to_pose",
  "params": {
    "goal_pose": {
      "position": {"x": 1.0, "y": 2.0, "z": 0.0},
      "orientation": {"yaw": 0.785}
    }
  },
  "request_id": "nav_20260108_110532",
  "source": "cli_terminal",
  "timestamp": 1704697532.123
}
```

#### `/cmd/response`

**类型**: `std_msgs/String`  
**方向**: CommandAdapter → 终端  
**内容**: CommandResponse 的 JSON 序列化

**示例**:
```json
{
  "request_id": "nav_20260108_110532",
  "status": "completed",
  "message": "Navigation completed successfully",
  "data": {
    "result": "success",
    "distance_traveled": 5.2
  },
  "timestamp": 1704697542.456
}
```

### Parameters

CommandAdapter 节点参数（在 `command_config.yaml` 中配置）：

| 参数名 | 类型 | 默认值 | 说明 |
|--------|------|--------|------|
| `max_queue_size` | int | 100 | 最大队列大小 |
| `queue_timeout_seconds` | float | 300.0 | 请求超时时间（秒） |
| `deduplication_window_seconds` | float | 5.0 | 去重窗口时间（秒） |
| `request_topic` | string | '/cmd/request' | 请求 Topic 名称 |
| `response_topic` | string | '/cmd/response' | 响应 Topic 名称 |
| `log_level` | string | 'INFO' | 日志级别 |

---

## 消息格式

### 请求消息格式

```json
{
  "action": "string (必需)",
  "params": {
    "key": "value (可选)"
  },
  "request_id": "string (可选，自动生成)",
  "source": "string (可选，默认 'unknown')",
  "timestamp": 1234567890.123
}
```

**字段说明**:

- `action` (string): 动作类型，必须是 ActionType 中的常量之一
- `params` (object): 动作参数，格式取决于具体动作
- `request_id` (string): 唯一请求ID，用于追踪请求生命周期
- `source` (string): 请求来源标识（如 "voice_terminal", "web_terminal"）
- `timestamp` (number): UNIX 时间戳（秒，浮点数）

### 响应消息格式

```json
{
  "request_id": "string (必需)",
  "status": "queued|executing|completed|failed|cancelled",
  "message": "string (可选)",
  "data": {
    "key": "value (可选)"
  },
  "timestamp": 1234567890.123
}
```

**字段说明**:

- `request_id` (string): 关联的请求ID
- `status` (string): 响应状态
  - `queued`: 请求已入队
  - `executing`: 正在执行
  - `completed`: 成功完成
  - `failed`: 执行失败
  - `cancelled`: 已取消
- `message` (string): 人类可读的响应消息
- `data` (object): 附加数据（如进度、结果、错误详情）
- `timestamp` (number): UNIX 时间戳（秒，浮点数）

---

## 错误处理

### 错误类型

**RequestQueueFull**: 队列已满

```python
# 请求被拒绝时返回
success, message = queue.add(request)
if not success:
    print(f"Failed to add request: {message}")
```

**DuplicateRequest**: 重复请求

```python
# 在去重窗口内发送相同请求
# 响应状态: "failed"
# 响应消息: "Duplicate request detected within 5.0 seconds"
```

**ServiceCallFailed**: 服务调用失败

```python
# 后端服务不可用或调用超时
# 响应状态: "failed"
# 响应消息: "Service call failed: {error_detail}"
```

---

## 性能指标

基于 [test_cmd_benchmark.py](../test/test_cmd_benchmark.py) 的测试结果：

| 指标 | 目标 | 实测 | 状态 |
|------|------|------|------|
| Queued 响应时间 | < 50ms | 3.7ms (median) | ✅ 优秀 |
| Completed 响应时间 | < 100ms | 13.9ms (median) | ✅ 优秀 |
| 吞吐量 | > 100 req/s | 960 req/s | ✅ 优秀 |
| 并发处理 | > 10 requests | 15+ | ✅ 良好 |

---

## 使用示例

完整的使用示例请参考：

- [test/test_integration.py](../test/test_integration.py) - 集成测试示例
- [test/cmd_terminal.py](../test/cmd_terminal.py) - 交互式终端实现
- [README.md](../README.md) - 快速开始指南

---

## 扩展开发

### 添加新的动作类型

1. 在 `sdk/action_types.py` 中添加常量
2. 在 `sdk/builders.py` 中添加便捷构造函数
3. 在 `ServiceAdapter` 中添加服务调用逻辑
4. 更新测试用例

### 自定义响应数据格式

响应的 `data` 字段可以包含任意 JSON 可序列化的数据：

```python
# 进度更新
response = CommandResponse(
    request_id="...",
    status="executing",
    message="Navigating...",
    data={"progress": 0.65, "eta_seconds": 12.5}
)

# 结果数据
response = CommandResponse(
    request_id="...",
    status="completed",
    message="Exploration completed",
    data={
        "result": "success",
        "map_size": {"width": 50, "height": 30},
        "explored_area": 1200.5,
        "duration_seconds": 180.2
    }
)
```

---

**API 参考完成** ✅

如有疑问或需要更多细节，请参考源代码或联系维护者。
