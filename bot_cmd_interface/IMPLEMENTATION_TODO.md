# bot_cmd_interface 实现TODO计划

**版本**: v1.0.0  
**创建日期**: 2026-01-07  
**基于设计文档**: ARCHITECTURE.md v0.3.0  
**状态**: 📋 待开始

---

## 📊 实施总览

**实施策略**: 分4个阶段递进实现，每阶段独立测试验证

| 阶段 | 内容 | 预计工作量 | 关键产出 |
|------|------|-----------|---------|
| Phase 1 | 基础框架 + SDK | 3-4天 | SDK模块、基础类、单元测试 |
| Phase 2 | 核心组件 | 4-5天 | CommandAdapter、队列管理、服务适配 |
| Phase 3 | 测试验证 | 2-3天 | 集成测试、性能测试、模拟终端 |
| Phase 4 | 文档完善 | 1-2天 | README、API文档、示例代码 |

**总预计**: 10-14天

---

## Phase 1: 基础框架与SDK模块 (3-4天)

### 1.1 项目结构搭建

**优先级**: P0 (必须) | **估时**: 0.5天

- [ ] **1.1.1** 创建包目录结构
  ```bash
  bot_cmd_interface/
  ├── bot_cmd_interface/
  │   ├── __init__.py
  │   ├── sdk/                    # SDK模块
  │   │   ├── __init__.py
  │   │   ├── message.py          # CommandRequest/CommandResponse
  │   │   ├── action_types.py     # ActionType常量
  │   │   └── validators.py       # 验证器
  │   ├── command_adapter_node.py # 主节点
  │   ├── components/             # 核心组件
  │   │   ├── __init__.py
  │   │   ├── request_queue.py
  │   │   ├── service_adapter.py
  │   │   └── response_publisher.py
  │   └── utils/                  # 工具模块
  │       ├── __init__.py
  │       └── logger.py
  ├── config/
  │   ├── command_config.yaml
  │   └── location_map.yaml
  ├── launch/
  │   └── cmd_adapter.launch.py
  ├── test/
  │   ├── test_sdk.py
  │   ├── test_queue.py
  │   └── test_adapter.py
  ├── setup.py
  ├── package.xml
  ├── README.md
  ├── ARCHITECTURE.md             # 已存在
  └── IMPLEMENTATION_TODO.md      # 本文件
  ```

- [ ] **1.1.2** 配置setup.py
  - 定义包信息 (name, version='1.0.0', description)
  - 添加依赖: `jsonschema>=4.0.0`, `pyyaml`
  - 配置entry_points: `command_adapter`节点

- [ ] **1.1.3** 配置package.xml
  - 声明ROS2包依赖: `rclpy`, `std_msgs`, `bot_navigation_msgs`
  - 添加构建依赖: `ament_cmake_python`
  - 添加测试依赖: `pytest`, `ament_lint_auto`

**验收标准**:
- ✅ 目录结构完整，所有`__init__.py`就位
- ✅ `colcon build --packages-select bot_cmd_interface`编译成功
- ✅ `ros2 pkg list | grep bot_cmd_interface`可见

---

### 1.2 SDK模块核心类实现

**优先级**: P0 (必须) | **估时**: 1.5天

#### 1.2.1 ActionType常量定义

**文件**: `sdk/action_types.py`

- [ ] **1.2.1.1** 实现ActionType类
  - 按设计文档Section 5.1定义13个动作常量
  - 添加`ALL_ACTIONS`列表
  - 添加`is_valid_action(action: str) -> bool`方法
  - **中英文双语注释** / Bilingual comments

**参考代码**:
```python
class ActionType:
    """命令动作类型常量 / Command action type constants"""
    
    # 导航类 / Navigation
    NAVIGATE_TO_POSE = "navigate_to_pose"
    NAVIGATE_TO_LOCATION = "navigate_to_location"  # Phase 1.3新增 / Added in Phase 1.3
    
    # 任务类 / Task
    START_EXPLORATION = "start_exploration"
    START_PATROL = "start_patrol"
    STOP_PATROL = "stop_patrol"
    PAUSE_TASK = "pause_task"
    RESUME_TASK = "resume_task"
    CANCEL_TASK = "cancel_task"
    
    # 查询类 / Query
    GET_TASK_STATUS = "get_task_status"
    GET_ROBOT_STATUS = "get_robot_status"
    
    # 控制类 / Control
    EMERGENCY_STOP = "emergency_stop"
    
    # 地图类 / Map
    SAVE_MAP = "save_map"
    LOAD_MAP = "load_map"
    
    ALL_ACTIONS = [
        NAVIGATE_TO_POSE, NAVIGATE_TO_LOCATION,
        START_EXPLORATION, START_PATROL, STOP_PATROL,
        PAUSE_TASK, RESUME_TASK, CANCEL_TASK,
        GET_TASK_STATUS, GET_ROBOT_STATUS,
        EMERGENCY_STOP, SAVE_MAP, LOAD_MAP
    ]
    
    @staticmethod
    def is_valid_action(action: str) -> bool:
        return action in ActionType.ALL_ACTIONS
```

**验收标准**:
- ✅ 13个动作常量完整定义
- ✅ `ALL_ACTIONS`包含所有动作
- ✅ `is_valid_action()`测试通过

---

#### 1.2.2 CommandRequest类实现

**文件**: `sdk/message.py`

- [ ] **1.2.2.1** 实现`CommandRequest`类
  - 构造函数: `__init__(action, params, request_id=None, priority=3, timeout=300.0)`
  - `request_id`默认使用`uuid.uuid4()`生成
  - `timestamp`使用`datetime.now().isoformat()`
  - 实现`to_json() -> str`方法 (按设计文档Section 2.1.1格式)
  - 实现`from_json(json_str: str) -> CommandRequest`类方法
  - 实现`validate() -> Tuple[bool, str]`验证方法
  - **中英文双语注释**

- [ ] **1.2.2.2** 实现便捷构造函数
  - `create_navigate_request(x, y, yaw=None, timeout=None) -> CommandRequest`
  - `create_patrol_request(waypoint_file, mode='loop', timeout=None) -> CommandRequest`
  - `create_exploration_request(map_name=None, save_on_completion=True) -> CommandRequest`
  - `create_emergency_stop_request(force_immediate=False) -> CommandRequest`
  - `create_get_status_request(task_id=None, request_id=None) -> CommandRequest`
  - `create_cancel_task_request(task_id=None, request_id=None) -> CommandRequest`
  
  **注意**: `create_navigate_to_location_request()` 在Phase 1.3实现

**参考代码片段**:
```python
def create_navigate_request(
    x: float,
    y: float,
    yaw: float = None,
    timeout: float = None
) -> CommandRequest:
    """
    创建导航到目标点请求 / Create navigate to pose request
    
    Args:
        x: 目标X坐标 / Target X coordinate
        y: 目标Y坐标 / Target Y coordinate
        yaw: 目标朝向（可选）/ Target yaw (optional)
        timeout: 超时时间（可选）/ Timeout (optional)
    """
    params = {
        "goal_pose": {
            "position": {"x": x, "y": y},
            "orientation": {"yaw": yaw} if yaw is not None else {}
        }
    }
    
    kwargs = {"action": ActionType.NAVIGATE_TO_POSE, "params": params}
    if timeout is not None:
        kwargs["timeout"] = timeout
    
    return CommandRequest(**kwargs)
```

**验收标准**:
- ✅ `CommandRequest`构造、序列化、反序列化测试通过
- ✅ 6个便捷函数单元测试通过
- ✅ `validate()`能检测出缺失字段

---

#### 1.2.3 CommandResponse类实现

**文件**: `sdk/message.py` (续)

- [ ] **1.2.3.1** 实现`CommandResponse`类
  - 构造函数: `__init__(request_id, status, message, code=0, result=None, progress=None, warnings=None)`
  - `status`必须是: `queued | executing | completed | failed | cancelled`
  - 实现`to_json() -> str` (按Section 2.1.2格式)
  - 实现`from_json(json_str: str) -> CommandResponse`
  - **特别注意**: `progress`仅在`status='completed' and result contains progress`时使用
  - **中英文双语注释**

- [ ] **1.2.3.2** 实现状态常量
  ```python
  class ResponseStatus:
      QUEUED = "queued"
      EXECUTING = "executing"
      COMPLETED = "completed"
      FAILED = "failed"
      CANCELLED = "cancelled"
  ```

**验收标准**:
- ✅ `CommandResponse`序列化/反序列化测试通过
- ✅ `progress`字段处理符合设计文档Section 2.2说明
- ✅ JSON格式严格符合设计文档示例

---

#### 1.2.4 ErrorCode枚举类实现

**文件**: `sdk/message.py` (续)

- [ ] **1.2.4.1** 实现`ErrorCode`类
  - 按设计文档Section 6.3定义8个错误码
  - 添加`get_message(code: int) -> str`方法返回错误描述
  - **中英文双语注释**

**参考代码**:
```python
class ErrorCode:
    """统一错误码定义 / Unified error code definitions"""
    
    SUCCESS = 0               # 成功 / Success
    BAD_REQUEST = 400         # 请求格式错误 / Bad request format
    NOT_FOUND = 404           # 资源未找到 / Resource not found
    CONFLICT = 409            # 状态冲突 / State conflict
    CLIENT_CLOSED = 499       # 客户端关闭连接 / Client closed connection
    INTERNAL_ERROR = 500      # 服务内部错误 / Internal server error
    SERVICE_UNAVAILABLE = 503 # 服务不可用 / Service unavailable
    GATEWAY_TIMEOUT = 504     # 网关超时 / Gateway timeout
    
    @staticmethod
    def get_message(code: int) -> str:
        messages = {
            0: "Success",
            400: "Bad request format",
            404: "Resource not found",
            409: "State conflict",
            499: "Client closed connection",
            500: "Internal server error",
            503: "Service unavailable",
            504: "Gateway timeout"
        }
        return messages.get(code, "Unknown error")
```

**验收标准**:
- ✅ 8个错误码定义完整
- ✅ `get_message()`能正确返回描述

---

#### 1.2.5 SDK验证器实现

**文件**: `sdk/validators.py`

- [ ] **1.2.5.1** 实现请求验证器
  ```python
  def validate_request(request: CommandRequest) -> Tuple[bool, str]:
      """
      验证请求消息合法性 / Validate request message
      
      检查项 / Checks:
      1. action是否合法 / Is action valid
      2. request_id是否存在 / request_id exists
      3. params是否为dict / params is dict
      4. 必需参数是否齐全 / Required params present
      """
  ```

- [ ] **1.2.5.2** 实现响应验证器
  ```python
  def validate_response(response: CommandResponse) -> Tuple[bool, str]:
      """验证响应消息合法性 / Validate response message"""
  ```

- [ ] **1.2.5.3** 实现动作参数验证
  - 为每个`ActionType`定义必需参数
  - `navigate_to_pose`: 需要`goal_pose.position.x/y`
  - `start_patrol`: 需要`waypoint_file`
  - `start_exploration`: 可选参数
  - `get_task_status`: 需要`task_id`或`request_id`至少一个
  - **使用jsonschema进行JSON Schema验证**

**验收标准**:
- ✅ 所有动作类型的参数验证规则定义完整
- ✅ 验证器单元测试覆盖率 > 90%

---

#### 1.2.6 SDK单元测试

**文件**: `test/test_sdk.py`

- [ ] **1.2.6.1** 测试ActionType
  - 测试所有常量值正确
  - 测试`is_valid_action()`

- [ ] **1.2.6.2** 测试CommandRequest
  - 测试构造函数
  - 测试`to_json()`和`from_json()`往返
  - 测试`validate()`各种边界情况
  - 测试6个便捷构造函数

- [ ] **1.2.6.3** 测试CommandResponse
  - 测试序列化/反序列化
  - 测试`progress`字段处理
  - 测试错误响应构造

- [ ] **1.2.6.4** 测试validators
  - 测试`validate_request()`各种非法输入
  - 测试`validate_response()`

**运行命令**:
```bash
cd ~/lododo_bot
source install/setup.bash
pytest src/bot_cmd_interface/test/test_sdk.py -v
```

**验收标准**:
- ✅ 所有测试通过
- ✅ 测试覆盖率 > 85%
- ✅ 无警告信息

---

### 1.3 配置文件管理

**优先级**: P1 (高) | **估时**: 0.5天

#### 1.3.1 command_config.yaml

**文件**: `config/command_config.yaml`

- [ ] **1.3.1.1** 创建配置文件
  - 按设计文档Section 7.2示例完整复制
  - 包含: queue配置、timeout配置、persistence配置、logging配置

**验收标准**:
- ✅ YAML格式正确，可被`yaml.load()`解析
- ✅ 所有参数有注释说明

---

#### 1.3.2 location_map.yaml

**文件**: `config/location_map.yaml`

- [ ] **1.3.2.1** 创建地点映射配置
  - 按设计文档Section 7.2示例完整复制
  - 初始包含5个示例地点: 厨房、客厅、卧室、阳台、充电桩

- [ ] **1.3.2.2** 实现配置加载工具类
  **文件**: `utils/config_loader.py`
  ```python
  class LocationMapLoader:
      """地点映射配置加载器 / Location map configuration loader"""
      
      def __init__(self, config_path: str):
          self.config_path = config_path
          self.locations = {}
          self._load_config()
      
      def get_pose(self, location_name: str) -> dict:
          """根据地点名称获取坐标 / Get pose by location name"""
          pass
      
      def list_locations(self) -> List[str]:
          """列出所有可用地点 / List all available locations"""
          pass
  ```

- [ ] **1.3.2.3** 扩展SDK支持地点导航
  **文件**: `sdk/message.py` (新增)
  ```python
  def create_navigate_to_location_request(
      location: str,
      timeout: float = None
  ) -> CommandRequest:
      """
      创建导航到命名地点请求 / Create navigate to named location request
      
      Note: 实际坐标由CommandAdapter从location_map.yaml解析
           / Actual coordinates resolved by CommandAdapter from location_map.yaml
      """
      params = {"location": location}
      return CommandRequest(
          action=ActionType.NAVIGATE_TO_LOCATION,
          params=params,
          timeout=timeout or 300.0
      )
  ```

**验收标准**:
- ✅ `location_map.yaml`格式正确
- ✅ `LocationMapLoader`能正确加载和查询
- ✅ `create_navigate_to_location_request()`测试通过

---

### 1.4 Phase 1 验收总结

- [ ] **1.4.1** 编译测试
  ```bash
  cd ~/lododo_bot
  colcon build --packages-select bot_cmd_interface --symlink-install
  source install/setup.bash
  ```

- [ ] **1.4.2** SDK导入测试
  ```bash
  python3 -c "from bot_cmd_interface.sdk import CommandRequest, ActionType; print('SDK import OK')"
  ```

- [ ] **1.4.3** 单元测试总报告
  ```bash
  pytest src/bot_cmd_interface/test/test_sdk.py -v --cov=bot_cmd_interface.sdk
  ```

**Phase 1 完成标志**:
- ✅ 包结构完整，编译无错误
- ✅ SDK所有类实现完成
- ✅ 单元测试覆盖率 > 85%
- ✅ 配置文件加载功能正常

---

## Phase 2: 核心组件实现 (4-5天)

### 2.1 RequestQueue请求队列

**优先级**: P0 (必须) | **估时**: 1.5天

**文件**: `components/request_queue.py`

#### 2.1.1 基础队列类

- [ ] **2.1.1.1** 实现`RequestQueue`类
  ```python
  class RequestQueue:
      """
      请求队列 - 基于请求ID的FIFO队列 / Request queue - FIFO queue based on request ID
      
      功能 / Features:
      1. 线程安全的队列操作 / Thread-safe queue operations
      2. 请求ID去重（5秒内相同内容）/ Request ID deduplication (5s window)
      3. 优先级管理（仅emergency_stop优先）/ Priority management (only emergency_stop has priority)
      4. 队列大小限制 / Queue size limit
      """
      
      def __init__(self, max_size: int = 100, dedup_window: float = 5.0):
          self.queue = queue.Queue(maxsize=max_size)
          self.dedup_cache = {}  # {request_id: (content_hash, timestamp)}
          self.dedup_window = dedup_window
          self.lock = threading.Lock()
      
      def enqueue(self, request: CommandRequest) -> Tuple[bool, str]:
          """入队请求 / Enqueue request"""
          pass
      
      def dequeue(self, timeout: float = None) -> CommandRequest:
          """出队请求 / Dequeue request"""
          pass
      
      def size(self) -> int:
          """队列大小 / Queue size"""
          pass
      
      def clear(self):
          """清空队列 / Clear queue"""
          pass
  ```

- [ ] **2.1.1.2** 实现去重逻辑
  - 计算请求内容哈希: `hash(json.dumps({"action": ..., "params": ...}, sort_keys=True))`
  - 5秒内相同哈希的请求拒绝入队
  - 定期清理过期缓存 (启动后台线程)

- [ ] **2.1.1.3** 实现优先级插队
  - `emergency_stop`请求直接插入队首
  - 其他请求按FIFO顺序

**验收标准**:
- ✅ 队列基本操作（enqueue/dequeue）测试通过
- ✅ 去重功能测试通过（5秒窗口）
- ✅ `emergency_stop`优先级测试通过
- ✅ 线程安全测试（多线程并发入队）

---

#### 2.1.2 队列单元测试

**文件**: `test/test_queue.py`

- [ ] **2.1.2.1** 测试基本队列操作
  - 测试入队/出队
  - 测试队列大小限制
  - 测试空队列超时dequeue

- [ ] **2.1.2.2** 测试去重机制
  - 测试5秒内重复请求被拒绝
  - 测试5秒后相同请求可重新入队
  - 测试不同request_id但相同内容的去重

- [ ] **2.1.2.3** 测试优先级
  - 测试`emergency_stop`插队
  - 测试其他请求FIFO顺序

- [ ] **2.1.2.4** 测试线程安全
  - 10个线程并发入队100个请求
  - 验证队列无数据丢失

**验收标准**:
- ✅ 所有测试通过
- ✅ 测试覆盖率 > 90%

---

### 2.2 ServiceAdapter服务适配器

**优先级**: P0 (必须) | **估时**: 2天

**文件**: `components/service_adapter.py`

#### 2.2.1 核心适配器类

- [ ] **2.2.1.1** 实现`ServiceAdapter`类
  ```python
  class ServiceAdapter:
      """
      服务适配器 - 将命令请求转换为ROS2 Service/Action调用 / Service adapter - Convert command requests to ROS2 Service/Action calls
      
      支持的服务 / Supported services:
      - /mission/start_exploration (StartExploration.srv)
      - /mission/start_patrol (StartPatrol.srv)
      - /mission/navigate_to_pose (NavigateToPose.action - 需使用action client)
      - /mission/get_task_status (GetTaskStatus.srv)
      - /mission/cancel_task (TaskControl.srv)
      - /mission/emergency_stop (EmergencyStop.srv)
      - /mission/pause_task (TaskControl.srv)
      - /mission/resume_task (TaskControl.srv)
      
      持久化映射 / Persistent mapping:
      - request_id -> task_id 映射表
      - 保存到文件，启动时加载
      """
      
      def __init__(self, node: Node, config: dict):
          self.node = node
          self.clients = {}
          self.action_clients = {}
          self.map_lock = threading.Lock()
          self.request_task_map = {}  # {request_id: task_id}
          self.persistence_file = config.get('persistence', {}).get('mapping_file', '~/.bot_cmd_interface/request_task_map.json')
          self._init_clients()
          self._load_mapping_from_file()
      
      def process_request(self, request: CommandRequest) -> dict:
          """
          处理请求，调用对应服务 / Process request, call corresponding service
          
          Returns:
              dict: {
                  'success': bool,
                  'message': str,
                  'code': int,
                  'result': dict,  # 可能包含task_id等信息
              }
          """
          pass
  ```

- [ ] **2.2.1.2** 实现服务客户端初始化
  - 创建所有需要的Service Client
  - 创建Action Client (用于`navigate_to_pose`)
  - 等待服务可用 (timeout=10s)

- [ ] **2.2.1.3** 实现请求分发逻辑
  - 根据`action`字段调用对应的处理函数
  - 使用策略模式: `_handlers = {ActionType.NAVIGATE_TO_POSE: self._call_navigate_to_pose, ...}`

**验收标准**:
- ✅ 所有服务客户端初始化成功
- ✅ 请求分发逻辑测试通过

---

#### 2.2.2 各Action处理函数

按照设计文档Section 3.1.3实现每个动作的处理函数：

- [ ] **2.2.2.1** `_call_navigate_to_pose(request_id, params) -> dict`
  - 解析`goal_pose`参数
  - 调用`/mission/navigate_to_pose` Action
  - 获取`task_id`，存储映射关系
  - 返回结果

- [ ] **2.2.2.2** `_call_navigate_to_location(request_id, params) -> dict`
  - 从`location_map.yaml`解析地点坐标
  - 转换为`navigate_to_pose`参数
  - 调用`_call_navigate_to_pose()`

- [ ] **2.2.2.3** `_call_start_exploration(request_id, params) -> dict`
  - 调用`/mission/start_exploration`服务
  - 处理`map_name`、`save_on_completion`参数

- [ ] **2.2.2.4** `_call_start_patrol(request_id, params) -> dict`
  - 调用`/mission/start_patrol`服务
  - 处理`waypoint_file`、`mode`参数

- [ ] **2.2.2.5** `_call_stop_patrol(request_id, params) -> dict`
  - 调用`/mission/stop_patrol`服务

- [ ] **2.2.2.6** `_call_emergency_stop(request_id, params) -> dict`
  - 调用`/mission/emergency_stop`服务
  - 处理`force_immediate`参数
  - 返回`cancelled_task_count`

- [ ] **2.2.2.7** `_call_pause_task(request_id, params) -> dict`
  - 调用`/mission/pause_task`服务

- [ ] **2.2.2.8** `_call_resume_task(request_id, params) -> dict`
  - 调用`/mission/resume_task`服务

- [ ] **2.2.2.9** `_call_cancel_task(request_id, params) -> dict`
  - 支持通过`task_id`或`request_id`查询
  - 从映射表获取`task_id`
  - 调用`/mission/cancel_task`服务

- [ ] **2.2.2.10** `_call_get_task_status(request_id, params) -> dict`
  - 支持通过`task_id`或`request_id`查询
  - 调用`/mission/get_task_status`服务
  - 返回`task_id`, `task_status`, `progress`, `task_type`

- [ ] **2.2.2.11** `_call_get_robot_status(request_id, params) -> dict`
  - 调用`/mission/get_robot_status`服务
  - 返回机器人状态信息

- [ ] **2.2.2.12** `_call_save_map(request_id, params) -> dict`
  - 调用地图保存服务

- [ ] **2.2.2.13** `_call_load_map(request_id, params) -> dict`
  - 调用地图加载服务

**参考代码** (按设计文档Section 3.1.3):
```python
def _call_navigate_to_pose(self, request_id: str, params: dict) -> dict:
    """导航到目标点 / Navigate to pose"""
    from bot_navigation_msgs.action import NavigateToPose
    
    goal_pose = params.get('goal_pose')
    if not goal_pose:
        return {
            'success': False,
            'message': "Missing goal_pose parameter",
            'code': ErrorCode.BAD_REQUEST
        }
    
    goal = NavigateToPose.Goal()
    # ... 构造goal消息 ...
    
    client = self.action_clients['navigate_to_pose']
    future = client.send_goal_async(goal)
    rclpy.spin_until_future_complete(self.node, future, timeout_sec=5.0)
    
    if future.result() is not None:
        goal_handle = future.result()
        if goal_handle.accepted:
            # 存储映射关系 / Store mapping
            task_id = f"nav-{goal_handle.goal_id}"
            with self.map_lock:
                self.request_task_map[request_id] = task_id
            self._save_mapping_to_file()
            
            return {
                'success': True,
                'message': "Navigation goal accepted",
                'code': ErrorCode.SUCCESS,
                'result': {'task_id': task_id}
            }
        else:
            return {
                'success': False,
                'message': "Navigation goal rejected",
                'code': ErrorCode.CONFLICT
            }
    else:
        return {
            'success': False,
            'message': "Navigation action client timeout",
            'code': ErrorCode.GATEWAY_TIMEOUT
        }
```

**验收标准**:
- ✅ 13个处理函数全部实现
- ✅ 每个函数的错误处理完善（超时、服务不可用等）
- ✅ `request_id -> task_id`映射正确存储

---

#### 2.2.3 持久化映射管理

- [ ] **2.2.3.1** 实现`_save_mapping_to_file()`
  - 按设计文档Section 3.1.3.4实现版本化JSON格式
  - 格式:
    ```json
    {
      "version": "1.0.0",
      "created_at": "2026-01-07T10:00:00",
      "format": "request_task_mapping",
      "schema_version": "1.0",
      "mappings": [
        {"request_id": "...", "task_id": "...", "timestamp": "..."}
      ]
    }
    ```
  - 支持向后兼容（读取旧版本纯数组格式）

- [ ] **2.2.3.2** 实现`_load_mapping_from_file()`
  - 支持版本化格式和旧格式
  - 文件不存在时自动创建
  - 格式错误时记录警告并跳过

**验收标准**:
- ✅ 保存/加载往返测试通过
- ✅ 旧格式兼容性测试通过

---

#### 2.2.4 重试逻辑实现

- [ ] **2.2.4.1** 实现服务调用重试包装器
  - 按设计文档Section 2.3.4实现`NavigationClientWithRetry`类
  - 仅对`503/504`错误重试
  - 重试间隔2秒，最多3次
  - 400/404错误直接返回，不重试

**参考代码** (按Section 2.3.4):
```python
class NavigationClientWithRetry:
    """带重试机制的导航客户端 / Navigation client with retry mechanism"""
    
    def submit_navigation_task_with_retry(
        self,
        request: CommandRequest,
        max_retries: int = 3
    ) -> CommandResponse:
        """提交导航任务，自动重试"""
        retries = 0
        while retries <= max_retries:
            response = self.submit_navigation_task(request)
            
            if response.code in [ErrorCode.SERVICE_UNAVAILABLE, ErrorCode.GATEWAY_TIMEOUT]:
                if retries < max_retries:
                    self.node.get_logger().warn(
                        f"Service call failed with code {response.code}, "
                        f"retrying ({retries+1}/{max_retries})..."
                    )
                    time.sleep(2.0)
                    retries += 1
                    continue
            
            # 成功或不可重试错误 / Success or non-retryable error
            return response
        
        # 重试次数用尽 / Max retries exceeded
        return CommandResponse(
            request_id=request.request_id,
            status="failed",
            message=f"Service unavailable after {max_retries} retries",
            code=ErrorCode.SERVICE_UNAVAILABLE
        )
```

**验收标准**:
- ✅ 重试逻辑测试通过
- ✅ 不可重试错误立即返回

---

### 2.3 ResponsePublisher响应发布器

**优先级**: P0 (必须) | **估时**: 0.5天

**文件**: `components/response_publisher.py`

- [ ] **2.3.1** 实现`ResponsePublisher`类
  ```python
  class ResponsePublisher:
      """
      响应发布器 - 将CommandResponse发布到/cmd/response / Response publisher - Publish CommandResponse to /cmd/response
      
      功能 / Features:
      1. 发布响应到Topic / Publish responses to topic
      2. 响应日志记录 / Response logging
      """
      
      def __init__(self, node: Node):
          self.node = node
          self.publisher = node.create_publisher(
              String,
              '/cmd/response',
              qos_profile=10
          )
      
      def publish(self, response: CommandResponse):
          """发布响应 / Publish response"""
          msg = String()
          msg.data = response.to_json()
          self.publisher.publish(msg)
          self.node.get_logger().info(
              f"Response published: request_id={response.request_id}, "
              f"status={response.status}"
          )
  ```

**验收标准**:
- ✅ 响应发布测试通过
- ✅ 日志输出正确

---

### 2.4 CommandAdapter主节点

**优先级**: P0 (必须) | **估时**: 1.5天

**文件**: `command_adapter_node.py`

#### 2.4.1 核心节点类

- [ ] **2.4.1.1** 实现`CommandAdapter`类
  ```python
  class CommandAdapter(Node):
      """
      命令适配器主节点 / Main command adapter node
      
      职责 / Responsibilities:
      1. 订阅/cmd/request，接收请求 / Subscribe to /cmd/request
      2. 验证请求，入队 / Validate and enqueue requests
      3. 后台线程处理队列 / Background thread processes queue
      4. 调用ServiceAdapter处理请求 / Call ServiceAdapter to process requests
      5. 发布响应到/cmd/response / Publish responses to /cmd/response
      """
      
      def __init__(self):
          super().__init__('command_adapter')
          
          # 加载配置 / Load configuration
          self.config = self._load_config()
          
          # 初始化组件 / Initialize components
          self.queue = RequestQueue(
              max_size=self.config['queue']['max_size'],
              dedup_window=self.config['queue']['dedup_window']
          )
          self.service_adapter = ServiceAdapter(self, self.config)
          self.response_publisher = ResponsePublisher(self)
          
          # 创建订阅器 / Create subscriber
          self.subscription = self.create_subscription(
              String,
              '/cmd/request',
              self._request_callback,
              qos_profile=10
          )
          
          # 启动处理线程 / Start processing thread
          self.processing_thread = threading.Thread(
              target=self._processing_loop,
              daemon=True
          )
          self.processing_thread.start()
          
          self.get_logger().info("CommandAdapter initialized successfully")
  ```

- [ ] **2.4.1.2** 实现`_request_callback()`
  - 接收`/cmd/request`消息
  - 解析JSON为`CommandRequest`
  - 验证请求
  - 入队
  - 发布`queued`响应

- [ ] **2.4.1.3** 实现`_processing_loop()`
  - 后台线程循环
  - 从队列取请求
  - 发布`executing`响应
  - 调用`ServiceAdapter.process_request()`
  - 根据返回结果发布`completed/failed`响应
  - 异常处理和日志

- [ ] **2.4.1.4** 实现配置加载
  - 从`command_config.yaml`加载配置
  - 支持ROS2参数覆盖
  - 默认配置fallback

**参考代码** (按Section 3.1.4):
```python
def _request_callback(self, msg: String):
    """请求回调 / Request callback"""
    try:
        # 解析JSON / Parse JSON
        request = CommandRequest.from_json(msg.data)
        
        # 验证请求 / Validate request
        valid, error_msg = request.validate()
        if not valid:
            self._publish_error_response(
                request.request_id,
                error_msg,
                ErrorCode.BAD_REQUEST
            )
            return
        
        # 入队 / Enqueue
        success, message = self.queue.enqueue(request)
        if success:
            # 发布queued响应 / Publish queued response
            response = CommandResponse(
                request_id=request.request_id,
                status="queued",
                message="Request queued successfully",
                code=ErrorCode.SUCCESS
            )
            self.response_publisher.publish(response)
        else:
            # 入队失败（重复或队列满）/ Enqueue failed (duplicate or queue full)
            self._publish_error_response(
                request.request_id,
                message,
                ErrorCode.CONFLICT
            )
    
    except json.JSONDecodeError as e:
        self.get_logger().error(f"Failed to parse request JSON: {e}")
    except Exception as e:
        self.get_logger().error(f"Request callback error: {e}")

def _processing_loop(self):
    """处理循环 / Processing loop"""
    while rclpy.ok():
        try:
            # 从队列取请求 / Dequeue request
            request = self.queue.dequeue(timeout=1.0)
            if request is None:
                continue
            
            # 发布executing响应 / Publish executing response
            executing_response = CommandResponse(
                request_id=request.request_id,
                status="executing",
                message="Processing request",
                code=ErrorCode.SUCCESS
            )
            self.response_publisher.publish(executing_response)
            
            # 调用服务适配器 / Call service adapter
            result = self.service_adapter.process_request(request)
            
            # 发布完成响应 / Publish completion response
            final_status = "completed" if result['success'] else "failed"
            final_response = CommandResponse(
                request_id=request.request_id,
                status=final_status,
                message=result['message'],
                code=result['code'],
                result=result.get('result', {})
            )
            self.response_publisher.publish(final_response)
        
        except Exception as e:
            self.get_logger().error(f"Processing loop error: {e}")
            if request:
                self._publish_error_response(
                    request.request_id,
                    f"Internal error: {str(e)}",
                    ErrorCode.INTERNAL_ERROR
                )
```

**验收标准**:
- ✅ 节点启动无错误
- ✅ 订阅`/cmd/request`成功
- ✅ 处理线程正常运行
- ✅ 请求-响应流程完整

---

#### 2.4.2 Launch文件

**文件**: `launch/cmd_adapter.launch.py`

- [ ] **2.4.2.1** 创建启动文件
  ```python
  from launch import LaunchDescription
  from launch_ros.actions import Node
  from launch.actions import DeclareLaunchArgument
  from launch.substitutions import LaunchConfiguration
  from ament_index_python.packages import get_package_share_directory
  import os
  
  def generate_launch_description():
      pkg_dir = get_package_share_directory('bot_cmd_interface')
      config_file = os.path.join(pkg_dir, 'config', 'command_config.yaml')
      location_map = os.path.join(pkg_dir, 'config', 'location_map.yaml')
      
      return LaunchDescription([
          DeclareLaunchArgument(
              'config_file',
              default_value=config_file,
              description='Path to command config file'
          ),
          DeclareLaunchArgument(
              'location_map',
              default_value=location_map,
              description='Path to location map file'
          ),
          Node(
              package='bot_cmd_interface',
              executable='command_adapter',
              name='command_adapter',
              output='screen',
              parameters=[
                  {'config_file': LaunchConfiguration('config_file')},
                  {'location_map': LaunchConfiguration('location_map')},
              ]
          )
      ])
  ```

**验收标准**:
- ✅ `ros2 launch bot_cmd_interface cmd_adapter.launch.py`启动成功
- ✅ 节点正常运行

---

### 2.5 Phase 2 验收总结

- [ ] **2.5.1** 集成测试 - 完整请求流程
  ```bash
  # Terminal 1: 启动CommandAdapter
  ros2 launch bot_cmd_interface cmd_adapter.launch.py
  
  # Terminal 2: 发送测试请求
  ros2 topic pub --once /cmd/request std_msgs/String \
    '{"data": "{\"request_id\": \"test-001\", \"action\": \"get_robot_status\", \"params\": {}}"}'
  
  # Terminal 3: 监听响应
  ros2 topic echo /cmd/response
  ```

- [ ] **2.5.2** 单元测试总报告
  ```bash
  pytest src/bot_cmd_interface/test/ -v --cov=bot_cmd_interface
  ```

**Phase 2 完成标志**:
- ✅ CommandAdapter节点正常运行
- ✅ 请求-响应完整流程测试通过
- ✅ 所有组件单元测试通过
- ✅ 测试覆盖率 > 80%

---

## Phase 3: 测试与验证 (2-3天)

### 3.1 集成测试套件

**优先级**: P0 (必须) | **估时**: 1.5天

**文件**: `test/test_integration.py`

- [ ] **3.1.1** 导航功能测试
  - 测试`navigate_to_pose`完整流程
  - 测试`navigate_to_location`（地点映射）
  - 测试导航超时处理

- [ ] **3.1.2** 任务管理测试
  - 测试`start_exploration` + `get_task_status`
  - 测试`start_patrol` + `pause_task` + `resume_task`
  - 测试`cancel_task`
  - 测试`emergency_stop`抢占

- [ ] **3.1.3** 查询功能测试
  - 测试`get_task_status`（通过task_id和request_id）
  - 测试`get_robot_status`
  - 测试映射表持久化

- [ ] **3.1.4** 异常场景测试
  - 测试非法JSON格式
  - 测试缺少必需参数
  - 测试服务不可用场景
  - 测试重复请求去重
  - 测试队列满场景

**测试框架**:
```python
class TestCommandAdapterIntegration:
    """命令适配器集成测试 / Command adapter integration tests"""
    
    @classmethod
    def setup_class(cls):
        """启动测试环境 / Setup test environment"""
        # 启动CommandAdapter节点
        # 创建测试客户端
        pass
    
    def test_navigate_to_pose_flow(self):
        """测试导航完整流程 / Test navigation flow"""
        # 1. 构造请求
        request = create_navigate_request(1.0, 2.0)
        
        # 2. 发送请求
        self.publish_request(request)
        
        # 3. 验证queued响应
        response = self.wait_for_response(request.request_id, timeout=5.0)
        assert response.status == "queued"
        
        # 4. 验证executing响应
        response = self.wait_for_response(request.request_id, timeout=5.0)
        assert response.status == "executing"
        
        # 5. 验证completed响应
        response = self.wait_for_response(request.request_id, timeout=10.0)
        assert response.status in ["completed", "failed"]
```

**验收标准**:
- ✅ 所有测试场景通过
- ✅ 集成测试覆盖核心流程

---

### 3.2 性能测试

**优先级**: P1 (高) | **估时**: 0.5天

**文件**: `test/test_performance.py`

- [ ] **3.2.1** 吞吐量测试
  - 100个请求连续发送，测试处理时间
  - 验证队列无阻塞

- [ ] **3.2.2** 并发测试
  - 10个线程同时发送请求
  - 验证线程安全

- [ ] **3.2.3** 内存泄漏测试
  - 长时间运行（1000个请求）
  - 监控内存使用

**验收标准**:
- ✅ 吞吐量 > 10 requests/s
- ✅ 无内存泄漏

---

### 3.3 模拟终端实现

**优先级**: P2 (中) | **估时**: 1天

**目的**: 提供一个参考实现，展示如何使用SDK集成终端

**文件**: `examples/mock_terminal.py`

- [ ] **3.3.1** 实现CLI模拟终端
  ```python
  class MockTerminal:
      """模拟终端 - 演示SDK使用 / Mock terminal - Demonstrate SDK usage"""
      
      def __init__(self):
          rclpy.init()
          self.node = rclpy.create_node('mock_terminal')
          
          # 创建发布器 / Create publisher
          self.publisher = self.node.create_publisher(
              String,
              '/cmd/request',
              qos_profile=10
          )
          
          # 订阅响应 / Subscribe to responses
          self.subscription = self.node.create_subscription(
              String,
              '/cmd/response',
              self._response_callback,
              qos_profile=10
          )
          
          self.pending_requests = {}  # {request_id: callback}
      
      def send_request(self, request: CommandRequest, callback=None):
          """发送请求 / Send request"""
          msg = String()
          msg.data = request.to_json()
          self.publisher.publish(msg)
          
          if callback:
              self.pending_requests[request.request_id] = callback
          
          print(f"[Sent] {request.action} | request_id={request.request_id}")
      
      def _response_callback(self, msg: String):
          """响应回调 / Response callback"""
          response = CommandResponse.from_json(msg.data)
          
          # 过滤自己的请求 / Filter own requests
          if response.request_id in self.pending_requests:
              print(f"[Received] {response.status} | {response.message}")
              
              if response.status in ["completed", "failed", "cancelled"]:
                  callback = self.pending_requests.pop(response.request_id)
                  if callback:
                      callback(response)
  ```

- [ ] **3.3.2** 实现交互式CLI
  ```python
  def interactive_cli():
      """交互式命令行 / Interactive CLI"""
      terminal = MockTerminal()
      
      print("=== LeKiwi Robot Mock Terminal ===")
      print("Commands:")
      print("  1. navigate <x> <y>")
      print("  2. goto <location>")
      print("  3. explore [map_name]")
      print("  4. patrol <waypoint_file>")
      print("  5. status <task_id>")
      print("  6. cancel <task_id>")
      print("  7. emergency")
      print("  8. quit")
      
      while True:
          cmd = input("\n> ").strip()
          
          if cmd == "quit":
              break
          elif cmd.startswith("navigate"):
              _, x, y = cmd.split()
              request = create_navigate_request(float(x), float(y))
              terminal.send_request(request)
          # ... 其他命令处理 ...
  ```

**验收标准**:
- ✅ CLI可正常运行
- ✅ 能发送所有类型请求
- ✅ 响应过滤正确

---

## Phase 4: 文档与完善 (1-2天)

### 4.1 README文档

**优先级**: P1 (高) | **估时**: 0.5天

**文件**: `README.md`

- [ ] **4.1.1** 编写完整README
  - 项目简介
  - 快速开始
  - SDK使用示例
  - 配置说明
  - 故障排查

**内容结构**:
```markdown
# bot_cmd_interface - LeKiwi统一命令接口

## 功能特性
- ✅ 统一的请求/响应协议
- ✅ 基于请求ID的生命周期管理
- ✅ 松耦合的终端集成
- ✅ JSON封装，简化消息定义

## 快速开始

### 安装
```bash
cd ~/lododo_bot
colcon build --packages-select bot_cmd_interface
source install/setup.bash
```

### 启动CommandAdapter
```bash
ros2 launch bot_cmd_interface cmd_adapter.launch.py
```

### 使用SDK发送请求
```python
from bot_cmd_interface.sdk import create_navigate_request
import rclpy
from std_msgs.msg import String

# 初始化ROS2
rclpy.init()
node = rclpy.create_node('my_terminal')

# 创建发布器
publisher = node.create_publisher(String, '/cmd/request', 10)

# 创建请求
request = create_navigate_request(x=1.5, y=2.0)

# 发布请求
msg = String()
msg.data = request.to_json()
publisher.publish(msg)
```

## API参考
...
```

**验收标准**:
- ✅ README完整清晰
- ✅ 示例代码可运行

---

### 4.2 API文档

**优先级**: P2 (中) | **估时**: 0.5天

**文件**: `docs/API.md`

- [ ] **4.2.1** 生成API文档
  - SDK类API说明
  - Topic接口说明
  - 配置文件说明
  - 错误码参考

**使用工具**: Sphinx或pdoc生成Python文档

**验收标准**:
- ✅ API文档完整
- ✅ 示例代码齐全

---

### 4.3 终端集成指南

**优先级**: P1 (高) | **估时**: 0.5天

**文件**: `docs/TERMINAL_INTEGRATION.md`

- [ ] **4.3.1** 编写终端集成指南
  - 如何依赖SDK
  - 如何发送请求
  - 如何过滤响应
  - 最佳实践

**内容示例**:
```markdown
# 终端集成指南

## 步骤1: 添加依赖
在你的包的`setup.py`中添加:
```python
install_requires=[
    'bot_cmd_interface',
]
```

## 步骤2: 导入SDK
```python
from bot_cmd_interface.sdk import (
    CommandRequest,
    CommandResponse,
    create_navigate_request,
    ActionType
)
```

## 步骤3: 创建发布器和订阅器
...

## 最佳实践
1. 使用便捷构造函数而非手动构造JSON
2. 订阅响应时根据request_id过滤
3. 处理所有响应状态（queued/executing/completed/failed）
4. 实现超时机制
```

**验收标准**:
- ✅ 集成指南清晰易懂
- ✅ 覆盖所有集成场景

---

### 4.4 最终验收与发布

- [ ] **4.4.1** 代码审查清单
  - [ ] 所有代码有中英文双语注释
  - [ ] 所有日志使用英文
  - [ ] 遵循ROS2命名规范
  - [ ] 无硬编码路径
  - [ ] 所有TODO注释已清理

- [ ] **4.4.2** 测试总报告
  ```bash
  pytest src/bot_cmd_interface/test/ -v --cov=bot_cmd_interface --cov-report=html
  ```
  - 单元测试覆盖率 > 85%
  - 集成测试全部通过
  - 性能测试符合预期

- [ ] **4.4.3** 文档检查
  - [ ] ARCHITECTURE.md v0.3.0最新
  - [ ] README.md完整
  - [ ] API.md完整
  - [ ] TERMINAL_INTEGRATION.md完整
  - [ ] IMPLEMENTATION_TODO.md更新状态

- [ ] **4.4.4** 发布准备
  - [ ] 更新`setup.py`版本号为1.0.0
  - [ ] 更新`package.xml`版本号
  - [ ] 提交代码到Git
  - [ ] 创建Release Tag: `v1.0.0`

**最终验收标准**:
- ✅ 所有Phase 1-4任务完成
- ✅ 测试覆盖率 > 85%
- ✅ 文档完整
- ✅ 无已知Bug

---

## 附录: 任务清单总结

### 关键里程碑

| 里程碑 | 完成标志 | 预计日期 |
|--------|---------|---------|
| M1: SDK完成 | Phase 1验收通过 | Day 4 |
| M2: 核心组件完成 | Phase 2验收通过 | Day 9 |
| M3: 测试验证完成 | Phase 3验收通过 | Day 12 |
| M4: 文档完善与发布 | Phase 4验收通过 | Day 14 |

### 任务统计

- **总任务数**: 78个
- **P0 (必须)**: 48个
- **P1 (高优先级)**: 18个
- **P2 (中优先级)**: 12个

### 依赖关系

```
Phase 1 (SDK) ──> Phase 2 (核心组件) ──> Phase 3 (测试) ──> Phase 4 (文档)
     │                 │                      │
     │                 └──> Phase 3可并行     │
     │                                        │
     └────────────────────────────────────────┘
                  Phase 4可部分并行
```

### 风险评估

| 风险项 | 概率 | 影响 | 缓解措施 |
|--------|------|------|---------|
| ROS2服务不可用 | 中 | 高 | 实现Mock服务用于独立测试 |
| 性能不达标 | 低 | 中 | 预留优化时间，使用性能分析工具 |
| 集成测试失败 | 中 | 高 | 分阶段测试，及早发现问题 |
| 文档编写延期 | 低 | 低 | 在开发过程中同步编写 |

---

**文档状态**: 📋 待开始  
**下一步**: 开始Phase 1.1 - 项目结构搭建  
**预计完成日期**: 2026-01-21

