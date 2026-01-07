# bot_cmd_interface - LeKiwi统一命令接口

**Version**: 1.0.0  
**Status**: Phase 1 Complete (SDK Module) ✅

LeKiwi机器人的统一命令接口层，提供基于ROS2 Topic的异步请求/响应协议，实现终端与后端服务的松耦合集成。

## 功能特性

- ✅ **统一协议标准** - 基于JSON的请求/响应消息格式
- ✅ **请求ID驱动** - 完整的请求生命周期管理
- ✅ **松耦合架构** - 终端与后端通过标准协议交互
- ✅ **SDK支持** - 完整的Python SDK，简化终端集成
- ✅ **13种动作类型** - 导航、任务、查询、控制、地图操作
- ✅ **错误码标准化** - 基于HTTP状态码的统一错误体系
- ✅ **配置驱动** - YAML配置文件，灵活配置超时、队列等参数

## 项目结构

```
bot_cmd_interface/
├── bot_cmd_interface/
│   ├── sdk/                          # ✅ SDK模块（Phase 1完成）
│   │   ├── __init__.py
│   │   ├── action_types.py           # 动作类型定义
│   │   ├── message.py                # 请求/响应消息类
│   │   └── validators.py             # 验证器
│   ├── components/                   # 🚧 核心组件（Phase 2）
│   │   ├── __init__.py
│   │   ├── request_queue.py          # 待实现
│   │   ├── service_adapter.py        # 待实现
│   │   └── response_publisher.py     # 待实现
│   ├── utils/                        # ✅ 工具模块（Phase 1完成）
│   │   ├── __init__.py
│   │   └── config_loader.py          # 配置加载器
│   └── command_adapter_node.py       # 🚧 主节点（Phase 2）
├── config/                           # ✅ 配置文件（Phase 1完成）
│   ├── command_config.yaml           # 命令适配器配置
│   └── location_map.yaml             # 地点映射配置
├── launch/                           # 🚧 启动文件（Phase 2）
│   └── cmd_adapter.launch.py         # 待实现
├── test/                             # ✅ 测试（Phase 1完成）
│   └── test_sdk.py                   # SDK单元测试
├── examples/                         # 🚧 示例（Phase 3）
├── ARCHITECTURE.md                   # ✅ 架构设计文档 v0.3.0
├── IMPLEMENTATION_TODO.md            # ✅ 实现计划
├── setup.py                          # ✅ 包配置
└── README.md                         # 本文件
```

## 快速开始

### 安装

```bash
cd ~/lododo_bot
colcon build --packages-select bot_cmd_interface --symlink-install
source install/setup.bash
```

### 使用SDK

#### 1. 导入SDK

```python
from bot_cmd_interface.sdk import (
    CommandRequest,
    CommandResponse,
    ActionType,
    ResponseStatus,
    ErrorCode,
    create_navigate_request,
    create_navigate_to_location_request,
    create_patrol_request,
    create_exploration_request,
    create_emergency_stop_request,
    create_get_status_request,
    create_cancel_task_request,
    validate_request,
    validate_response,
)
```

#### 2. 创建导航请求

```python
# 方法1: 使用便捷构造函数（推荐）
request = create_navigate_request(x=1.5, y=2.0, yaw=0.785)

# 方法2: 手动构造
request = CommandRequest(
    action=ActionType.NAVIGATE_TO_POSE,
    params={
        "goal_pose": {
            "position": {"x": 1.5, "y": 2.0},
            "orientation": {"yaw": 0.785}
        }
    }
)

# 序列化为JSON
json_str = request.to_json()
print(json_str)
```

#### 3. 解析响应消息

```python
# 从JSON解析响应
response = CommandResponse.from_json(json_str)

# 检查响应状态
if response.is_success():
    print(f"任务成功完成: {response.message}")
    print(f"结果: {response.result}")
elif response.is_final():
    print(f"任务失败: {response.message}, 错误码: {response.code}")
else:
    print(f"任务进行中: {response.status}")
```

#### 4. 使用地点映射

```python
from bot_cmd_interface.utils import LocationMapLoader

# 加载地点映射
loader = LocationMapLoader("config/location_map.yaml")

# 获取地点坐标
pose = loader.get_pose("厨房")
print(pose)  # {'x': 2.5, 'y': 3.0, 'yaw': 1.57}

# 列出所有地点
locations = loader.list_locations()
print(locations)  # ['厨房', '客厅', '卧室', '阳台', '充电桩']

# 创建导航到地点请求
request = create_navigate_to_location_request("厨房")
```

## SDK API参考

### ActionType类

13种动作类型常量：

| 动作类型 | 说明 | 分类 |
|---------|------|------|
| `NAVIGATE_TO_POSE` | 导航到目标点 (x, y, yaw) | 导航 |
| `NAVIGATE_TO_LOCATION` | 导航到命名地点 | 导航 |
| `START_EXPLORATION` | 开始自主探索建图 | 任务 |
| `START_PATROL` | 开始巡航任务 | 任务 |
| `STOP_PATROL` | 停止巡航任务 | 任务 |
| `PAUSE_TASK` | 暂停当前任务 | 任务 |
| `RESUME_TASK` | 恢复暂停的任务 | 任务 |
| `CANCEL_TASK` | 取消任务 | 任务 |
| `GET_TASK_STATUS` | 查询任务状态 | 查询 |
| `GET_ROBOT_STATUS` | 查询机器人状态 | 查询 |
| `EMERGENCY_STOP` | 紧急停止 | 控制 |
| `SAVE_MAP` | 保存地图 | 地图 |
| `LOAD_MAP` | 加载地图 | 地图 |

### ErrorCode类

标准化错误码：

| 错误码 | 说明 | 可重试 |
|--------|------|-------|
| 0 | 成功 | N/A |
| 400 | 请求格式错误或缺少必需参数 | ❌ |
| 404 | 资源未找到 | ❌ |
| 409 | 状态冲突（重复请求、队列满） | ❌ |
| 499 | 客户端关闭连接 | ❌ |
| 500 | 服务内部错误 | ❌ |
| 503 | 服务不可用 | ✅ |
| 504 | 网关超时 | ✅ |

### ResponseStatus类

响应状态常量：

- `QUEUED` - 请求已入队
- `EXECUTING` - 请求正在执行
- `COMPLETED` - 请求已完成
- `FAILED` - 请求失败
- `CANCELLED` - 请求已取消

### 便捷构造函数

| 函数 | 说明 |
|------|------|
| `create_navigate_request(x, y, yaw=None)` | 创建导航到坐标请求 |
| `create_navigate_to_location_request(location)` | 创建导航到地点请求 |
| `create_patrol_request(waypoint_file, mode='loop')` | 创建巡航请求 |
| `create_exploration_request(map_name=None)` | 创建探索请求 |
| `create_emergency_stop_request()` | 创建紧急停止请求 |
| `create_get_status_request(task_id=None, request_id=None)` | 创建状态查询请求 |
| `create_cancel_task_request(task_id=None, request_id=None)` | 创建取消任务请求 |

## 配置文件

### command_config.yaml

包含队列、超时、持久化、日志、服务等配置：

```yaml
queue:
  max_size: 100
  dedup_window: 5.0

timeout:
  default: 300.0
  navigation: 300.0
  exploration: 3600.0
  
persistence:
  enabled: true
  mapping_file: ~/.bot_cmd_interface/request_task_map.json
```

### location_map.yaml

地点映射配置，包含5个示例地点：

```yaml
locations:
  - name: "厨房"
    x: 2.5
    y: 3.0
    yaw: 1.57
    description: "Kitchen area"
    tags: ["room", "indoor"]
```

## 实施进度

### ✅ Phase 1: 基础框架与SDK模块（已完成）

- [x] 1.1 项目结构搭建
  - [x] 目录结构创建
  - [x] setup.py配置
  - [x] package.xml配置
  - [x] 编译测试通过

- [x] 1.2 SDK模块核心类实现
  - [x] ActionType常量定义（13个动作）
  - [x] CommandRequest类（构造、序列化、反序列化、验证）
  - [x] CommandResponse类（序列化、反序列化、状态判断）
  - [x] ErrorCode枚举类（8个错误码）
  - [x] 6个便捷构造函数
  - [x] SDK验证器

- [x] 1.3 配置文件管理
  - [x] command_config.yaml（完整配置）
  - [x] location_map.yaml（5个示例地点）
  - [x] LocationMapLoader类
  - [x] load_command_config函数

- [x] 1.4 Phase 1 验收
  - [x] 编译无错误
  - [x] SDK导入测试通过
  - [x] 功能测试通过

**测试结果**:
```
✅ SDK import OK
Request: CommandRequest(request_id='41d9e782-0567-4945-91ca-97afebe8e204', 
         action='navigate_to_pose', priority=3)
JSON: {"header": {...}, "body": {"action": "navigate_to_pose", ...}}
```

### 🚧 Phase 2: 核心组件（进行中）

- [ ] 2.1 RequestQueue请求队列
- [ ] 2.2 ServiceAdapter服务适配器
- [ ] 2.3 ResponsePublisher响应发布器
- [ ] 2.4 CommandAdapter主节点

### 📅 Phase 3: 测试验证（待开始）

- [ ] 3.1 集成测试套件
- [ ] 3.2 性能测试
- [ ] 3.3 模拟终端实现

### 📅 Phase 4: 文档完善（待开始）

- [ ] 4.1 README文档
- [ ] 4.2 API文档
- [ ] 4.3 终端集成指南

## 开发指南

### 编码规范

1. **中英文双语注释** - 所有代码注释采用中文+英文对照
   ```python
   # 初始化请求队列 / Initialize request queue
   self.request_queue = RequestQueue(max_size=100)
   ```

2. **统一英文日志** - 所有ROS2日志使用英文
   ```python
   self.get_logger().info("Request queued successfully")
   ```

3. **类型注解** - 使用Python类型注解提高代码可读性
   ```python
   def validate_request(request: CommandRequest) -> Tuple[bool, str]:
       pass
   ```

### 测试

```bash
# 安装pytest（如果尚未安装）
pip3 install pytest pytest-cov

# 运行单元测试
pytest src/bot_cmd_interface/test/test_sdk.py -v

# 运行测试并生成覆盖率报告
pytest src/bot_cmd_interface/test/ --cov=bot_cmd_interface --cov-report=html
```

### 构建

```bash
cd ~/lododo_bot
colcon build --packages-select bot_cmd_interface --symlink-install
source install/setup.bash
```

## 技术栈

- **ROS2 Humble** - 机器人操作系统
- **Python 3.10** - 主要开发语言
- **std_msgs/String** - 消息传输（JSON封装）
- **YAML** - 配置文件格式
- **pytest** - 单元测试框架

## 参考文档

- [ARCHITECTURE.md](ARCHITECTURE.md) - 完整架构设计文档 v0.3.0
- [IMPLEMENTATION_TODO.md](IMPLEMENTATION_TODO.md) - 详细实现计划（78个任务）

## 许可证

MIT License

## 维护者

LeKiwi Robot Team

---

**最后更新**: 2026-01-07  
**Phase 1状态**: ✅ 完成  
**下一步**: 开始Phase 2 - 核心组件实现
