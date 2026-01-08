# 终端集成指南 - bot_cmd_interface

**版本**: v1.0.0  
**最后更新**: 2026-01-08

本指南介绍如何将你的终端（语音、Web、CLI等）集成到 LeKiwi 统一命令接口。

---

## 目录

1. [概述](#概述)
2. [集成步骤](#集成步骤)
3. [SDK使用](#sdk使用)
4. [响应处理](#响应处理)
5. [最佳实践](#最佳实践)
6. [示例代码](#示例代码)
7. [故障排查](#故障排查)

---

## 概述

### 什么是终端集成？

终端集成是指将你的用户界面（UI）或输入系统连接到 LeKiwi 机器人的统一命令接口。通过集成，你的终端可以：

- 发送导航、探索、巡逻等命令
- 接收实时状态更新
- 查询机器人和任务状态
- 控制任务（暂停、恢复、取消）

### 架构模型

```
你的终端
    ↓ (发布 CommandRequest JSON)
  /cmd/request
    ↓
CommandAdapter (黑盒)
    ↓
  /cmd/response
    ↓ (订阅 CommandResponse JSON)
你的终端
```

**关键点**:
- 终端与 CommandAdapter 之间完全解耦
- 使用标准 ROS2 Topic 通信
- JSON 格式消息，易于调试和扩展
- SDK 提供便捷的请求构造和响应解析

---

## 集成步骤

### 步骤 1: 添加依赖

#### Python 包依赖

在你的 `setup.py` 中添加：

```python
from setuptools import setup

setup(
    name='my_terminal',
    version='1.0.0',
    packages=['my_terminal'],
    install_requires=[
        'bot_cmd_interface',  # 添加此依赖
        'rclpy',
        'std_msgs',
    ],
    # ...
)
```

#### ROS2 包依赖

在你的 `package.xml` 中添加：

```xml
<?xml version="1.0"?>
<package format="3">
  <name>my_terminal</name>
  <!-- ... -->
  
  <depend>rclpy</depend>
  <depend>std_msgs</depend>
  <depend>bot_cmd_interface</depend>  <!-- 添加此依赖 -->
  
  <!-- ... -->
</package>
```

### 步骤 2: 导入 SDK

```python
from bot_cmd_interface.sdk import (
    CommandRequest,
    CommandResponse,
    ActionType,
    create_navigate_request,
    create_exploration_request,
    create_patrol_request,
    create_emergency_stop_request,
    create_robot_status_request,
    create_task_status_request,
)
```

### 步骤 3: 创建终端节点

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from bot_cmd_interface.sdk import CommandRequest, CommandResponse

class MyTerminalNode(Node):
    def __init__(self):
        super().__init__('my_terminal')
        
        # 创建发布器（发送请求）
        self.request_publisher = self.create_publisher(
            String,
            '/cmd/request',
            10
        )
        
        # 创建订阅器（接收响应）
        self.response_subscription = self.create_subscription(
            String,
            '/cmd/response',
            self._response_callback,
            10
        )
        
        # 待处理请求字典
        self.pending_requests = {}
        
        self.get_logger().info('Terminal node initialized')
    
    def send_request(self, request: CommandRequest, callback=None):
        """发送请求"""
        # 序列化为 JSON
        msg = String()
        msg.data = request.to_json()
        
        # 发布到 Topic
        self.request_publisher.publish(msg)
        
        # 记录待处理请求
        if callback:
            self.pending_requests[request.request_id] = callback
        
        self.get_logger().info(f'Sent request: {request.request_id}')
    
    def _response_callback(self, msg: String):
        """响应回调"""
        # 解析 JSON
        response = CommandResponse.from_json(msg.data)
        
        # 过滤自己的请求
        if response.request_id in self.pending_requests:
            # 调用回调函数
            callback = self.pending_requests[response.request_id]
            callback(response)
            
            # 最终状态时清理
            if response.is_final():
                del self.pending_requests[response.request_id]
```

### 步骤 4: 实现请求发送

```python
def navigate_to_position(self, x: float, y: float):
    """导航到指定位置"""
    from bot_cmd_interface.sdk import create_navigate_request
    
    # 创建请求
    request = create_navigate_request(x, y)
    
    # 定义响应回调
    def on_response(response: CommandResponse):
        if response.status == 'queued':
            self.get_logger().info('Navigation request queued')
        elif response.status == 'executing':
            self.get_logger().info('Navigation in progress...')
        elif response.status == 'completed':
            self.get_logger().info('Navigation completed!')
        elif response.status == 'failed':
            self.get_logger().error(f'Navigation failed: {response.message}')
    
    # 发送请求
    self.send_request(request, on_response)
```

### 步骤 5: 构建和运行

```bash
# 构建你的包
cd ~/lododo_bot
colcon build --packages-select my_terminal --symlink-install
source install/setup.bash

# 运行你的终端节点
ros2 run my_terminal my_terminal_node
```

---

## SDK使用

### 创建请求

#### 方法 1: 使用便捷构造函数（推荐）

```python
from bot_cmd_interface.sdk import (
    create_navigate_request,
    create_exploration_request,
    create_patrol_request,
)

# 导航到坐标
request = create_navigate_request(x=1.5, y=2.0, yaw=0.785)

# 开始探索
request = create_exploration_request(
    map_name='office_floor1',
    save_on_completion=True
)

# 开始巡逻
request = create_patrol_request(
    waypoint_file='/path/to/waypoints.yaml',
    mode='loop'
)
```

#### 方法 2: 手动构造

```python
from bot_cmd_interface.sdk import CommandRequest, ActionType

request = CommandRequest(
    action=ActionType.NAVIGATE_TO_POSE,
    params={
        "goal_pose": {
            "position": {"x": 1.5, "y": 2.0, "z": 0.0},
            "orientation": {"yaw": 0.785}
        }
    },
    source="my_terminal"  # 可选：标识请求来源
)
```

### 发送请求

```python
# 序列化为 JSON
msg = String()
msg.data = request.to_json()

# 发布到 /cmd/request
self.request_publisher.publish(msg)

self.get_logger().info(f'Sent request: {request.request_id}')
```

### 解析响应

```python
def _response_callback(self, msg: String):
    # 从 JSON 解析响应
    response = CommandResponse.from_json(msg.data)
    
    # 检查是否是你的请求
    if response.request_id in self.pending_requests:
        # 处理响应
        self._handle_response(response)

def _handle_response(self, response: CommandResponse):
    if response.status == 'queued':
        print(f"⟳ Request queued: {response.request_id}")
    
    elif response.status == 'executing':
        print(f"⟳ Executing: {response.message}")
        # 提取进度（如果有）
        if response.data and 'progress' in response.data:
            progress = response.data['progress']
            print(f"   Progress: {progress*100:.1f}%")
    
    elif response.status == 'completed':
        print(f"✓ Completed: {response.message}")
        # 提取结果（如果有）
        if response.data and 'result' in response.data:
            result = response.data['result']
            print(f"   Result: {result}")
    
    elif response.status == 'failed':
        print(f"✗ Failed: {response.message}")
        # 提取错误信息（如果有）
        if response.data and 'error' in response.data:
            error = response.data['error']
            print(f"   Error: {error}")
    
    elif response.status == 'cancelled':
        print(f"⊗ Cancelled: {response.message}")
```

---

## 响应处理

### 响应状态生命周期

```
queued → executing → completed
                   ↓
                  failed
                   ↓
                cancelled
```

### 状态说明

| 状态 | 说明 | 是否最终状态 |
|------|------|-------------|
| `queued` | 请求已入队，等待处理 | ❌ |
| `executing` | 正在执行 | ❌ |
| `completed` | 成功完成 | ✅ |
| `failed` | 执行失败 | ✅ |
| `cancelled` | 已取消 | ✅ |

### 响应数据字段

响应的 `data` 字段可能包含：

#### `queued` 状态
```python
response.data = {}  # 通常为空
```

#### `executing` 状态
```python
response.data = {
    "progress": 0.65,        # 进度 (0.0-1.0)
    "eta_seconds": 12.5,     # 预计剩余时间
    "current_state": "..."   # 当前状态描述
}
```

#### `completed` 状态
```python
response.data = {
    "result": "success",
    "distance_traveled": 5.2,
    "duration_seconds": 18.5,
    # ... 其他结果数据
}
```

#### `failed` 状态
```python
response.data = {
    "error": "Obstacle detected",
    "error_code": "COLLISION_AVOIDANCE",
    "retry_possible": True
}
```

### 过滤响应

**重要**: 你的终端会收到所有终端的响应。你必须过滤出自己的请求。

#### 方法 1: 使用待处理请求字典

```python
class MyTerminalNode(Node):
    def __init__(self):
        super().__init__('my_terminal')
        self.pending_requests = {}  # {request_id: callback}
    
    def send_request(self, request, callback):
        # 记录请求
        self.pending_requests[request.request_id] = callback
        # 发布请求...
    
    def _response_callback(self, msg):
        response = CommandResponse.from_json(msg.data)
        
        # 仅处理自己的请求
        if response.request_id in self.pending_requests:
            callback = self.pending_requests[response.request_id]
            callback(response)
            
            # 最终状态时清理
            if response.is_final():
                del self.pending_requests[response.request_id]
```

#### 方法 2: 使用 source 字段

```python
TERMINAL_SOURCE = "my_terminal"

# 发送请求时设置 source
request = CommandRequest(
    action=ActionType.NAVIGATE_TO_POSE,
    params={...},
    source=TERMINAL_SOURCE  # 设置你的终端标识
)

# 响应回调中检查 source（需要在响应中回传 source）
def _response_callback(self, msg):
    response = CommandResponse.from_json(msg.data)
    
    # 注意：当前实现的响应中没有 source 字段
    # 建议使用方法 1（request_id 过滤）
```

---

## 最佳实践

### 1. 使用便捷构造函数

✅ **推荐**:
```python
request = create_navigate_request(1.5, 2.0)
```

❌ **不推荐**:
```python
request = CommandRequest(
    action=ActionType.NAVIGATE_TO_POSE,
    params={"goal_pose": {"position": {"x": 1.5, "y": 2.0, "z": 0.0}, ...}}
)
```

### 2. 实现超时机制

```python
import time

class MyTerminalNode(Node):
    def __init__(self):
        super().__init__('my_terminal')
        self.pending_requests = {}  # {request_id: (callback, timestamp)}
        
        # 创建定时器检查超时
        self.create_timer(1.0, self._check_timeouts)
    
    def send_request(self, request, callback, timeout=30.0):
        self.pending_requests[request.request_id] = {
            'callback': callback,
            'timestamp': time.time(),
            'timeout': timeout
        }
        # 发布请求...
    
    def _check_timeouts(self):
        current_time = time.time()
        timed_out = []
        
        for request_id, info in self.pending_requests.items():
            if current_time - info['timestamp'] > info['timeout']:
                timed_out.append(request_id)
        
        for request_id in timed_out:
            info = self.pending_requests[request_id]
            # 调用回调，传入超时错误
            error_response = CommandResponse(
                request_id=request_id,
                status='failed',
                message='Request timeout'
            )
            info['callback'](error_response)
            del self.pending_requests[request_id]
```

### 3. 处理所有响应状态

```python
def on_response(response: CommandResponse):
    if response.status == 'queued':
        # 显示"正在排队"
        pass
    elif response.status == 'executing':
        # 更新进度条
        pass
    elif response.status == 'completed':
        # 显示成功消息
        pass
    elif response.status == 'failed':
        # 显示错误，可能提供重试选项
        pass
    elif response.status == 'cancelled':
        # 显示取消消息
        pass
```

### 4. 使用日志记录

```python
self.get_logger().info(f'Sent request: {request.request_id}')
self.get_logger().debug(f'Request params: {request.params}')
self.get_logger().warn(f'Request timeout: {request.request_id}')
self.get_logger().error(f'Request failed: {response.message}')
```

### 5. 验证请求

```python
from bot_cmd_interface.sdk import validate_request

# 发送前验证
is_valid, error_msg = validate_request(request.to_dict())
if not is_valid:
    self.get_logger().error(f'Invalid request: {error_msg}')
    return

# 发送请求...
```

### 6. 处理重复请求

CommandAdapter 有5秒的去重窗口。如果你需要短时间内发送相同的请求：

```python
# 方法 1: 修改 params
request1 = create_navigate_request(1.0, 2.0)
time.sleep(1)
request2 = create_navigate_request(1.0, 2.0)
request2.params['_unique'] = str(time.time())  # 添加唯一标识

# 方法 2: 等待 5 秒
request1 = create_navigate_request(1.0, 2.0)
# 发送...
time.sleep(5)  # 等待去重窗口过期
request2 = create_navigate_request(1.0, 2.0)
# 发送...
```

---

## 示例代码

### 完整示例: 语音终端

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from bot_cmd_interface.sdk import (
    CommandRequest,
    CommandResponse,
    create_navigate_request,
    create_emergency_stop_request,
)

class VoiceTerminal(Node):
    def __init__(self):
        super().__init__('voice_terminal')
        
        # 发布器和订阅器
        self.request_pub = self.create_publisher(String, '/cmd/request', 10)
        self.response_sub = self.create_subscription(
            String, '/cmd/response', self._response_callback, 10
        )
        
        # 待处理请求
        self.pending_requests = {}
        
        self.get_logger().info('Voice terminal ready')
    
    def process_voice_command(self, command: str):
        """处理语音命令"""
        if command.startswith("go to"):
            # 解析坐标
            parts = command.split()
            if len(parts) >= 4:
                try:
                    x = float(parts[2])
                    y = float(parts[3])
                    self.navigate_to(x, y)
                except ValueError:
                    self.get_logger().error('Invalid coordinates')
        
        elif command == "emergency stop":
            self.emergency_stop()
        
        else:
            self.get_logger().warn(f'Unknown command: {command}')
    
    def navigate_to(self, x: float, y: float):
        """导航到坐标"""
        request = create_navigate_request(x, y)
        
        def on_response(response: CommandResponse):
            if response.status == 'completed':
                self.speak(f"I have arrived at {x}, {y}")
            elif response.status == 'failed':
                self.speak(f"Navigation failed: {response.message}")
        
        self.send_request(request, on_response)
        self.speak(f"Navigating to {x}, {y}")
    
    def emergency_stop(self):
        """紧急停止"""
        request = create_emergency_stop_request()
        
        def on_response(response: CommandResponse):
            if response.status == 'completed':
                self.speak("Emergency stop completed")
        
        self.send_request(request, on_response)
        self.speak("Stopping immediately")
    
    def send_request(self, request: CommandRequest, callback):
        """发送请求"""
        msg = String()
        msg.data = request.to_json()
        self.request_pub.publish(msg)
        
        self.pending_requests[request.request_id] = callback
        self.get_logger().info(f'Sent: {request.request_id}')
    
    def _response_callback(self, msg: String):
        """响应回调"""
        response = CommandResponse.from_json(msg.data)
        
        if response.request_id in self.pending_requests:
            callback = self.pending_requests[response.request_id]
            callback(response)
            
            if response.is_final():
                del self.pending_requests[response.request_id]
    
    def speak(self, text: str):
        """语音播报（示例）"""
        self.get_logger().info(f'[SPEAK] {text}')
        # 实际实现：调用 TTS 服务

def main():
    rclpy.init()
    terminal = VoiceTerminal()
    
    # 模拟语音命令
    terminal.process_voice_command("go to 1.5 2.0")
    
    rclpy.spin(terminal)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### 完整示例: Web 终端（FastAPI）

```python
from fastapi import FastAPI, WebSocket
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from bot_cmd_interface.sdk import (
    CommandResponse,
    create_navigate_request,
    create_robot_status_request,
)
import json
import threading

app = FastAPI()

class WebTerminalNode(Node):
    def __init__(self):
        super().__init__('web_terminal')
        
        self.request_pub = self.create_publisher(String, '/cmd/request', 10)
        self.response_sub = self.create_subscription(
            String, '/cmd/response', self._response_callback, 10
        )
        
        self.pending_requests = {}
        self.websocket_clients = []
    
    def send_request(self, request):
        msg = String()
        msg.data = request.to_json()
        self.request_pub.publish(msg)
        
        self.pending_requests[request.request_id] = request
        return request.request_id
    
    def _response_callback(self, msg: String):
        response = CommandResponse.from_json(msg.data)
        
        if response.request_id in self.pending_requests:
            # 发送到所有 WebSocket 客户端
            for client in self.websocket_clients:
                client.send_text(json.dumps({
                    'type': 'response',
                    'data': response.to_dict()
                }))
            
            if response.is_final():
                del self.pending_requests[response.request_id]

# 全局节点实例
terminal_node = None

def ros2_spin():
    rclpy.spin(terminal_node)

@app.on_event("startup")
def startup():
    global terminal_node
    rclpy.init()
    terminal_node = WebTerminalNode()
    
    # 在后台线程运行 ROS2
    thread = threading.Thread(target=ros2_spin, daemon=True)
    thread.start()

@app.websocket("/ws")
async def websocket_endpoint(websocket: WebSocket):
    await websocket.accept()
    terminal_node.websocket_clients.append(websocket)
    
    try:
        while True:
            data = await websocket.receive_json()
            
            if data['command'] == 'navigate':
                request = create_navigate_request(
                    data['x'], data['y'], data.get('yaw', 0.0)
                )
                request_id = terminal_node.send_request(request)
                
                await websocket.send_json({
                    'type': 'ack',
                    'request_id': request_id
                })
    
    except Exception as e:
        terminal_node.get_logger().error(f'WebSocket error: {e}')
    finally:
        terminal_node.websocket_clients.remove(websocket)

@app.get("/api/status")
def get_status():
    request = create_robot_status_request()
    request_id = terminal_node.send_request(request)
    return {"request_id": request_id}
```

---

## 故障排查

### 问题 1: 收不到响应

**检查清单**:
1. CommandAdapter 是否运行？
   ```bash
   ros2 node list | grep command_adapter
   ```

2. Topic 是否存在？
   ```bash
   ros2 topic list | grep cmd
   ```

3. 是否正确过滤了响应？
   ```python
   # 确保检查 request_id
   if response.request_id in self.pending_requests:
       # 处理响应
   ```

### 问题 2: 请求被拒绝（重复请求）

**原因**: 5秒内发送了相同的请求

**解决方案**:
- 修改请求参数
- 等待5秒
- 调整配置文件的 `deduplication_window_seconds`

### 问题 3: 响应超时

**可能原因**:
- MissionPlanner 未启动
- 后端服务不可用
- 网络延迟

**解决方案**:
```bash
# 检查 MissionPlanner
ros2 node list | grep mission_planner

# 检查服务是否可用
ros2 service list | grep mission
```

### 问题 4: JSON 解析错误

**检查**:
```python
try:
    response = CommandResponse.from_json(msg.data)
except json.JSONDecodeError as e:
    self.get_logger().error(f'JSON decode error: {e}')
    self.get_logger().debug(f'Raw data: {msg.data}')
```

---

## 参考资料

- **[README.md](../README.md)** - 快速开始指南
- **[API.md](API.md)** - 完整 API 参考
- **[test/cmd_terminal.py](../test/cmd_terminal.py)** - 交互式终端实现（600+ 行参考代码）
- **[test/test_integration.py](../test/test_integration.py)** - 集成测试示例

---

## 需要帮助？

如有问题，请：
1. 查看 [故障排查](#故障排查) 部分
2. 查看日志输出
3. 参考示例代码
4. 联系维护者

---

**集成指南完成** ✅

祝你集成顺利！
