# CommandAdapter Test Scripts

## test_command_adapter.py

集成测试脚本，验证CommandAdapter节点的完整请求-响应流程。

Integration test script for validating CommandAdapter node's complete request-response flow.

### 功能特性 / Features

- ✅ 使用SDK构造和解析消息（模拟真实客户端）/ Uses SDK to construct and parse messages (simulating real clients)
- ✅ 验证完整的响应流程：queued → executing → completed
- ✅ 测试请求去重功能 / Tests request deduplication
- ✅ 测试错误处理（无效JSON）/ Tests error handling (invalid JSON)
- ✅ 异步响应处理（不依赖响应顺序）/ Async response handling (order-independent)

### 使用方法 / Usage

#### 1. 启动CommandAdapter节点 / Start CommandAdapter node

```bash
cd ~/lododo_bot
source install/setup.bash
ros2 launch bot_cmd_interface cmd_adapter.launch.py
```

#### 2. 运行测试脚本 / Run test script

在另一个终端中 / In another terminal:

```bash
cd ~/lododo_bot
source install/setup.bash
python3 src/bot_cmd_interface/scripts/test_command_adapter.py
```

### 测试用例 / Test Cases

1. **GET_ROBOT_STATUS** - 获取机器人状态
   - 验证SDK构造请求
   - 验证3个响应状态（queued/executing/completed）
   - 验证mock结果包含battery_level, status等字段

2. **EMERGENCY_STOP** - 紧急停止
   - 验证紧急停止命令处理
   - 验证返回cancelled_task_count

3. **NAVIGATE_TO_POSE** - 导航到目标点
   - 验证带参数的请求
   - 验证返回mock task_id

4. **Request Deduplication** - 请求去重
   - 发送两个相同request_id的请求
   - 验证第二个请求被拒绝

5. **Invalid JSON Format** - 无效JSON
   - 发送格式错误的JSON
   - 验证返回error响应

### 预期输出 / Expected Output

```
============================================================
Test Summary
============================================================
GET_ROBOT_STATUS               ✅ PASSED
EMERGENCY_STOP                 ✅ PASSED
NAVIGATE_TO_POSE               ✅ PASSED
Request Deduplication          ✅ PASSED
Invalid JSON                   ✅ PASSED
------------------------------------------------------------
Total: 5/5 tests passed
============================================================
```

### 作为客户端参考 / As Client Reference

这个测试脚本展示了如何正确使用SDK与CommandAdapter通信，可作为未来客户端实现的参考：

This test script demonstrates how to properly use SDK to communicate with CommandAdapter, serving as a reference for future client implementations:

```python
from bot_cmd_interface.sdk.message import CommandRequest, CommandResponse
from bot_cmd_interface.sdk.action_types import ActionType

# 1. 构造请求 / Construct request
request = CommandRequest(
    action=ActionType.GET_ROBOT_STATUS,
    params={},
    priority=3,
    timeout=300.0
)

# 2. 序列化并发布 / Serialize and publish
msg = String()
msg.data = request.to_json()
publisher.publish(msg)

# 3. 接收并解析响应 / Receive and parse response
def callback(msg):
    response = CommandResponse.from_json(msg.data)
    print(f"Status: {response.status}, Result: {response.result}")
```

### 故障排查 / Troubleshooting

**测试失败：无法连接到CommandAdapter**
- 确认CommandAdapter节点正在运行
- 检查topic名称是否正确：`/cmd/request`, `/cmd/response`

**测试超时**
- 增加`verify_response_flow()`中的timeout参数
- 检查CommandAdapter日志是否有错误

**响应顺序问题**
- 正常现象！多线程环境下响应顺序可能不一致
- 测试脚本已处理异步响应，只检查状态集合而非顺序
