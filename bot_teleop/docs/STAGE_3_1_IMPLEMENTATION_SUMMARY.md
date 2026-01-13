# Stage 3.1 实现总结 - 任务状态实时显示

**实施日期**: 2026-01-12  
**功能模块**: 任务控制与实时状态监控  
**状态**: ✅ 完成待测试

---

## 实现概览

### 已完成功能

#### 1. 前端实现（已完成）
- ✅ `CurrentTaskStatus.tsx` - 任务状态显示组件（136行）
  - 任务信息卡片（ID、类型、状态、进度）
  - 进度条动画（0-100%）
  - 控制按钮（暂停/恢复/取消）
  - 状态标签（queued/executing/completed/failed/paused）
  - 多语言支持（zh-CN）

- ✅ `TaskControl.tsx` - WebSocket 集成
  - 连接 `ws://localhost:8000/ws`
  - 监听 `task_status` 和 `task_response` 消息
  - 调用 API 服务方法（pauseTask/resumeTask/cancelTask）

- ✅ API 服务方法
  - `pauseTask(task_id: number)`
  - `resumeTask(task_id: number)`
  - `cancelTask(task_id: string | number)`

- ✅ 编译验证
  - TypeScript 编译成功（无错误）
  - Bundle 大小：1,097.24 kB
  - 编译时间：4.72s

#### 2. 后端实现（本次新增）

##### 2.1 WebTerminalNode 扩展
**文件**: `web/backend/nodes/web_terminal_node.py`

新增方法：
```python
def pause_task(self, task_id: str) -> str:
    """暂停任务，返回 request_id"""
    request = CommandRequest(
        action=ActionType.PAUSE_TASK,
        params={'task_id': task_id},
        priority=2,
        timeout=5.0
    )
    return self._publish_request(request)

def resume_task(self, task_id: str) -> str:
    """恢复任务，返回 request_id"""
    request = CommandRequest(
        action=ActionType.RESUME_TASK,
        params={'task_id': task_id},
        priority=2,
        timeout=5.0
    )
    return self._publish_request(request)
```

**关键点**：
- ✅ 使用 `bot_cmd_interface.sdk.ActionType`
- ✅ 直接构造 `CommandRequest`（SDK 未提供便捷函数）
- ✅ 遵循短生命周期模型（立即返回 request_id）

##### 2.2 FastAPI 端点
**文件**: `web/backend/api/tasks.py`

新增端点：
```python
@router.post("/tasks/{task_id}/pause")
async def pause_task(task_id: int, node = Depends(get_node)):
    """暂停任务"""
    request_id = node.pause_task(task_id=str(task_id))
    return TaskResponse(
        success=True,
        message=f"Pause command sent for task {task_id}",
        request_id=request_id,
        task_id=str(task_id)
    )

@router.post("/tasks/{task_id}/resume")
async def resume_task(task_id: int, node = Depends(get_node)):
    """恢复任务"""
    request_id = node.resume_task(task_id=str(task_id))
    return TaskResponse(
        success=True,
        message=f"Resume command sent for task {task_id}",
        request_id=request_id,
        task_id=str(task_id)
    )
```

**符合设计原则**：
- ✅ 不等待任务完成（短生命周期）
- ✅ 立即返回 request_id
- ✅ 任务执行状态由 MissionPlanner 维护

##### 2.3 WebSocket 状态推送
**文件**: `web/backend/websocket_handler.py`

新增功能：
1. **后台任务循环**（每 2 秒）
   ```python
   async def _status_update_loop(self):
       while True:
           await asyncio.sleep(2.0)
           if self.active_connections and self.web_terminal_node:
               # 查询所有任务状态
               request_id = self.web_terminal_node.query_task_status(task_id=None)
   ```

2. **响应解析与广播**
   ```python
   async def broadcast_response(self, response):
       # 提取 task_status 信息
       if response.result.get('task_status'):
           task_message = {
               "type": "task_status",
               "task": {
                   "task_id": task_id,
                   "status": self._map_task_status(task_status),
                   "progress": response.result.get('progress', 0)
               }
           }
           await self.broadcast(task_message)
   ```

3. **状态映射**
   ```python
   def _map_task_status(self, task_status: str) -> str:
       """MissionPlanner 状态 → 前端状态"""
       status_map = {
           "RUNNING": "executing",
           "PAUSED": "paused",
           "SUCCESS": "completed",
           "FAILED": "failed",
           "CANCELLED": "cancelled"
       }
   ```

**启动集成**（`web_server.py`）：
```python
# 设置 WebTerminalNode
websocket_handler.set_web_terminal_node(web_terminal_node)

# 启动状态推送
await websocket_handler.start_status_updates()
```

---

## bot_cmd_interface 架构说明

### 关键设计原则（已补充到 IMPLEMENTATION_PLAN.md）

#### 1. 松耦合架构
```
Web前端 → HTTP API → WebTerminalNode → /cmd/request 
  ↓
CommandAdapter（统一接口层）
  ├─ 请求验证、去重
  ├─ 调用 MissionPlanner 服务
  └─ 获得 task_id 后立即响应
  ↓
/cmd/response → WebTerminalNode → WebSocket → 前端
```

#### 2. 短生命周期模型
- **命令请求**：~1秒（queued → executing → completed）
- **获得 task_id** 后立即清理 request_id 状态
- **任务执行**：由 MissionPlanner 维护（可能几秒到几分钟）
- **进度查询**：通过新请求获取（action="get_task_status"）

#### 3. 禁止的错误模式
```python
# ❌ 严禁：直接调用 ROS 服务
from bot_navigation_msgs.srv import StartExploration
mission_client = node.create_client(StartExploration, ...)
```

#### 4. 正确的实现模式
```python
# ✅ 正确：使用 SDK
from bot_cmd_interface.sdk import CommandRequest, ActionType

request = CommandRequest(
    action=ActionType.START_EXPLORATION,
    params={"map_name": "office_floor1"}
)
self.cmd_publisher.publish(String(data=request.to_json()))
```

---

## 测试指南

### 1. 单元测试（后端）
```bash
# 测试脚本
bash src/bot_teleop/scripts/test_backend_stage3.sh

# 预期输出
✓ 服务器健康检查
✓ 创建导航任务
✓ 暂停任务（task_id=1）
✓ 恢复任务（task_id=1）
✓ 查询任务状态
```

### 2. 集成测试（完整环境）

#### 步骤1：启动 ROS2 环境
```bash
# 终端1：启动仿真 + Nav2 + MissionPlanner + CommandAdapter
ros2 launch bot_bringup simulation_cmd_interface_test.launch.py \
  slam:=false map_name:=exploration_test
```

#### 步骤2：启动 Web 后端
```bash
# 终端2：启动 FastAPI 服务器
cd ~/lododo_bot/src/bot_teleop
bash scripts/start_web_server.sh
```

#### 步骤3：前端测试
1. 打开浏览器：http://localhost:8000
2. 进入"任务控制"面板
3. 创建导航任务（输入坐标或点击地图）
4. 观察 CurrentTaskStatus 组件显示
5. 点击"暂停"按钮 → 验证机器人停止
6. 点击"恢复"按钮 → 验证机器人继续
7. 点击"取消"按钮 → 验证任务取消

#### 步骤4：WebSocket 验证
打开浏览器开发者工具 → Network → WS：
```json
// 应该每 2 秒收到状态推送
{
  "type": "task_status",
  "task": {
    "task_id": 123,
    "status": "executing",
    "progress": 0.65,
    "message": "Navigating to goal"
  }
}
```

---

## 已知限制与后续工作

### 当前限制
1. ⚠️ **SDK 缺失便捷函数**：`ActionType.PAUSE_TASK` 已定义，但无 `create_pause_task_request()`
   - **解决方案**：直接使用 `CommandRequest` 构造（已实现）
   - **建议**：未来向 bot_cmd_interface 提交 PR 添加便捷函数

2. ⚠️ **状态推送频率固定**：当前 2 秒间隔
   - **优化建议**：可配置化（通过 web_config.yaml）

3. ⚠️ **任务历史未实现**：TaskHistory 组件待开发（Stage 3.2）

### Stage 3.2 待实现功能
- [ ] TaskHistory 组件（最近 10 个任务）
- [ ] 探索任务专用面板（地图名称、保存选项）
- [ ] 巡逻任务专用面板（路点文件、模式选择）
- [ ] `/api/tasks` GET 端点（任务历史列表）

---

## 文件清单

### 修改的文件
1. `src/bot_teleop/docs/IMPLEMENTATION_PLAN.md`
   - 新增：bot_cmd_interface 架构约束说明（~200行）
   - 包含：禁止模式、正确模式、短生命周期示例

2. `src/bot_teleop/web/backend/nodes/web_terminal_node.py`
   - 新增：`pause_task()` 方法（+18行）
   - 新增：`resume_task()` 方法（+18行）

3. `src/bot_teleop/web/backend/api/tasks.py`
   - 新增：`POST /tasks/{task_id}/pause` 端点（+20行）
   - 新增：`POST /tasks/{task_id}/resume` 端点（+20行）

4. `src/bot_teleop/web/backend/websocket_handler.py`
   - 新增：`_status_update_loop()` 后台任务（+30行）
   - 新增：`broadcast_response()` 增强（任务状态提取）
   - 新增：`_map_task_status()` 状态映射（+15行）
   - 新增：`start_status_updates()` / `stop_status_updates()`

5. `src/bot_teleop/web/backend/web_server.py`
   - 修改：lifespan 启动逻辑（设置 node + 启动推送）

### 新增的文件
1. `src/bot_teleop/scripts/test_backend_stage3.sh`
   - 后端功能测试脚本（50行）

### 前端文件（Stage 3.1 前半部分已完成）
1. `web_frontend/src/components/TaskControl/CurrentTaskStatus.tsx` (136行)
2. `web_frontend/src/components/TaskControl/TaskControl.tsx` (更新)
3. `web_frontend/src/services/api.ts` (更新)
4. `web_frontend/src/locales/zh-CN.json` (更新)

---

## 架构合规性确认

✅ **已遵守 bot_cmd_interface 设计原则**：
- [x] 使用 SDK 构造请求（CommandRequest + ActionType）
- [x] 通过 WebTerminalNode 公共接口调用
- [x] 发布到 /cmd/request Topic（不直接调用服务）
- [x] 订阅 /cmd/response 接收响应
- [x] 遵循短生命周期（立即返回 request_id）
- [x] 任务状态通过查询获取（新请求）
- [x] 未导入 bot_navigation_msgs.srv.*
- [x] 未使用 node.create_client()

---

## 下一步行动

### 立即任务
1. ✅ 完成后端实现（本次完成）
2. ⏳ 执行测试脚本验证后端
3. ⏳ 启动完整环境进行集成测试
4. ⏳ 验证 WebSocket 实时推送

### Stage 3.2 规划
1. 实现 TaskHistory 组件（~4小时）
2. 完善探索/巡逻任务面板（~4小时）
3. 实现 `/api/tasks` 历史查询端点（~2小时）
4. 集成测试与 bug 修复（~2小时）

**预计完成时间**：Stage 3 整体 2 天（当前进度：50%）

---

**文档版本**: v1.0  
**作者**: GitHub Copilot  
**最后更新**: 2026-01-12
