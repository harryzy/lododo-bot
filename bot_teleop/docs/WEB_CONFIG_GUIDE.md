# Web界面配置管理

## 配置文件位置

主配置文件：`src/bot_teleop/config/web_config.yaml`

## 配置项说明

```yaml
server:
  host: 0.0.0.0          # Web服务器监听地址
  port: 8000             # Web服务器端口
  reload: false          # 开发模式自动重载
  workers: 1             # 工作进程数

ros:
  use_sim_time: false    # 是否使用仿真时间

websocket:
  ping_interval: 30      # WebSocket心跳间隔（秒）
  ping_timeout: 10       # WebSocket超时时间（秒）

rosbridge:
  url: ws://localhost:9090           # ROSBridge URL（预留）
  reconnect_attempts: 5              # 重连尝试次数
  reconnect_interval: 5              # 重连间隔（秒）

update_rates:
  map: 1.0               # 地图更新频率（Hz）
  pose: 10.0             # 位姿更新频率（Hz）
  costmap: 5.0           # Costmap更新频率（Hz）
  planned_path: 2.0      # 规划路径更新频率（Hz）

costmap:
  opacity: 0.5           # Costmap透明度
  inflation_color: rgba(255, 165, 0, 0.5)  # 膨胀区颜色
  lethal_color: rgba(255, 0, 0, 0.5)       # 致命区颜色

ui:
  language: zh-CN        # 默认语言（zh-CN/en-US）
  theme: light           # 默认主题（light/dark）

paths:
  maps_dir: ~/workDisk/lododo_bot/maps          # 地图存储目录
  waypoints_dir: ~/workDisk/lododo_bot/waypoints # 路点存储目录
```

## 配置优先级

系统按以下优先级读取配置：

1. **配置文件** (`web_config.yaml`) - 最高优先级
2. **环境变量** - 次优先级（如 `LODODO_MAPS_DIR`）
3. **自动检测** - 最低优先级（从工作空间自动推断）

## 修改配置

### 方法1：直接编辑配置文件

```bash
vim ~/workDisk/lododo_bot/src/bot_teleop/config/web_config.yaml
```

修改后重启Web服务器生效。

### 方法2：使用环境变量覆盖（不推荐）

```bash
export LODODO_MAPS_DIR=~/custom_maps
```

环境变量会被配置文件覆盖，不建议使用。

## 前端配置获取

前端通过 `configService` 获取配置：

```typescript
import { configService } from '@/services/config'

// 获取WebSocket URL
const wsUrl = await configService.getWebSocketUrl()

// 获取完整配置
const config = await configService.getConfig()
```

## 后端配置读取

后端通过 `load_config()` 读取配置：

```python
from api.config import load_config

config = load_config()
maps_dir = config.get('paths', {}).get('maps_dir', '~/lododo_bot/maps')
```

## 配置检查

检查当前使用的配置：

```bash
# 查看Web服务器启动日志
# 会显示实际使用的地图目录、路点目录等

# 前端访问配置API
curl http://localhost:8000/api/config
```

## 常见问题

### Q: 修改配置后不生效？

A: 需要重启Web服务器：
```bash
# 停止当前服务
pkill -f "uvicorn.*web_server"

# 重新启动
ros2 launch bot_teleop web_ui.launch.py
```

### Q: 如何使用不同的端口？

A: 修改 `web_config.yaml` 中的 `server.port`：
```yaml
server:
  port: 8080  # 改为8080端口
```

### Q: 地图和路点目录在哪里？

A: 查看配置文件中的 `paths` 部分，支持 `~` 符号：
```yaml
paths:
  maps_dir: ~/workDisk/lododo_bot/maps
  waypoints_dir: ~/workDisk/lododo_bot/waypoints
```

## 迁移指南（避免硬编码）

**✅ 正确做法**：
```typescript
// 前端：使用configService
const wsUrl = await configService.getWebSocketUrl()
```

```python
# 后端：从配置读取
from api.config import load_config
config = load_config()
port = config.get('server', {}).get('port', 8000)
```

**❌ 错误做法（硬编码）**：
```typescript
// 不要这样！
const wsUrl = 'ws://localhost:8000/ws'  // 硬编码端口
```

```python
# 不要这样！
maps_dir = '/home/user/lododo_bot/maps'  # 硬编码路径
```

## 架构说明

```
配置文件 (web_config.yaml)
    ↓
后端API (/api/config)
    ↓
前端ConfigService
    ↓
React组件使用配置
```

所有配置统一管理，避免硬编码，易于维护和部署。
