# 部署指南 - bot_cmd_interface

**版本**: v1.0.0  
**最后更新**: 2026-01-08

本指南介绍如何在生产环境中部署和运维 LeKiwi 统一命令接口。

---

## 目录

1. [系统要求](#系统要求)
2. [部署架构](#部署架构)
3. [安装部署](#安装部署)
4. [配置调优](#配置调优)
5. [监控与日志](#监控与日志)
6. [故障恢复](#故障恢复)
7. [性能优化](#性能优化)
8. [安全建议](#安全建议)

---

## 系统要求

### 硬件要求

| 组件 | 最低要求 | 推荐配置 |
|------|---------|---------|
| CPU | 4 核 | 8 核 |
| 内存 | 4 GB | 8 GB |
| 存储 | 20 GB | 50 GB SSD |
| 网络 | 100 Mbps | 1 Gbps |

### 软件要求

- **操作系统**: Ubuntu 22.04 LTS
- **ROS2**: Humble Hawksbill
- **Python**: 3.10+
- **依赖包**: 
  - `rclpy`
  - `std_msgs`
  - `bot_navigation_msgs`
  - `jsonschema >= 4.0.0`

### 环境依赖

```bash
# 检查 ROS2 版本
ros2 --version
# 应该显示: ros2 doctor version X.X.X

# 检查 Python 版本
python3 --version
# 应该显示: Python 3.10.x

# 检查必要的包
ros2 pkg list | grep bot_navigation_msgs
```

---

## 部署架构

### 单机部署（推荐用于开发和小型部署）

```
┌─────────────────────────────────────┐
│       Ubuntu 22.04 Server           │
│                                     │
│  ┌──────────────────────────────┐  │
│  │    CommandAdapter Node       │  │
│  └──────────────────────────────┘  │
│              ↕                      │
│  ┌──────────────────────────────┐  │
│  │    MissionPlanner Node       │  │
│  └──────────────────────────────┘  │
│              ↕                      │
│  ┌──────────────────────────────┐  │
│  │    Terminals (Voice/Web)     │  │
│  └──────────────────────────────┘  │
└─────────────────────────────────────┘
```

### 分布式部署（推荐用于生产环境）

```
┌─────────────────┐      ┌─────────────────┐
│  Terminal Host  │      │  Robot Host     │
│                 │      │                 │
│  ┌──────────┐   │      │  ┌──────────┐   │
│  │ Voice    │───┼──────┼──│CommandAdp│   │
│  │ Terminal │   │ DDS  │  │  ter     │   │
│  └──────────┘   │      │  └──────────┘   │
│                 │      │       ↕         │
│  ┌──────────┐   │      │  ┌──────────┐   │
│  │ Web      │───┼──────┼──│ Mission  │   │
│  │ Terminal │   │      │  │ Planner  │   │
│  └──────────┘   │      │  └──────────┘   │
└─────────────────┘      └─────────────────┘
```

---

## 安装部署

### 步骤 1: 安装系统依赖

```bash
# 更新系统
sudo apt update && sudo apt upgrade -y

# 安装 ROS2 Humble（如果还没有）
sudo apt install ros-humble-desktop -y

# 安装 Python 依赖
pip3 install jsonschema pyyaml
```

### 步骤 2: 克隆和构建

```bash
# 创建工作空间
mkdir -p ~/lododo_bot/src
cd ~/lododo_bot/src

# 克隆代码（替换为你的仓库地址）
git clone <your_repo_url> .

# 构建包
cd ~/lododo_bot
colcon build --packages-select bot_cmd_interface bot_navigation_msgs --symlink-install

# 安装到系统（可选，用于生产环境）
source install/setup.bash
```

### 步骤 3: 配置参数

```bash
# 复制配置模板
cp ~/lododo_bot/src/bot_cmd_interface/config/command_config.yaml \
   ~/lododo_bot/config/command_config_production.yaml

# 编辑生产配置
nano ~/lododo_bot/config/command_config_production.yaml
```

**生产环境推荐配置**:
```yaml
command_adapter:
  ros__parameters:
    # 队列配置
    max_queue_size: 200           # 增加队列容量
    queue_timeout_seconds: 600.0  # 增加超时时间
    
    # 去重配置
    deduplication_window_seconds: 10.0  # 增加去重窗口
    
    # Topic 配置
    request_topic: '/cmd/request'
    response_topic: '/cmd/response'
    
    # 日志级别（生产环境建议 WARN 或 ERROR）
    log_level: 'WARN'
```

### 步骤 4: 创建启动脚本

```bash
# 创建启动脚本
cat > ~/start_cmd_interface.sh << 'EOF'
#!/bin/bash

# 环境变量
source /opt/ros/humble/setup.bash
source ~/lododo_bot/install/setup.bash

# 启动 CommandAdapter
ros2 launch bot_cmd_interface cmd_adapter.launch.py \
  config_file:=~/lododo_bot/config/command_config_production.yaml
EOF

chmod +x ~/start_cmd_interface.sh
```

### 步骤 5: 配置 systemd 服务（可选）

```bash
# 创建 systemd 服务文件
sudo cat > /etc/systemd/system/cmd_interface.service << EOF
[Unit]
Description=LeKiwi Command Interface
After=network.target

[Service]
Type=simple
User=$USER
WorkingDirectory=/home/$USER
ExecStart=/home/$USER/start_cmd_interface.sh
Restart=always
RestartSec=5

[Install]
WantedBy=multi-user.target
EOF

# 启用并启动服务
sudo systemctl daemon-reload
sudo systemctl enable cmd_interface.service
sudo systemctl start cmd_interface.service

# 检查状态
sudo systemctl status cmd_interface.service
```

---

## 配置调优

### 队列容量调整

根据你的使用场景调整队列大小：

```yaml
# 低并发（1-5个终端）
max_queue_size: 50

# 中并发（5-20个终端）
max_queue_size: 100

# 高并发（20+个终端）
max_queue_size: 200
```

### 超时时间调整

```yaml
# 短任务（导航、查询）
queue_timeout_seconds: 60.0

# 中等任务（巡逻）
queue_timeout_seconds: 300.0

# 长任务（探索建图）
queue_timeout_seconds: 1800.0
```

### 去重窗口调整

```yaml
# 严格去重（防止误操作）
deduplication_window_seconds: 10.0

# 宽松去重（快速响应）
deduplication_window_seconds: 3.0

# 禁用去重（不推荐）
deduplication_window_seconds: 0.0
```

### 日志级别调整

```yaml
# 开发环境
log_level: 'DEBUG'

# 测试环境
log_level: 'INFO'

# 生产环境
log_level: 'WARN'

# 严重错误跟踪
log_level: 'ERROR'
```

---

## 监控与日志

### 实时监控

#### 1. 节点状态监控

```bash
# 检查节点是否运行
ros2 node list | grep command_adapter

# 查看节点信息
ros2 node info /command_adapter
```

#### 2. Topic 监控

```bash
# 监控请求 Topic
ros2 topic hz /cmd/request

# 监控响应 Topic
ros2 topic hz /cmd/response

# 实时查看请求
ros2 topic echo /cmd/request

# 实时查看响应
ros2 topic echo /cmd/response
```

#### 3. 系统资源监控

```bash
# CPU 和内存使用
top -p $(pgrep -f command_adapter)

# 详细资源统计
htop
```

### 日志管理

#### 1. 查看日志

```bash
# 实时日志
ros2 topic echo /rosout | grep command_adapter

# 节点日志（如果使用 systemd）
sudo journalctl -u cmd_interface.service -f

# 最近的日志
sudo journalctl -u cmd_interface.service -n 100
```

#### 2. 日志轮转

```bash
# 创建日志轮转配置
sudo cat > /etc/logrotate.d/cmd_interface << EOF
/var/log/cmd_interface.log {
    daily
    rotate 7
    compress
    delaycompress
    missingok
    notifempty
    create 644 $USER $USER
}
EOF
```

#### 3. 自定义日志输出

```bash
# 启动时重定向日志到文件
ros2 launch bot_cmd_interface cmd_adapter.launch.py \
  2>&1 | tee /var/log/cmd_interface.log
```

### 性能监控

#### 1. 响应时间监控

```bash
# 运行性能测试
cd ~/lododo_bot/src/bot_cmd_interface/test
python3 test_cmd_benchmark.py

# 预期结果
# Queued response: <10ms
# Completed response: <20ms
# Throughput: >500 req/s
```

#### 2. 队列监控

```python
# 自定义监控脚本
import rclpy
from rclpy.node import Node

class QueueMonitor(Node):
    def __init__(self):
        super().__init__('queue_monitor')
        
        # 订阅队列状态 Topic（需要在 CommandAdapter 中添加）
        self.subscription = self.create_subscription(
            String,
            '/cmd/queue_status',
            self.callback,
            10
        )
    
    def callback(self, msg):
        # 解析队列状态
        # {"size": 5, "pending": 2, "completed": 120}
        pass
```

---

## 故障恢复

### 常见故障场景

#### 1. CommandAdapter 崩溃

**症状**: 节点不在 `ros2 node list` 中

**恢复步骤**:
```bash
# 如果使用 systemd
sudo systemctl restart cmd_interface.service

# 手动重启
~/start_cmd_interface.sh
```

**预防措施**:
- 使用 systemd 自动重启（`Restart=always`）
- 监控节点状态
- 定期检查日志

#### 2. 队列满导致请求被拒绝

**症状**: 日志显示 "Queue full"

**临时解决**:
```bash
# 重启节点清空队列
sudo systemctl restart cmd_interface.service
```

**长期解决**:
- 增加 `max_queue_size`
- 检查后端服务性能
- 优化任务处理速度

#### 3. 响应超时

**症状**: 请求发送后长时间无响应

**检查**:
```bash
# 检查 MissionPlanner 状态
ros2 node list | grep mission_planner

# 检查服务可用性
ros2 service list | grep mission

# 检查网络连接（分布式部署）
ping <robot_host>
```

**恢复**:
- 重启相关服务
- 检查网络连接
- 增加超时时间

#### 4. 内存泄漏

**症状**: 内存使用持续增长

**诊断**:
```bash
# 监控内存使用
watch -n 1 'ps aux | grep command_adapter'

# 使用 memory_profiler
pip3 install memory_profiler
python3 -m memory_profiler command_adapter_node.py
```

**解决**:
- 定期重启节点
- 修复代码中的内存泄漏
- 实现请求清理机制

---

## 性能优化

### 1. 减少响应延迟

```yaml
# 优化配置
command_adapter:
  ros__parameters:
    # 减小队列大小
    max_queue_size: 50
    
    # 减小去重窗口
    deduplication_window_seconds: 3.0
    
    # 减少日志输出
    log_level: 'ERROR'
```

### 2. 提高吞吐量

```yaml
# 增加队列容量
max_queue_size: 200

# 异步处理（已实现）
# ServiceAdapter 使用 asyncio

# 并发处理（需要修改代码）
# 使用线程池处理多个请求
```

### 3. 优化网络性能（分布式部署）

```bash
# 配置 DDS QoS
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export FASTRTPS_DEFAULT_PROFILES_FILE=/path/to/fastrtps.xml
```

**fastrtps.xml 示例**:
```xml
<?xml version="1.0" encoding="UTF-8" ?>
<profiles xmlns="http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles">
    <transport_descriptors>
        <transport_descriptor>
            <transport_id>CustomUDPTransport</transport_id>
            <type>UDPv4</type>
            <sendBufferSize>8388608</sendBufferSize>
            <receiveBufferSize>8388608</receiveBufferSize>
        </transport_descriptor>
    </transport_descriptors>
</profiles>
```

### 4. 数据库优化（如果使用持久化）

```python
# 使用索引加速查询
# 定期清理过期数据
# 使用内存缓存（Redis）
```

---

## 安全建议

### 1. 网络安全

#### DDS 安全配置

```bash
# 启用 DDS 安全
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export ROS_SECURITY_KEYSTORE=/path/to/keystore
export ROS_SECURITY_ENABLE=true
export ROS_SECURITY_STRATEGY=Enforce
```

#### 防火墙配置

```bash
# 仅允许特定 IP 访问
sudo ufw allow from 192.168.1.0/24 to any port 7400:7500 proto udp

# 启用防火墙
sudo ufw enable
```

### 2. 访问控制

#### 请求来源验证

```python
# 在 CommandAdapter 中验证 source
ALLOWED_SOURCES = ['voice_terminal', 'web_terminal', 'admin_terminal']

def is_valid_source(request: CommandRequest) -> bool:
    return request.source in ALLOWED_SOURCES
```

#### 动作权限控制

```python
# 敏感动作需要特殊权限
PROTECTED_ACTIONS = [
    ActionType.EMERGENCY_STOP,
    ActionType.SAVE_MAP,
    ActionType.LOAD_MAP,
]

def check_permission(request: CommandRequest, user: str) -> bool:
    if request.action in PROTECTED_ACTIONS:
        return user in ADMIN_USERS
    return True
```

### 3. 数据安全

#### 敏感数据加密

```python
# 加密请求参数（如果包含敏感信息）
from cryptography.fernet import Fernet

key = Fernet.generate_key()
cipher = Fernet(key)

# 加密
encrypted_params = cipher.encrypt(json.dumps(params).encode())

# 解密
params = json.loads(cipher.decrypt(encrypted_params).decode())
```

#### 日志脱敏

```python
# 不记录敏感参数
def log_request(request: CommandRequest):
    safe_params = request.params.copy()
    if 'password' in safe_params:
        safe_params['password'] = '***'
    
    logger.info(f'Request: {request.action}, params: {safe_params}')
```

### 4. 系统安全

#### 限制资源使用

```bash
# systemd 服务限制
[Service]
MemoryLimit=2G
CPUQuota=200%
```

#### 定期更新

```bash
# 自动更新脚本
cat > ~/update_cmd_interface.sh << 'EOF'
#!/bin/bash
cd ~/lododo_bot
git pull
colcon build --packages-select bot_cmd_interface
sudo systemctl restart cmd_interface.service
EOF

# 添加到 cron
crontab -e
# 每天凌晨 3 点更新
0 3 * * * ~/update_cmd_interface.sh
```

---

## 备份与恢复

### 配置文件备份

```bash
# 备份配置
tar -czf cmd_interface_config_$(date +%Y%m%d).tar.gz \
  ~/lododo_bot/config/command_config_production.yaml

# 恢复配置
tar -xzf cmd_interface_config_20260108.tar.gz -C ~/
```

### 数据备份（如果使用持久化）

```bash
# 备份请求历史
cp ~/lododo_bot/data/request_history.db \
   ~/backup/request_history_$(date +%Y%m%d).db
```

---

## 生产环境清单

部署前检查：

- [ ] 硬件资源充足（CPU、内存、存储）
- [ ] 所有依赖已安装（ROS2、Python包）
- [ ] 配置文件已优化（队列大小、超时、日志级别）
- [ ] systemd 服务已配置并测试
- [ ] 日志轮转已配置
- [ ] 监控脚本已部署
- [ ] 防火墙规则已配置
- [ ] DDS 安全已启用（如果需要）
- [ ] 备份策略已实施
- [ ] 故障恢复流程已测试

---

## 故障排查清单

遇到问题时按以下顺序检查：

1. [ ] 节点是否运行？ (`ros2 node list`)
2. [ ] Topic 是否正常？ (`ros2 topic list`)
3. [ ] 日志中有错误吗？ (`journalctl -u cmd_interface.service`)
4. [ ] 系统资源是否充足？ (`top`, `df -h`)
5. [ ] 网络连接正常吗？ (`ping`, `ros2 topic hz`)
6. [ ] 配置文件正确吗？ (检查语法和参数)
7. [ ] 后端服务运行正常吗？ (MissionPlanner)

---

## 联系与支持

- **文档**: [README.md](../README.md) | [API.md](API.md)
- **示例**: [test/](../test/)
- **问题报告**: <your_issue_tracker_url>
- **维护者**: [@hurry](https://github.com/hurry)

---

**部署指南完成** ✅

祝你部署顺利！
