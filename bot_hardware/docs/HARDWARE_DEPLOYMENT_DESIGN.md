# lododo机器人真机硬件部署详细设计文档

**项目名称**: lododo三轮全向移动机器人真机部署  
**基于需求**: [HARDWARE_DEPLOYMENT_REQUIREMENTS.md](HARDWARE_DEPLOYMENT_REQUIREMENTS.md)  
**设计版本**: v0.1 (草稿)  
**创建日期**: 2026-01-15  
**状态**: 🔍 设计审核中

---

## 目录

1. [系统架构设计](#1-系统架构设计)
2. [数据流设计](#2-数据流设计)
3. [模块详细设计](#3-模块详细设计)
4. [关键设计决策](#4-关键设计决策)
5. [风险点与应对策略](#5-风险点与应对策略)
6. [测试策略](#6-测试策略)
7. [与仿真环境的差异分析](#7-与仿真环境的差异分析)

---

## 1. 系统架构设计

### 1.1 整体架构图

```
┌─────────────────────────────────────────────────────────────────────┐
│                        lododo真机硬件层                              │
├─────────────────────────────────────────────────────────────────────┤
│                                                                     │
│   ┌──────────────┐      ┌──────────────┐      ┌──────────────┐    │
│   │  ST3215      │      │  Astra Pro   │      │  IMU         │    │
│   │  Servos ×3   │      │  Camera      │      │  Sensor      │    │
│   │              │      │              │      │              │    │
│   │  /dev/       │      │  USB 3.0     │      │  I2C/Serial  │    │
│   │  ttyACM0     │      │              │      │              │    │
│   └──────┬───────┘      └──────┬───────┘      └──────┬───────┘    │
│          │                     │                     │             │
└──────────┼─────────────────────┼─────────────────────┼─────────────┘
           │                     │                     │
           ▼                     ▼                     ▼
┌─────────────────────────────────────────────────────────────────────┐
│                        ROS2 硬件接口层                               │
├─────────────────────────────────────────────────────────────────────┤
│                                                                     │
│   ┌──────────────────────────────────────────────────────────┐    │
│   │  ST3215Driver                                            │    │
│   │  - 串口通信管理                                            │    │
│   │  - 指令封装/解析                                           │    │
│   │  - 错误处理与重试                                          │    │
│   └────────────┬─────────────────────────────────────────────┘    │
│                │                                                   │
│   ┌────────────▼──────────────────────────────────────────────┐   │
│   │  OmniHardwareInterface (ros2_control SystemInterface)     │   │
│   │  - read(): 读取编码器 → 正向运动学 → /wheel/odom         │   │
│   │  - write(): /cmd_vel → 逆向运动学 → 舵机速度指令          │   │
│   │  - 50Hz控制循环                                           │   │
│   └───────────────────────────────────────────────────────────┘   │
│                                                                     │
│   ┌──────────────────────┐      ┌──────────────────────┐          │
│   │  astra_camera_node   │      │  imu_driver_node     │          │
│   │  (官方驱动)           │      │  (待确定型号)         │          │
│   │  → /camera/*         │      │  → /imu/data         │          │
│   └──────────────────────┘      └──────────────────────┘          │
│                                                                     │
└─────────────────────────────────────────────────────────────────────┘
           │                     │                     │
           ▼                     ▼                     ▼
┌─────────────────────────────────────────────────────────────────────┐
│                        导航与感知层 (复用仿真代码)                   │
├─────────────────────────────────────────────────────────────────────┤
│                                                                     │
│   ┌─────────────────────┐       ┌─────────────────────┐            │
│   │ robot_localization  │       │  RTABMap SLAM       │            │
│   │ (EKF融合)            │       │  (视觉SLAM)          │            │
│   │ - /wheel/odom       │       │  - RGB-D输入        │            │
│   │ - /imu/data         │       │  - 地图构建/定位    │            │
│   │ → /odometry/filtered│       │  → /map, /rtabmap/* │            │
│   └─────────────────────┘       └─────────────────────┘            │
│                                                                     │
│   ┌─────────────────────────────────────────────────────────┐      │
│   │  Nav2 Navigation Stack                                  │      │
│   │  - 路径规划、局部避障、控制器                              │      │
│   │  - DWB控制器 → /cmd_vel                                 │      │
│   └─────────────────────────────────────────────────────────┘      │
│                                                                     │
│   ┌─────────────────────────────────────────────────────────┐      │
│   │  Mission Planner (任务管理)                              │      │
│   │  - 探索、巡航、导航任务编排                                │      │
│   └─────────────────────────────────────────────────────────┘      │
│                                                                     │
└─────────────────────────────────────────────────────────────────────┘
```

### 1.2 分层设计说明

**硬件层** (新开发):
- 直接与物理硬件交互
- 提供ROS2标准接口
- 硬件抽象，对上层透明

**ROS2硬件接口层** (新开发):
- ST3215Driver: 封装串口协议
- OmniHardwareInterface: ros2_control标准接口
- 传感器驱动集成

**导航与感知层** (复用):
- 与仿真环境共享代码
- 通过配置切换（EKF配置、use_sim_time等）
- 无需修改核心算法

### 1.3 设计原则

1. **最小侵入原则**: 尽量复用现有仿真代码，只在硬件接口层新增代码
2. **配置驱动**: 通过YAML配置和launch参数区分仿真/真机
3. **标准接口**: 遵循ros2_control标准，便于未来扩展
4. **故障隔离**: 硬件层故障不影响上层逻辑，能优雅降级

### 1.4 核心编码规范 ⚠️ 强制要求

#### 1.4.1 统一配置管理 🔒

**原则**: 所有代码中的参数禁止硬编码，必须从统一配置文件读取

**配置文件路径**: `bot_hardware/config/hardware_config.yaml`

**适用范围**:
- ✅ 硬件参数：串口路径、波特率、设备ID
- ✅ 运动学参数：轮子半径、轮距、速度限制
- ✅ 传感器参数：分辨率、帧率、发布频率
- ✅ 控制参数：循环频率、超时时间、重试次数
- ✅ 路径配置：地图目录、日志路径、标定数据路径
- ✅ 安全参数：紧急停止阈值、过载保护阈值

**正确示例**:
```python
# ✅ 正确：从配置文件读取
class ST3215Driver:
    def __init__(self, config_file):
        # 加载配置 / Load configuration
        with open(config_file, 'r') as f:
            config = yaml.safe_load(f)
        
        # 从配置读取参数 / Read parameters from config
        self.port = config['serial']['servo_port']
        self.baudrate = config['serial']['servo_baudrate']
        self.timeout = config['serial']['servo_timeout']
        self.wheel_1_id = config['servo']['wheel_1_id']
        
        self.get_logger().info(f'Servo port: {self.port}, baudrate: {self.baudrate}')
```

**错误示例**:
```python
# ❌ 错误：硬编码参数
class ST3215Driver:
    def __init__(self):
        self.port = '/dev/ttyACM0'        # 硬编码串口路径
        self.baudrate = 1000000           # 硬编码波特率
        self.timeout = 0.01               # 硬编码超时
        self.wheel_1_id = 1               # 硬编码舵机ID
```

**配置文件加载方式**:
```python
import yaml
import os
from ament_index_python.packages import get_package_share_directory

# 方式1：直接加载（推荐用于驱动类）
config_path = os.path.join(
    get_package_share_directory('bot_hardware'),
    'config', 'hardware_config.yaml'
)
with open(config_path, 'r') as f:
    config = yaml.safe_load(f)

# 方式2：通过ROS2参数服务器（推荐用于节点）
self.declare_parameter('config_file', config_path)
config_file = self.get_parameter('config_file').value
```

**违反此原则的后果**:
- 🚫 代码审查不通过
- 🚫 真机部署时参数调整困难
- 🚫 多机器人部署时无法快速配置
- 🚫 参数版本管理混乱

---

#### 1.4.2 双语注释与英文日志 🌐

**原则1**: 所有代码注释必须使用中英文双语

**格式要求**:
```python
# 中文说明 / English explanation
```

**注释示例**:
```python
# ✅ 正确：双语注释
class OmniHardwareInterface(SystemInterface):
    def __init__(self):
        # 初始化舵机驱动 / Initialize servo driver
        self.driver = ST3215Driver(config_file)
        
        # 上次编码器读取时间戳 / Last encoder read timestamp
        self.last_read_time = time.time()
        
        # 累积位姿 (x, y, theta) / Accumulated pose (x, y, theta)
        self.pose = [0.0, 0.0, 0.0]
    
    def read(self, time, duration):
        # 读取三个舵机的编码器位置 / Read encoder positions from three servos
        positions = []
        for servo_id in [1, 2, 3]:
            pos = self.driver.read_position(servo_id)
            positions.append(pos)
        
        # 正向运动学计算位姿增量 / Calculate pose increment using forward kinematics
        delta_pose = self._forward_kinematics(positions)
        
        # 更新累积位姿 / Update accumulated pose
        self.pose[0] += delta_pose[0]
        self.pose[1] += delta_pose[1]
        self.pose[2] += delta_pose[2]
        
        return hardware_interface.return_type.OK
```

**错误示例**:
```python
# ❌ 错误：仅中文注释
def read_encoder(self):
    # 读取编码器
    position = self.serial.read()
    # 检查校验和
    if not self.verify_checksum(position):
        return None
    return position

# ❌ 错误：仅英文注释
def calculate_velocity(self, delta_pos, delta_time):
    # Calculate wheel velocity in rad/s
    velocity = (delta_pos * 2 * math.pi) / (self.encoder_resolution * delta_time)
    return velocity
```

---

**原则2**: 所有日志输出必须使用纯英文

**日志示例**:
```python
# ✅ 正确：英文日志
self.get_logger().info('Hardware interface initialized successfully')
self.get_logger().warn(f'Servo {servo_id} response timeout, retrying...')
self.get_logger().error(f'Failed to read encoder from servo {servo_id} after {retry_times} attempts')
self.get_logger().debug(f'Wheel velocities: [{w1:.3f}, {w2:.3f}, {w3:.3f}] rad/s')

# 带数值的日志
self.get_logger().info(f'Odometry published at {publish_rate:.1f} Hz')
self.get_logger().warn(f'IMU data variance too high: {variance:.6f} (threshold: 0.01)')

# 状态转换日志
self.get_logger().info('State transition: IDLE -> ACTIVE')
self.get_logger().info(f'Controller loaded: {controller_name}')
```

**错误示例**:
```python
# ❌ 错误：中文日志（严禁！）
self.get_logger().info('硬件接口初始化成功')
self.get_logger().warn(f'舵机{servo_id}响应超时，正在重试...')
self.get_logger().error(f'读取舵机{servo_id}编码器失败，已重试{retry_times}次')

# ❌ 错误：中英混合日志
self.get_logger().info(f'Servo {servo_id} 初始化成功')
self.get_logger().warn('响应超时 timeout detected')
```

**日志级别使用指南**:
- `DEBUG`: 详细调试信息（如每次读取的原始数据）
- `INFO`: 重要状态信息（如节点启动、初始化完成）
- `WARN`: 警告信息（如传感器超时但已恢复）
- `ERROR`: 错误信息（如硬件通信失败）
- `FATAL`: 致命错误（如硬件完全不可用）

**日志格式规范**:
```python
# 简洁明了，避免冗余
self.get_logger().info('Servo driver initialized')  # ✅ 简洁
self.get_logger().info('The servo driver has been initialized successfully')  # ❌ 冗长

# 包含关键数值时使用格式化
self.get_logger().info(f'Update rate: {rate} Hz')  # ✅ 清晰
self.get_logger().info(f'The current update rate is {rate} Hz')  # ❌ 冗长

# 错误日志包含足够的上下文
self.get_logger().error(f'Servo communication failed: port={self.port}, id={servo_id}, error={e}')  # ✅ 详细
self.get_logger().error('Communication failed')  # ❌ 信息不足
```

**违反此原则的后果**:
- 🚫 代码审查不通过
- 🚫 国际化支持困难
- 🚫 日志分析工具无法正常解析中文
- 🚫 开源社区无法理解代码逻辑

---

#### 1.4.3 编码规范检查

**代码提交前自检清单**:
- [ ] 所有参数从`hardware_config.yaml`读取，无硬编码
- [ ] 所有注释使用中英双语格式 `# 中文 / English`
- [ ] 所有日志输出使用纯英文
- [ ] 类名使用`PascalCase`，函数名使用`snake_case`
- [ ] 私有方法使用`_snake_case`前缀
- [ ] ROS节点名使用`snake_case`

**自动化检查工具** (待开发):
```bash
# 检查硬编码（搜索常见硬编码模式）
./scripts/check_hardcode.sh

# 检查中文日志（搜索中文字符在logger调用中）
./scripts/check_chinese_log.sh

# 检查注释格式（验证双语注释比例）
./scripts/check_comments.sh
```

---

#### 1.4.4 相对路径管理 📁

**原则**: 所有配置文件和接口传入的文件系统路径必须使用ROS工作空间下的相对路径

**设计目标**:
- ✅ 支持环境迁移（开发环境→生产环境→其他机器人）
- ✅ 避免硬编码用户主目录（~）或绝对路径（/home/user/...）
- ✅ 统一路径解析逻辑，便于调试和维护
- ✅ 配置文件可直接版本管理，无需修改

**路径基准**:
```
ROS2工作空间根目录: ~/lododo_bot/
相对路径基准: ~/lododo_bot/  (通过环境变量或程序自动获取)
```

**路径格式要求**:
```yaml
# ✅ 正确：相对路径（从工作空间根目录开始）
maps_directory: 'maps'
waypoints_directory: 'waypoints'
log_directory: 'log'
calibration_file: 'calibration/imu_bias.yaml'
config_file: 'src/bot_hardware/config/hardware_config.yaml'

# ❌ 错误：绝对路径（禁止！）
maps_directory: '/home/hurry/lododo_bot/maps'
waypoints_directory: '~/lododo_bot/waypoints'
log_directory: '/home/user/lododo_bot/log'

# ❌ 错误：使用~展开符（依赖shell环境）
maps_directory: '~/lododo_bot/maps'
calibration_file: '~/.ros/calibration.yaml'
```

**路径解析工具类**:
```python
import os
from pathlib import Path
from ament_index_python.packages import get_package_share_directory

class PathManager:
    """路径管理器 / Path Manager"""
    
    def __init__(self):
        # 获取ROS2工作空间根目录 / Get ROS2 workspace root
        self.workspace_root = self._get_workspace_root()
        
    def _get_workspace_root(self) -> Path:
        """获取工作空间根目录 / Get workspace root directory"""
        # 方式1: 通过环境变量（推荐）
        if 'COLCON_PREFIX_PATH' in os.environ:
            # /path/to/lododo_bot/install -> /path/to/lododo_bot
            install_path = Path(os.environ['COLCON_PREFIX_PATH'].split(':')[0])
            workspace_root = install_path.parent
            return workspace_root
        
        # 方式2: 通过package路径推导
        try:
            pkg_share = get_package_share_directory('bot_hardware')
            # /path/to/lododo_bot/install/bot_hardware/share/bot_hardware
            # -> /path/to/lododo_bot
            workspace_root = Path(pkg_share).parent.parent.parent.parent
            return workspace_root
        except Exception:
            pass
        
        # 方式3: 通过当前文件路径推导（最后备选）
        current_file = Path(__file__).resolve()
        # 假设当前文件在 src/bot_hardware/...
        for parent in current_file.parents:
            if (parent / 'src').exists() and (parent / 'install').exists():
                return parent
        
        raise RuntimeError('Cannot determine workspace root directory')
    
    def resolve_path(self, relative_path: str, create_if_missing: bool = False) -> Path:
        """解析相对路径为绝对路径 / Resolve relative path to absolute path
        
        Args:
            relative_path: 相对于工作空间根目录的路径 / Path relative to workspace root
            create_if_missing: 如果目录不存在是否创建 / Create directory if missing
        
        Returns:
            Path: 绝对路径对象 / Absolute path object
        
        Raises:
            ValueError: 如果传入绝对路径 / If absolute path is provided
            FileNotFoundError: 如果路径不存在且不创建 / If path doesn't exist and not creating
        """
        # 验证不是绝对路径 / Validate not absolute path
        if os.path.isabs(relative_path) or relative_path.startswith('~'):
            raise ValueError(
                f'Absolute path detected: {relative_path}. '
                f'Only relative paths from workspace root are allowed. '
                f'Example: "maps" instead of "/home/user/lododo_bot/maps"'
            )
        
        # 解析为绝对路径 / Resolve to absolute path
        absolute_path = self.workspace_root / relative_path
        
        # 创建目录（如果需要） / Create directory if needed
        if create_if_missing and not absolute_path.exists():
            if '.' in absolute_path.name:  # 文件路径，创建父目录
                absolute_path.parent.mkdir(parents=True, exist_ok=True)
            else:  # 目录路径，创建目录本身
                absolute_path.mkdir(parents=True, exist_ok=True)
        
        return absolute_path
    
    def validate_config_paths(self, config: dict) -> None:
        """验证配置文件中的所有路径 / Validate all paths in config
        
        Args:
            config: 配置字典 / Configuration dictionary
        
        Raises:
            ValueError: 如果发现绝对路径 / If absolute path found
        """
        path_keys = ['directory', 'path', 'file', 'dir']  # 常见路径关键字
        
        def check_paths(obj, path_prefix=''):
            if isinstance(obj, dict):
                for key, value in obj.items():
                    current_path = f'{path_prefix}.{key}' if path_prefix else key
                    if any(keyword in key.lower() for keyword in path_keys):
                        if isinstance(value, str) and value:
                            # 检查是否为绝对路径
                            if os.path.isabs(value) or value.startswith('~'):
                                raise ValueError(
                                    f'Absolute path in config at "{current_path}": {value}\n'
                                    f'Please use relative path from workspace root.\n'
                                    f'Example: "maps" instead of "{value}"'
                                )
                    if isinstance(value, dict):
                        check_paths(value, current_path)
        
        check_paths(config)

# 使用示例 / Usage Example
class HardwareNode(Node):
    def __init__(self):
        super().__init__('hardware_node')
        
        # 初始化路径管理器 / Initialize path manager
        self.path_manager = PathManager()
        
        # 加载配置文件 / Load config file
        config_relative = 'src/bot_hardware/config/hardware_config.yaml'
        config_path = self.path_manager.resolve_path(config_relative)
        
        with open(config_path, 'r') as f:
            config = yaml.safe_load(f)
        
        # 验证配置中的路径 / Validate paths in config
        try:
            self.path_manager.validate_config_paths(config)
            self.get_logger().info('Configuration paths validated successfully')
        except ValueError as e:
            self.get_logger().error(f'Invalid path in configuration: {e}')
            raise
        
        # 解析地图目录 / Resolve maps directory
        maps_dir = self.path_manager.resolve_path(
            config['paths']['maps_directory'], 
            create_if_missing=True
        )
        self.get_logger().info(f'Maps directory: {maps_dir}')
        
        # 解析标定文件 / Resolve calibration file
        calib_file = self.path_manager.resolve_path(
            'calibration/imu_bias.yaml'
        )
        if calib_file.exists():
            self.get_logger().info(f'Loading calibration from: {calib_file}')
```

**配置文件路径规范**:
```yaml
# hardware_config.yaml 中的路径配置示例
paths:
  # ✅ 所有路径都是相对路径（从工作空间根目录开始）
  maps_directory: 'maps'                    # → ~/lododo_bot/maps
  waypoints_directory: 'waypoints'          # → ~/lododo_bot/waypoints
  logs_directory: 'log'                     # → ~/lododo_bot/log
  calibration_directory: 'calibration'      # → ~/lododo_bot/calibration
  
  # 配置文件路径（在src目录下）
  hardware_config: 'src/bot_hardware/config/hardware_config.yaml'
  
  # 标定数据文件
  imu_calibration_file: 'calibration/imu_bias.yaml'
  camera_calibration_file: 'calibration/camera_info.yaml'
```

**launch文件中的路径处理**:
```python
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
from pathlib import Path

def get_workspace_root():
    """获取工作空间根目录 / Get workspace root"""
    if 'COLCON_PREFIX_PATH' in os.environ:
        install_path = Path(os.environ['COLCON_PREFIX_PATH'].split(':')[0])
        return str(install_path.parent)
    pkg_share = get_package_share_directory('bot_hardware')
    return str(Path(pkg_share).parent.parent.parent.parent)

def generate_launch_description():
    workspace_root = get_workspace_root()
    
    # 配置文件使用相对路径 / Use relative path for config
    config_relative = 'src/bot_hardware/config/hardware_config.yaml'
    config_file = os.path.join(workspace_root, config_relative)
    
    return LaunchDescription([
        Node(
            package='bot_hardware',
            executable='omni_hardware_interface',
            name='omni_hardware_interface',
            parameters=[{
                'config_file': config_file,
                'workspace_root': workspace_root  # 传递给节点
            }]
        )
    ])
```

**路径迁移检查工具**:
```bash
#!/bin/bash
# scripts/check_absolute_paths.sh
# 检查配置文件中是否有绝对路径 / Check for absolute paths in config files

echo "Checking for absolute paths in configuration files..."

find src/ -name "*.yaml" -o -name "*.yml" | while read file; do
    # 检查是否包含绝对路径模式
    if grep -E "(: ['\"]?/|: ['\"]?~/)" "$file"; then
        echo "❌ Found absolute path in: $file"
        exit 1
    fi
done

echo "✅ All configuration files use relative paths"
```

**违反此原则的后果**:
- 🚫 代码审查不通过
- 🚫 配置文件无法在不同机器人间共享
- 🚫 环境迁移时需要手动修改路径
- 🚫 团队协作时产生路径冲突
- 🚫 Docker容器化部署失败

**最佳实践**:
1. 所有新文件路径使用相对路径
2. 启动时验证配置文件中的路径
3. 提供路径转换工具类供所有节点使用
4. CI/CD流程中加入路径检查
5. 文档中明确说明路径规范

---

## 2. 数据流设计

### 2.1 控制指令流 (cmd_vel → 轮子转动)

```
Nav2 DWB Controller
      │ /cmd_vel (Twist: vx, vy, ω)
      ▼
ros2_control framework
      │ joint velocity commands
      ▼
OmniHardwareInterface::write()
      │ 逆向运动学计算
      ▼ wheel velocities (rad/s)
      │ 单位转换: rad/s → RPM
      ▼
ST3215Driver::write_speed()
      │ 构造指令包
      ▼
Serial Port (/dev/ttyACM0)
      │ TTL串口通信
      ▼
ST3215 Servos ×3 → 机器人移动
```

**关键设计点**:
- 逆向运动学在`OmniHardwareInterface`中实现，复用`omni_controller_node.py`的雅可比矩阵
- 需要考虑轮速限制（ST3215最大45 RPM）和加速度限制
- 异常处理：串口超时、舵机过载、指令校验失败

🔖 **待补充**: 第2轮设计补充逆向运动学矩阵推导、单位转换公式、异常处理流程图

---

### 2.2 里程计反馈流 (编码器 → /wheel/odom)

```
ST3215 Servos (编码器 4096分辨率)
      │
      ▼
ST3215Driver::read_position()
      │ 定时轮询 (50Hz)
      ▼
OmniHardwareInterface::read()
      │ 编码器差分计算
      │ 正向运动学: Δencoder → Δpose
      ▼
/wheel/odom (nav_msgs/Odometry)
      │ 包含: pose(x,y,θ) + twist(vx,vy,vθ)
      ▼
robot_localization (EKF)
      │ 融合 /wheel/odom + /imu/data
      ▼
/odometry/filtered → Nav2使用
```

**关键设计点**:
- 编码器读取频率与控制周期一致（50Hz）
- 正向运动学计算累积位姿，需处理编码器溢出（0→4095→0循环）
- 协方差矩阵设置：真机里程计不确定度远大于仿真

🔖 **待补充**: 第2轮设计补充正向运动学推导、编码器溢出处理逻辑、协方差矩阵调参指南

---

### 2.3 传感器数据流

```
┌─────────────────┐     ┌─────────────────┐
│  Astra Pro      │     │  IMU Sensor     │
│  RGB-D Camera   │     │  陀螺仪+加速度计  │
└────────┬────────┘     └────────┬────────┘
         │ 30fps                 │ 50-100Hz
         ▼                       ▼
/camera/color/image_raw    /imu/data
/camera/depth/image_raw    (linear_acc + angular_vel + orientation)
         │                       │
         ▼                       ▼
   RTABMap SLAM            robot_localization EKF
         │                       │
         ▼                       ▼
      /map                /odometry/filtered
/rtabmap/localization_pose      │
         │                       │
         └───────────┬───────────┘
                     ▼
              Nav2 Navigation
```

**关键设计点**:
- 相机帧率与深度同步（astra_camera驱动默认已处理）
- IMU数据必须包含角速度（gyro），用于EKF yaw融合
- 时间戳同步：所有传感器使用系统时间（use_sim_time=false）

🔖 **待补充**: 第3轮设计补充IMU静态标定流程、相机内参标定方法、时间戳同步验证方案

---

## 3. 模块详细设计

### 3.1 ST3215舵机驱动模块 (ST3215Driver)

#### 3.1.1 设计目标

封装ST3215舵机的TTL串口通信协议，提供简洁的Python API供上层调用。

#### 3.1.2 核心功能

| 功能 | 接口 | 说明 |
|------|------|------|
| 速度控制 | `write_speed(servo_id, rpm)` | 设置舵机目标速度 |
| 位置读取 | `read_position(servo_id)` | 读取编码器绝对位置 |
| 状态查询 | `read_status(servo_id)` | 读取电压、温度、负载 |
| 初始化 | `initialize()` | 串口连接、舵机ID扫描 |
| 错误处理 | `get_last_error()` | 获取通信错误信息 |

#### 3.1.3 协议设计

**指令包结构** (参考ST3215手册):
```
[0xFF] [0xFF] [ID] [Length] [Instruction] [Param1...N] [Checksum]
  帧头   帧头   设备ID  数据长度    指令码        参数          校验和
```

**设计考虑**:
- 支持波特率自适应（1Mbps首选，115200备选）
- 校验和算法：`~(ID + Length + Instruction + Params) & 0xFF`
- 超时机制：单次读取超时10ms，失败重试3次
- 并发保护：多线程访问时使用线程锁

🔖 **待补充**: 第2轮设计补充完整指令码表、参数字节序（大端/小端）、错误码定义

#### 3.1.4 关键设计问题

**Q1: 如何处理舵机ID冲突或未响应？**
- 设计思路：启动时遍历ID 1-3，发送ping指令，记录响应舵机
- 失败处理：如果任意舵机未响应，发布警告但不阻止启动，等待人工检查

**Q2: 编码器读取频率50Hz会不会太高导致串口拥塞？**
- 计算：每个read指令10字节，响应10字节，3个舵机 = 60字节/周期
- 1Mbps波特率 = 125KB/s，60字节仅占0.48ms，完全充足
- 但需要考虑延迟累积，设计串行读取（舵机1→2→3）而非并行

**Q3: 舵机突然掉电重启，如何恢复？**
- 设计思路：每次write_speed前先检测舵机是否在线（缓存上次成功时间）
- 如果超过1秒无响应，尝试重新初始化该舵机

🔖 **待补充**: 第3轮设计补充舵机在线检测流程图、串口读取时序图、掉电恢复测试用例

---

### 3.2 硬件接口模块 (OmniHardwareInterface)

#### 3.2.1 设计目标

实现ros2_control的`SystemInterface`，将底层舵机驱动适配到ROS2控制框架。

#### 3.2.2 接口定义

**ros2_control要求实现的方法**:
```python
class OmniHardwareInterface(SystemInterface):
    def on_init(hardware_info):
        # 解析URDF中的硬件参数，初始化ST3215Driver
        pass
    
    def on_configure():
        # 打开串口，扫描舵机
        pass
    
    def on_activate():
        # 使能舵机，进入控制循环
        pass
    
    def on_deactivate():
        # 舵机停止，但保持串口连接
        pass
    
    def read(time, duration):
        # 读取编码器 → 更新joint state
        # 计算里程计 → 发布/wheel/odom
        pass
    
    def write(time, duration):
        # 接收joint velocity commands
        # 调用ST3215Driver写入速度
        pass
```

#### 3.2.3 运动学设计

**逆向运动学** (write方法中使用):
- 输入：`cmd_vel` (vx, vy, ω)
- 输出：三个轮子的角速度 [w1, w2, w3]
- 雅可比矩阵：复用`omni_controller_node.py`中的矩阵（已验证）

**正向运动学** (read方法中使用):
- 输入：三个编码器的位置增量 [Δθ1, Δθ2, Δθ3]
- 输出：机器人位姿增量 (Δx, Δy, Δθ)
- 累积计算：当前位姿 = 上次位姿 + 位姿增量

🔖 **待补充**: 第2轮设计补充运动学矩阵LaTeX公式、代码实现框架、单元测试用例

#### 3.2.4 关键设计问题

**Q1: read()和write()调用频率是多少？由谁控制？**
- 答：由ros2_control框架的`update_rate`参数控制，我们设定50Hz
- 注意：read()和write()在同一个线程中串行调用，总耗时需 < 20ms

**Q2: /wheel/odom的frame_id设置？**
- 设计：`frame_id = "odom"`, `child_frame_id = "base_link"`
- 注意：OmniHardwareInterface只负责发布odom消息，TF由robot_state_publisher发布

**Q3: 如何平滑处理速度突变（避免舵机过载）？**
- 设计思路：在write()中实现速度斜坡，限制加速度
- 参数：最大加速度2.5 m/s² (与Nav2配置一致)

🔖 **待补充**: 第3轮设计补充速度斜坡算法、舵机过载保护逻辑、性能分析（read/write耗时）

---

### 3.3 EKF配置切换模块

#### 3.3.1 设计目标

提供两套独立的EKF配置，通过launch参数自动选择，无需手动修改文件。

#### 3.3.2 配置对比

| 配置项 | 仿真配置 | 真机配置 | 原因 |
|--------|---------|---------|------|
| 轮式里程计yaw | ✅ 使用 | ❌ 禁用 | 真机轮子打滑导致yaw漂移 |
| 轮式里程计vyaw | ✅ 使用 | ❌ 禁用 | 同上 |
| IMU yaw | ❌ 禁用 | ✅ 使用 | 真机依赖IMU角度 |
| IMU vyaw | ❌ 禁用 | ✅ 使用 | 真机依赖IMU角速度 |

#### 3.3.3 launch文件设计

```python
# real_robot_navigation.launch.py
use_sim_time = LaunchConfiguration('use_sim_time', default='false')

# 根据use_sim_time选择配置文件
ekf_config = IfCondition(
    use_sim_time,
    'robot_localization_sim.yaml',
    'robot_localization.yaml'  # 真机配置
)
```

#### 3.3.4 关键设计问题

**Q1: 如果IMU传感器故障，能否降级到轮式里程计？**
- 设计思路：EKF节点检测IMU话题超时（> 1秒），自动降低IMU权重
- 但风险：三轮全向轮yaw误差会快速累积，建议强制要求IMU可用

**Q2: 仿真和真机的里程计协方差矩阵差异有多大？**
- 仿真：position协方差 ~0.001（接近完美）
- 真机：position协方差预估 ~0.1（需实测调整）
- 设计：在配置文件中显式声明协方差，不使用默认值

🔖 **待补充**: 第3轮设计补充协方差矩阵调参实验方案、IMU故障降级测试用例

---

### 3.4 启动文件模块

#### 3.4.1 启动文件清单

| 文件名 | 功能 | 依赖关系 |
|--------|------|---------|
| `real_robot_bringup.launch.py` | 硬件节点+控制器 | 基础层 |
| `real_robot_navigation.launch.py` | Nav2定位模式 | 需要已有地图 |
| `real_robot_slam.launch.py` | RTABMap建图模式 | 用于首次建图 |
| `real_robot_mission.launch.py` | 完整系统+MissionPlanner | 生产环境 |

#### 3.4.2 启动顺序设计

**关键时序**:
```
1. 硬件驱动节点 (ST3215, IMU, Camera)
   ↓ 等待2秒
2. ros2_control controller_manager
   ↓ 等待1秒
3. 加载控制器 (spawner)
   ↓ 等待控制器ACTIVE
4. robot_localization (EKF)
   ↓ 等待/odometry/filtered发布
5. RTABMap / Nav2
   ↓
6. MissionPlanner
```

**设计考虑**:
- 使用`RegisterEventHandler`监听节点状态
- 避免硬编码sleep时间（改用状态监听）
- 失败时提供清晰的错误提示

🔖 **待补充**: 第3轮设计补充launch文件状态机图、启动失败排查指南

---

### 3.5 传感器集成模块

#### 3.5.1 Astra Pro相机

**驱动选择**: 使用官方`astra_camera` ROS2包

**配置要点**:
- 深度格式：16UC1 (毫米单位)
- RGB-D对齐：启用硬件对齐
- 深度范围：设置为0.6m-5m（室内导航够用，减少噪声）

**需验证事项**:
- RGB和Depth的时间戳同步误差 < 10ms
- 深度图在低纹理区域的填充率

🔖 **待补充**: 第3轮设计补充相机标定流程（使用kalibr工具）

#### 3.5.2 IMU传感器

**已确定硬件**: 亚博6轴IMU
- 传感器配置：3轴加速度计 + 3轴陀螺仪
- 通信接口：串口（待确认波特率，常见115200或230400）
- 输出频率：50-100Hz可配置

**驱动开发**:
- 串口通信驱动（参考ST3215Driver的串口封装）
- 发布频率：50Hz (与轮式里程计同步)
- 坐标系：遵循REP-103标准 (x前y左z上)
- 数据解析：根据亚博IMU协议文档实现数据包解析

**静态标定**:
- 机器人静止10分钟，记录陀螺仪输出
- 计算零偏均值，写入配置文件
- 启动时自动减去零偏
亚博6轴IMU驱动开发指南（包含串口协议解析
🔖 **待补充**: 第3轮设计补充IMU驱动开发指南（针对具体型号）、坐标系转换矩阵

---

## 4. 关键设计决策

### 4.1 为什么不用Gazebo的ros2_control插件？

**决策**: 真机使用自定义`OmniHardwareInterface`，而非复用Gazebo的 gazebo_ros2_control

**原因**:
1. Gazebo插件依赖仿真环境，无法直接控制真实硬件
2. 真机需要处理串口通信、编码器溢出、舵机故障等Gazebo不存在的问题
3. 性能考虑：真机直接与硬件交互，无需Gazebo物理引擎开销

**一致性保证**:
- 运动学矩阵完全一致（复用omni_controller_node.py的矩阵）
- 输入输出接口一致（都是ros2_control标准接口）
- 上层节点（Nav2、RTABMap）无感知差异

### 4.2 为什么EKF不融合视觉里程计？

**决策**: 真机EKF只融合轮式里程计+IMU，不使用RTABMap的visual_odom

**原因**:
1. **简化依赖**: visual_odom依赖特征点检测，低纹理环境会失效
2. **计算资源**: 树莓派4B计算有限，同时跑RTABMap+visual_odom会降低帧率
3. **定位精度**: 轮式+IMU已满足室内导航需求（误差 < 5cm/min）

**备选方案**: 如果实测发现轮式里程计打滑严重，可以后期启用visual_odom作为第三数据源

### 4.3 为什么不使用DDS通信优化？

**决策**: 初期使用ROS2默认DDS配置，不做通信层优化

**原因**:
1. 单机器人系统，无跨网络通信需求
2. 话题数量可控（< 50个），不会成为瓶颈
3. 避免过早优化，先验证功能正确性

**后期优化方向**: 如果发现通信延迟，可考虑：
- 使用Cyclone DDS替换Fast-DDS
- 调整QoS策略（Reliable → Best Effort for sensor data）

---

## 5. 风险点与应对策略

### 5.1 硬件风险

| 风险 | 影响 | 应对策略 | 优先级 |
|------|------|---------|--------|
| ST3215舵机通信不稳定 | 机器人抖动/停止 | 1. 降低波特率至115200<br>2. 增加重试次数<br>3. 添加看门狗检测 | 🔴 高 |
| IMU零偏漂移 | 长时间运行yaw累积误差 | 1. 定期IMU标定<br>2. 使用磁力计校正<br>3. 降低IMU权重 | 🟡 中 |
| 相机USB断连 | SLAM失败 | 1. USB hub供电稳定性检查<br>2. 自动重连机制<br>3. 降级到盲导航 | 🟡 中 |
| 树莓派性能不足 | 控制延迟/掉帧 | 1. 关闭桌面环境<br>2. 限制RTABMap帧率至15fps<br>3. 使用CPU超频 | 🟢 低 |

### 5.2 软件风险

| 风险 | 影响 | 应对策略 | 优先级 |
|------|------|---------|--------|
| ros2_control节点崩溃 | 失去控制 | 1. 添加硬件看门狗<br>2. 自动重启机制<br>3. 紧急停止按钮 | 🔴 高 |
| 编码器读取超时 | 里程计数据缺失 | 1. 使用上次有效值<br>2. EKF自动降低odom权重<br>3. 发布诊断信息 | 🟡 中 |
| EKF发散 | 位姿估计错误 | 1. 调整协方差矩阵<br>2. 降低odom/IMU权重<br>3. 重置EKF状态 | 🟡 中 |
| launch启动顺序错误 | 节点无法初始化 | 1. 使用状态监听<br>2. 详细日志输出<br>3. 启动脚本自检 | 🟢 低 |

### 5.3 环境风险

| 风险 | 影响 | 应对策略 |
|------|------|---------|
| 光滑地面打滑 | 里程计误差增大 | 增加IMU权重，考虑使用麦克纳姆轮 |
| 低纹理环境 | RTABMap定位失败 | 添加人工特征点（二维码/棋盘格） |
| 强光/背光 | 深度图噪声 | 调整相机曝光参数，添加遮光罩 |

---

## 6. 测试策略

### 6.1 单元测试

**ST3215Driver测试**:
- 模拟串口测试（使用pyserial-test）
- 指令包构造正确性测试
- 校验和计算测试
- 错误处理测试（超时、错误响应）

**OmniHardwareInterface测试**:
- 运动学正逆运算精度测试（与MATLAB结果对比）
- 编码器溢出处理测试
- 速度斜坡算法测试

🔖 **待补充**: 第4轮设计补充pytest测试框架配置、测试用例模板

### 6.2 集成测试

**阶段1: 硬件连通性测试**
- 目标：验证所有硬件能正常通信
- 工具：ros2 topic echo, rqt_graph
- 预期结果：所有话题正常发布，无错误日志

**阶段2: 运动学校准测试**
- 目标：验证运动学参数准确性
- 方法：直线1米、旋转360°、对角线移动
- 预期结果：误差 < 5cm (位置), < 5° (角度)

**阶段3: 导航功能测试**
- 目标：验证导航栈工作正常
- 方法：2米路点导航，避障测试
- 预期结果：成功到达目标，无碰撞

**阶段4: 长时间稳定性测试**
- 目标：验证系统可靠性
- 方法：连续巡航2小时
- 预期结果：无崩溃，里程计漂移 < 10cm

🔖 **待补充**: 第4轮设计补充集成测试清单表格、测试报告模板

### 6.3 性能测试

**关键指标**:
- 控制周期延迟：read() + write() < 20ms (50Hz)
- 编码器读取延迟：< 10ms
- 话题发布频率：/wheel/odom 稳定50Hz
- CPU占用率：< 70% (留30%余量)

🔖 **待补充**: 第4轮设计补充性能测试工具（ros2_tracing）、性能基准值表格

---

## 7. 与仿真环境的差异分析

### 7.1 相同部分（无需修改）

✅ **导航层**:
- Nav2配置文件完全复用
- DWB控制器参数不变
- Costmap配置不变

✅ **感知层**:
- RTABMap配置基本复用（仅调整帧率）
- 深度图转激光扫描逻辑不变

✅ **任务层**:
- MissionPlanner完全复用
- ExplorationHandler、PatrolHandler不变
- WaypointRecorder不变

### 7.2 不同部分（需要适配）

❌ **硬件接口**:
| 组件 | 仿真 | 真机 |
|------|------|------|
| 电机控制 | Gazebo插件 | ST3215Driver |
| 里程计 | 完美无噪声 | 有打滑和漂移 |
| IMU | 角速度微弱 | 角速度准确 |
| 相机 | 虚拟场景 | 真实RGB-D |

❌ **EKF配置**:
- 仿真：轮式里程计提供yaw
- 真机：IMU提供yaw

❌ **启动参数**:
- 仿真：use_sim_time=true
- 真机：use_sim_time=false

### 7.3 迁移清单

**代码迁移**:
- ✅ 导航层：0行修改
- ✅ 感知层：0行修改
- 🆕 硬件层：~500行新增代码

**配置迁移**:
- 🔄 EKF配置：1个新文件
- 🔄 启动文件：3个新文件
- ✅ Nav2配置：复用

**预计迁移工作量**: 1周开发 + 1周测试

---

## 8. 后续补充计划

### 第2轮补充 (代码实现细节)
- [ ] ST3215Driver完整类设计（含指令码表）
- [ ] OmniHardwareInterface运动学公式推导
- [ ] 单位转换公式和查表
- [ ] 异常处理流程图

### 第3轮补充 (集成细节)
- [ ] launch文件启动状态机设计
- [ ] 传感器标定详细步骤
- [ ] IMU坐标系转换矩阵
- [ ] 时间戳同步验证方案

### 第4轮补充 (测试细节)
- [ ] pytest测试框架配置
- [ ] 测试用例模板（单元/集成/性能）
- [ ] 测试报告格式
- [ ] CI/CD集成（GitHub Actions）

### 第5轮补充 (部署细节)
- [ ] 树莓派系统配置脚本
- [ ] 依赖安装指南（ROS2 + 驱动）
- [ ] 开机自启动配置
- [ ] 远程调试工具配置（SSH + VNC）

---

## 9. 设计审核清单

**请审核以下设计决策**:

### 架构设计
- [ ] 分层架构是否合理？是否有遗漏的层级？
- [ ] 数据流设计是否清晰？控制流和反馈流是否闭环？
- [ ] 与仿真环境的复用度是否足够高？

### 模块设计
- [ ] ST3215Driver的功能是否完整？是否需要添加其他功能？
- [ ] OmniHardwareInterface是否符合ros2_control规范？
- [ ] EKF配置切换方案是否合理？是否需要支持动态切换？
- [ ] 启动文件的数量和划分是否合理？

### 风险管理
- [ ] 风险清单是否完整？是否有遗漏的高风险项？
- [ ] 应对策略是否可行？是否需要补充备选方案？

### 测试策略
- [ ] 测试覆盖度是否足够？是否有遗漏的测试场景？
- [ ] 验收标准是否可量化？是否需要放宽/收紧？

### 开发计划
- [ ] 开发顺序是否合理？是否存在依赖冲突？
- [ ] 时间估算是否合理？是否需要增加缓冲时间？

---

**设计文档状态**: 📝 第一版草稿，等待审核  
**下一步**: 根据审核意见修改设计，然后进入第2轮细节补充  
**预期审核时间**: 1-2小时  
**审核通过标准**: 无重大架构缺陷，风险可控，开发计划可行
