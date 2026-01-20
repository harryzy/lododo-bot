# P3阶段配置核对清单

**目的**: 确保P3传感器集成符合统一配置管理原则  
**创建日期**: 2026-01-20  
**设计原则**: 单一配置源 + 标定数据分离

---

## ✅ 配置管理原则

### 原则1: 统一配置文件 (hardware_config.yaml)

**适用范围**: 所有运行时参数
- ✅ 硬件设备参数（串口、波特率、设备ID）
- ✅ 传感器参数（分辨率、帧率、发布频率）
- ✅ 坐标系配置（frame_id、mounting_rotation）
- ✅ 滤波参数（窗口大小、滤波系数）
- ✅ 话题名称（topic names）

**位置**: `src/bot_hardware/config/hardware_config.yaml`

---

### 原则2: 标定数据分离 (calibration/)

**适用范围**: 仅标定工具生成的数据文件
- ✅ IMU零偏标定 → `calibration/imu_bias.yaml`
- ✅ 相机内参标定 → `calibration/camera_info.yaml`
- ✅ 舵机参数标定 → `calibration/servo_params.yaml`

**位置**: `~/lododo_bot/calibration/`

**特点**:
- 由专门标定工具生成
- 不包含运行时参数
- 通过hardware_config.yaml中的`paths.*_calibration_file`引用

---

## 📋 P3配置核对清单

### ❌ 禁止创建的文件（违反设计原则）

- [ ] ❌ `config/imu_config.yaml` - 应使用hardware_config.yaml
- [ ] ❌ `config/camera_config.yaml` - 应使用hardware_config.yaml
- [ ] ❌ 任何单独的传感器配置文件
- [ ] ❌ 在config/目录下存放标定数据

**违反后果**: 
- 配置分散，维护困难
- 违反单一配置源原则
- 代码审查不通过

---

### ✅ hardware_config.yaml检查项

#### IMU配置 (line 12-118)

- [ ] **串口配置** (line 15-17)
  ```yaml
  serial:
    imu_port: '/dev/ybimu'      # ✅ 已配置
    imu_baudrate: 115200        # ✅ 已配置
    imu_timeout: 0.02           # ✅ 已配置
  ```

- [ ] **基本参数** (line 73-76)
  ```yaml
  imu:
    model: 'yabo_6axis'         # ✅ 已配置
    publish_rate: 50            # ✅ 已配置 (Hz)
    frame_id: 'imu_link'        # ✅ 已配置
  ```

- [ ] **mounting_rotation** (line 83-86)
  ```yaml
  mounting_rotation:
    roll: 0.0                   # ✅ 已配置 (需实测调整)
    pitch: 0.0                  # ✅ 已配置 (需实测调整)
    yaw: 0.0                    # ✅ 已配置 (需实测调整)
  ```

- [ ] **标定配置** (line 88-100)
  ```yaml
  calibration:
    enabled: true               # ✅ 已配置
    calibration_file: 'calibration/imu_bias.yaml'  # ✅ 相对路径
    gyro_bias_x: 0.0            # ⚠️ 待标定后填写
    gyro_bias_y: 0.0            # ⚠️ 待标定后填写
    gyro_bias_z: 0.0            # ⚠️ 待标定后填写
  ```

- [ ] **滤波参数** (line 113-118)
  ```yaml
  filter:
    enable_dynamic_bias: true   # ✅ 已配置
    low_pass_alpha: 0.2         # ✅ 已配置
    median_window_size: 5       # ✅ 已配置
    complementary_alpha: 0.98   # ✅ 已配置
  ```

#### 相机配置 (line 120-145)

- [ ] **RGB参数** (line 125-128)
  ```yaml
  camera:
    rgb:
      width: 640                # ✅ 已配置
      height: 480               # ✅ 已配置
      fps: 30                   # ✅ 已配置
  ```

- [ ] **Depth参数** (line 130-134)
  ```yaml
    depth:
      width: 640                # ✅ 已配置
      height: 480               # ✅ 已配置
      fps: 30                   # ✅ 已配置
      min_range: 0.6            # ✅ 已配置 (m)
      max_range: 8.0            # ✅ 已配置 (m)
  ```

- [ ] **对齐配置** (line 139)
  ```yaml
    enable_alignment: true      # ✅ 已配置 (硬件RGB-D对齐)
  ```

- [ ] **话题名称** (line 141-145)
  ```yaml
    topics:
      rgb_image: '/camera/color/image_raw'        # ✅ 已配置
      depth_image: '/camera/depth/image_raw'      # ✅ 已配置
      rgb_camera_info: '/camera/color/camera_info'    # ✅ 已配置
      depth_camera_info: '/camera/depth/camera_info'  # ✅ 已配置
  ```

#### 路径配置 (line 310-323)

- [ ] **标定文件路径** (line 320-323)
  ```yaml
  paths:
    calibration_directory: 'calibration'            # ✅ 相对路径
    imu_calibration_file: 'calibration/imu_bias.yaml'       # ✅ 相对路径
    camera_calibration_file: 'calibration/camera_info.yaml' # ✅ 相对路径
  ```

**验证命令**:
```bash
# 检查所有路径都是相对路径
grep -E "^  .*_file:|^  .*_directory:" config/hardware_config.yaml | grep -v "^#"

# 确认没有绝对路径或~
grep -E "(/home|/root|~)" config/hardware_config.yaml
# 预期：无输出或仅在注释中
```

---

### ✅ 标定文件准备

#### 创建标定目录

```bash
mkdir -p ~/lododo_bot/calibration
```

#### IMU标定文件模板 (calibration/imu_bias.yaml)

```yaml
# IMU零偏标定数据
# 生成时间: 2026-01-XX
# 标定时长: 600秒（10分钟）
# 标定条件: 机器人静止，水平放置

imu_bias:
  gyro:
    x: 0.001234    # rad/s (待标定工具生成)
    y: -0.000567   # rad/s
    z: 0.000891    # rad/s
  
  accel:
    x: 0.0234      # m/s² (待标定工具生成)
    y: -0.0156     # m/s²
    z: 0.0089      # m/s² (注意：z轴应接近0，因为重力已在mounting_rotation中补偿)

calibration_quality:
  gyro_std_dev: 0.0005   # rad/s
  accel_std_dev: 0.01    # m/s²
  gravity_error: 0.05    # m/s²
  status: "GOOD"         # GOOD | WARN | FAIL
```

**注意**: 此文件由IMU标定工具自动生成，不要手动创建初始值！

#### 相机标定文件模板 (calibration/camera_info.yaml)

```yaml
# 相机内参标定数据
# 生成时间: 2026-01-XX
# 标定板: 8x6棋盘格，25mm方格
# 重投影误差: 0.XX像素

image_width: 640
image_height: 480

camera_name: astra_pro_color

camera_matrix:
  rows: 3
  cols: 3
  data: [fx, 0, cx,
         0, fy, cy,
         0, 0, 1]

distortion_model: plumb_bob

distortion_coefficients:
  rows: 1
  cols: 5
  data: [k1, k2, p1, p2, k3]

rectification_matrix:
  rows: 3
  cols: 3
  data: [1, 0, 0,
         0, 1, 0,
         0, 0, 1]

projection_matrix:
  rows: 3
  cols: 4
  data: [fx, 0, cx, 0,
         0, fy, cy, 0,
         0, 0, 1, 0]
```

**注意**: 此文件由camera_calibration工具生成，不要手动编辑！

---

### ✅ Launch文件配置核对

#### imu_bringup.launch.py

- [ ] 从hardware_config.yaml读取参数
- [ ] 不创建单独的imu_config.yaml
- [ ] ybimu_driver读取`serial.imu_*`参数
- [ ] imu_filter_node读取`imu.*`参数

```python
# ✅ 正确方式
hardware_config = os.path.join(config_dir, 'hardware_config.yaml')

Node(
    package='bot_hardware',
    executable='ybimu_driver',
    parameters=[hardware_config]  # 统一配置文件
)
```

```python
# ❌ 错误方式
imu_config = os.path.join(config_dir, 'imu_config.yaml')  # 不应创建

Node(
    package='bot_hardware',
    executable='ybimu_driver',
    parameters=[imu_config]  # 违反单一配置源原则
)
```

#### camera_bringup.launch.py

- [ ] 从hardware_config.yaml读取运行参数
- [ ] 从calibration/目录读取标定文件
- [ ] 不创建单独的camera_config.yaml

```python
# ✅ 正确方式
hardware_config = os.path.join(config_dir, 'hardware_config.yaml')
camera_info_file = os.path.join(
    os.path.expanduser('~'), 'lododo_bot', 'calibration', 'camera_info.yaml'
)

Node(
    package='astra_camera',
    executable='astra_camera_node',
    parameters=[
        hardware_config,  # 运行参数
        {'camera_info_url': f'file://{camera_info_file}'}  # 标定数据
    ]
)
```

---

## 🧪 验证测试

### Test 1: 配置文件结构验证

```bash
cd ~/lododo_bot/src/bot_hardware

# 检查是否存在不应创建的文件
if [ -f config/imu_config.yaml ]; then
    echo "❌ FAIL: config/imu_config.yaml should not exist"
else
    echo "✅ PASS: No imu_config.yaml"
fi

if [ -f config/camera_config.yaml ]; then
    echo "❌ FAIL: config/camera_config.yaml should not exist"
else
    echo "✅ PASS: No camera_config.yaml"
fi

# 检查标定目录是否存在
if [ -d ~/lododo_bot/calibration ]; then
    echo "✅ PASS: Calibration directory exists"
else
    echo "⚠️ WARN: Create calibration directory: mkdir -p ~/lododo_bot/calibration"
fi
```

### Test 2: hardware_config.yaml参数完整性

```bash
# 检查IMU必需参数
grep -q "imu_port:" config/hardware_config.yaml && echo "✅ imu_port" || echo "❌ Missing imu_port"
grep -q "publish_rate:" config/hardware_config.yaml && echo "✅ publish_rate" || echo "❌ Missing publish_rate"
grep -q "mounting_rotation:" config/hardware_config.yaml && echo "✅ mounting_rotation" || echo "❌ Missing mounting_rotation"

# 检查相机必需参数
grep -q "camera:" config/hardware_config.yaml && echo "✅ camera section" || echo "❌ Missing camera section"
grep -q "enable_alignment:" config/hardware_config.yaml && echo "✅ enable_alignment" || echo "❌ Missing enable_alignment"
```

### Test 3: 路径配置验证

```bash
# 运行PathManager验证工具（如果已实现）
python3 -c "
import yaml
with open('config/hardware_config.yaml') as f:
    config = yaml.safe_load(f)

# 检查所有路径都是相对路径
paths = config.get('paths', {})
for key, path in paths.items():
    if path.startswith('/') or path.startswith('~'):
        print(f'❌ FAIL: {key} uses absolute path: {path}')
    else:
        print(f'✅ PASS: {key} uses relative path: {path}')
"
```

---

## 📝 P3实施注意事项

### 开发顺序建议

1. **首先**: 确认hardware_config.yaml包含所有需要的参数 ✅ (已完成)
2. **然后**: 创建calibration/目录结构
3. **接着**: 编写ybimu_driver适配代码（读取hardware_config.yaml）
4. **同时**: 编写imu_filter_node（读取hardware_config.yaml）
5. **之后**: 创建launch文件（使用统一配置）
6. **最后**: 执行标定流程，生成标定文件

### 代码审查检查点

- [ ] 所有节点从hardware_config.yaml读取参数
- [ ] 没有创建imu_config.yaml或camera_config.yaml
- [ ] 标定文件存放在calibration/目录
- [ ] launch文件通过参数传递hardware_config路径
- [ ] 没有硬编码任何设备路径或参数

### 常见错误预防

**错误1**: 为每个传感器创建单独配置文件
```bash
# ❌ 错误
config/
  ├── hardware_config.yaml
  ├── imu_config.yaml          # 不应创建
  └── camera_config.yaml       # 不应创建
```

**错误2**: 将标定数据放入config/目录
```bash
# ❌ 错误
config/
  ├── hardware_config.yaml
  ├── imu_bias.yaml            # 应在calibration/
  └── camera_info.yaml         # 应在calibration/
```

**正确结构**:
```bash
# ✅ 正确
config/
  └── hardware_config.yaml     # 唯一配置文件

calibration/
  ├── imu_bias.yaml            # IMU标定数据
  └── camera_info.yaml         # 相机标定数据
```

---

## 🎯 完成标准

P3阶段配置管理验收：

- [ ] hardware_config.yaml包含所有IMU运行参数
- [ ] hardware_config.yaml包含所有相机运行参数
- [ ] 不存在imu_config.yaml
- [ ] 不存在camera_config.yaml
- [ ] calibration/目录结构正确
- [ ] launch文件使用统一配置文件
- [ ] 所有节点代码从hardware_config.yaml读取参数
- [ ] 路径配置全部使用相对路径

**验收命令**:
```bash
cd ~/lododo_bot/src/bot_hardware
bash docs/P3_CONFIG_CHECKLIST.md  # 运行本文档中的验证测试
```

---

**文档版本**: v1.0  
**最后更新**: 2026-01-20  
**参考文档**: HARDWARE_DEPLOYMENT_DESIGN.md § 1.4.1 (行500-841)
