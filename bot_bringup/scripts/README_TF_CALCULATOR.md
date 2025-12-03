# TF坐标转换计算工具

## 功能说明

`tf_calculator.py` 是一个用于计算URDF joint origin值的工具。当你需要将某个坐标系放置在base_link中的特定位置时，这个工具可以自动计算出相对于父坐标系的joint origin参数。

## 使用场景

- 调整相机、激光雷达等传感器的安装位置
- 需要精确控制某个link在base_link中的位置和姿态
- 避免手动计算复杂的坐标变换

## 使用方法

### 1. 默认模式（camera_frame示例）

直接运行脚本查看camera_frame的计算示例：

```bash
python3 src/bot_bringup/scripts/tf_calculator.py
```

### 2. 交互模式

交互式输入自定义参数：

```bash
python3 src/bot_bringup/scripts/tf_calculator.py -i
```

然后按提示输入：
1. 目标在base_link中的期望位置和姿态（xyz, rpy）
2. 父坐标系在base_link中的位置和姿态（xyz, rpy）

工具会自动计算出joint origin值，并提供URDF格式输出。

## 工作原理

工具使用齐次变换矩阵进行坐标系变换：

```
T_parent_child = inv(T_base_parent) × T_base_child
```

其中：
- `T_base_parent`: 父坐标系相对base_link的变换
- `T_base_child`: 目标坐标系相对base_link的期望变换
- `T_parent_child`: 计算结果，即joint origin值

## 输入参数说明

### 位置（xyz）
- 单位：米（m）
- 示例：`[0.146, 0.000, 0.038]`

### 姿态（rpy）
- 单位：度（°）- 交互模式
- 单位：弧度（rad）- Python代码
- 顺序：Roll-Pitch-Yaw（绕X-Y-Z轴依次旋转）
- 示例：`[90, 0, 0]` 表示绕X轴旋转90度

## 输出格式

工具会输出：
1. 输入参数的可读格式
2. 计算结果（xyz和rpy）
3. URDF格式的`<origin>`标签（包含xacro常量版本）
4. 验证结果（正向计算验证位置误差）

## 获取父坐标系TF的方法

### 方法1：从运行的ROS2系统查询

```bash
ros2 run tf2_ros tf2_echo base_link <parent_link_name>
```

### 方法2：从URDF手动计算

参考 `TF_REFERENCE.md` 文档中的坐标变换链。

## 示例：camera_frame计算

```python
# 期望camera_frame在base_link中的位置
target_in_base = {
    'xyz': [0.146, 0.000, 0.038],  # 向前146mm, Y=0, 高度38mm
    'rpy': [np.pi/2, 0, 0]  # Z轴指向前方
}

# Camera-Model-v3在base_link中的TF
parent_xyz = [0.106, 0.014, 0.038]
parent_rpy = [np.pi, np.deg2rad(-80), np.pi]

# 计算结果
joint_xyz = [-0.006946, -0.014000, -0.039392]
joint_rpy = [-np.pi/2, np.deg2rad(80), np.pi]
```

## 常用的xacro常量

```xml
${M_PI}     = 3.14159...  (180°)
${M_PI_2}   = 1.57079...  (90°)
${M_PI_3}   = 1.04719...  (60°)
${DEG_80}   = 1.39626...  (80°)
```

## 注意事项

1. **单位一致性**：确保所有长度使用米（m），角度使用弧度或度（根据模式）
2. **TF准确性**：父坐标系的TF必须准确，建议从实际运行系统查询
3. **坐标系约定**：ROS2使用右手坐标系，RPY是内旋欧拉角
4. **验证结果**：工具会自动验证计算结果，确保误差在可接受范围内

## 依赖

- Python 3
- numpy
- scipy

安装依赖：
```bash
pip3 install numpy scipy
```

## 相关文档

- [TF_REFERENCE.md](../../TF_REFERENCE.md) - TF变换参考文档
- [ROS2 TF2教程](https://docs.ros.org/en/humble/Tutorials/Intermediate/Tf2/Tf2-Main.html)
