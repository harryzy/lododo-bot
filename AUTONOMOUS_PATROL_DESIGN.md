# LeKiwi机器人自主巡航系统设计方案

**项目名称**: LeKiwi自主巡航系统 (Autonomous Patrol System)  
**版本**: v1.0  
**创建日期**: 2025年12月22日  
**依赖**: ROS2 Humble + Nav2 + 现有探索建图系统  
**设计阶段**: Phase 4 - 高级功能开发

---

## 目录

1. [系统概述](#1-系统概述)
2. [架构设计](#2-架构设计)
3. [核心组件详细设计](#3-核心组件详细设计)
4. [文件结构](#4-文件结构)
5. [数据结构与接口](#5-数据结构与接口)
6. [实施计划](#6-实施计划)
7. [测试方案](#7-测试方案)
8. [未来扩展](#8-未来扩展)

---

## 1. 系统概述

### 1.1 设计目标

在现有探索建图系统 (`exploration_mapper.py`) 基础上，构建一个完整的自主导航与任务管理系统，支持：

#### 核心功能
- ✅ **任务类型管理**: 探索建图（临时/保存）、多模式巡航（自主/配置/录制）
- ✅ **路点管理**: 添加、删除、加载、保存、录制路点
- ✅ **多种巡航模式**: 循环、往返、单次、随机、优先级、覆盖
- ✅ **地图库管理**: 地图持久化、版本控制、加载切换
- ✅ **任务调度**: 任务优先级、队列管理、状态机控制
- ✅ **任务持久化**: 保存任务配置、恢复中断任务、历史记录
- ✅ **异常恢复**: 导航失败、障碍物、电量不足等异常处理
- ✅ **语音集成**: 与现有语音控制系统深度集成
- ✅ **Web控制**: 远程监控、可视化、任务管理界面
- ✅ **充电管理**: 自动返回充电站（可选）
- ✅ **多楼层支持**: 电梯导航、楼层切换（未来扩展）

### 1.2 任务类型定义

#### 1.2.1 探索建图任务 (Exploration Mapping)

| 任务类型 | 任务ID | 说明 | 地图处理 |
|---------|--------|------|----------|
| **临时探索** | `exploration_temp` | 快速探索，不保存地图 | 仅内存，关闭即丢失 |
| **持久化探索** | `exploration_save` | 探索并保存地图到文件 | 自动保存到地图库 |
| **增量建图** | `exploration_incremental` | 在已有地图基础上扩展 | 合并到现有地图 |
| **重建地图** | `exploration_rebuild` | 清空重新建图 | 覆盖旧地图 |

**参数配置**:
```yaml
exploration:
  map_name: "office_floor1"         # 地图名称
  save_map: true                    # 是否保存
  completion_threshold: 0.90        # 完成度阈值
  max_exploration_time: 1800        # 最大探索时间(秒)
  auto_save_interval: 300           # 自动保存间隔(秒)
```

#### 1.2.2 巡航任务 (Patrol Mission)

| 任务类型 | 任务ID | 说明 | 路点来源 |
|---------|--------|------|----------|
| **自主巡航** | `patrol_autonomous` | 基于地图自动生成路点 | 算法生成 |
| **配置文件巡航** | `patrol_config` | 使用YAML配置的路点 | 配置文件 |
| **录制路点巡航** | `patrol_recorded` | 使用实时录制的路点 | 人工录制 |
| **混合巡航** | `patrol_hybrid` | 结合配置+实时调整 | 配置+动态 |

**巡航模式**:
- `loop`: 循环模式 (0→1→2→0→...)
- `ping_pong`: 往返模式 (0→1→2→1→0→...)
- `once`: 单次模式 (0→1→2→结束)
- `random`: 随机模式 (随机选择路点)
- `priority`: 优先级模式 (按路点优先级)
- `coverage`: 覆盖模式 (最大化区域覆盖)

**参数配置**:
```yaml
patrol:
  waypoint_source: "config"         # config, recorded, autonomous
  waypoint_file: "waypoints.yaml"  # 配置文件路径
  patrol_mode: "loop"              # 巡航模式
  max_loops: -1                    # 循环次数(-1无限)
  dwell_time: 2.0                  # 停留时间
  speed_profile: "normal"          # slow, normal, fast
```

#### 1.2.3 导航任务 (Navigation Task)

| 任务类型 | 任务ID | 说明 |
|---------|--------|------|
| **单点导航** | `nav_single` | 导航到指定点 |
| **跟随导航** | `nav_follow` | 跟随目标移动 |
| **返回基地** | `nav_return_home` | 返回起点/充电站 |
| **紧急停止** | `nav_emergency_stop` | 立即停止所有运动 |

#### 1.2.4 混合任务 (Composite Task)

| 任务类型 | 任务ID | 说明 |
|---------|--------|------|
| **探索后巡航** | `explore_then_patrol` | 先探索建图，后自动巡航 |
| **定时巡航** | `scheduled_patrol` | 按时间表执行巡航 |
| **区域清扫** | `area_cleaning` | 覆盖式区域遍历 |

### 1.3 功能包架构与职责划分

#### 1.3.1 功能包组织

```
┌─────────────────────────────────────────────────────────────────┐
│                    bot_cmd_interface (命令接口层)                │
│  ┌───────────────────────────────────────────────────────────┐  │
│  │  统一命令接口 - 异步转同步适配器                          │  │
│  │  - 订阅Topic命令 (语音、Web、App、手柄等)                  │  │
│  │  - 转换为ROS2服务调用                                      │  │
│  │  - 命令队列和去重                                          │  │
│  │  - 多终端支持                                             │  │
│  └───────────────────────────────────────────────────────────┘  │
└──────────────────────┬──────────────────────────────────────────┘
                       │ 服务调用 (同步、有状态)
                       ⬇️
┌─────────────────────────────────────────────────────────────────┐
│                bot_navigation (导航与任务管理核心)               │
│  ┌───────────────────────────────────────────────────────────┐  │
│  │  MissionPlanner (任务规划器)                              │  │
│  │  ├─ TaskManager (任务管理器)                              │  │
│  │  │   - 任务持久化、队列、状态机                           │  │
│  │  ├─ ExplorationManager (探索管理器)                       │  │
│  │  │   - Frontier探索、地图构建                            │  │
│  │  ├─ PatrolManager (巡航管理器)                            │  │
│  │  │   - 路点巡航、多模式支持                              │  │
│  │  └─ NavigationExecutor (导航执行器基类)                   │  │
│  │      - Nav2接口封装、状态管理                             │  │
│  └───────────────────────────────────────────────────────────┘  │
│  ┌───────────────────────────────────────────────────────────┐  │
│  │  WaypointManager (路点管理)                               │  │
│  │  WaypointRecorder (路点录制)                              │  │
│  └───────────────────────────────────────────────────────────┘  │
└──────────────────────┬──────────────────────────────────────────┘
                       │
┌──────────────────────┴──────────────────────────────────────────┐
│                    bot_slam (SLAM与地图管理)                     │
│  ┌───────────────────────────────────────────────────────────┐  │
│  │  MapLibraryManager (地图库管理器)                         │  │
│  │  - 地图持久化存储 (~/.ros/lekiwi_maps/)                   │  │
│  │  - 地图版本控制                                           │  │
│  │  - 地图合并、导入导出                                      │  │
│  │  - RTABMap/SLAM Toolbox集成                               │  │
│  └───────────────────────────────────────────────────────────┘  │
│  ┌───────────────────────────────────────────────────────────┐  │
│  │  SLAM配置和启动文件                                        │  │
│  │  - rtabmap_config.yaml                                    │  │
│  │  - slam_toolbox_config.yaml                               │  │
│  └───────────────────────────────────────────────────────────┘  │
└─────────────────────────────────────────────────────────────────┘
                       │
                       ⬇️
┌─────────────────────────────────────────────────────────────────┐
│                       共享基础设施                               │
│  - Nav2导航栈                                                   │
│  - TF2变换系统                                                  │
│  - ROS2服务和Action接口                                          │
└─────────────────────────────────────────────────────────────────┘
```

#### 1.3.2 功能包职责

| 功能包 | 职责 | 关键组件 |
|--------|------|---------|
| **bot_navigation** | 导航、任务管理、路点管理 | MissionPlanner, TaskManager, PatrolManager, ExplorationManager |
| **bot_slam** | SLAM、地图库管理 | MapLibraryManager, SLAM配置 |
| **bot_cmd_interface** | 统一命令接口、多终端适配 | CommandAdapter, VoiceHandler, WebHandler |
| **bot_voice** | 语音识别和合成（已有） | - |
| **bot_control** | 底层运动控制（已有） | omni_controller |
| **bot_perception** | 传感器处理（已有） | depth_to_laserscan |

#### 1.3.3 接口设计哲学

**核心原则：异步接收 + 同步执行**

```
┌──────────────────────────────────────────────────────────────┐
│  外部终端 (多个异步输入源)                                     │
│  ┌────────┐  ┌────────┐  ┌────────┐  ┌────────┐             │
│  │ 语音   │  │  Web   │  │  App   │  │ 手柄   │             │
│  └────┬───┘  └───┬────┘  └───┬────┘  └───┬────┘             │
│       │          │           │           │                   │
│       └──────────┴───────────┴───────────┘                   │
│                      │ Topic 异步命令                         │
└──────────────────────┼──────────────────────────────────────┘
                       ⬇️
┌──────────────────────────────────────────────────────────────┐
│  bot_cmd_interface (适配器层)                                 │
│  ┌────────────────────────────────────────────────────────┐  │
│  │  CommandQueue (命令队列)                                │  │
│  │  - 接收异步Topic命令                                     │  │
│  │  - 命令去重和优先级排序                                  │  │
│  │  - 并发控制（同一时间只执行一个任务命令）                 │  │
│  └────────────────────────────────────────────────────────┘  │
│                      │ 转换                                   │
│  ┌────────────────────────────────────────────────────────┐  │
│  │  ServiceAdapter (服务适配器)                            │  │
│  │  - Topic命令 → ROS2服务调用                             │  │
│  │  - 等待服务响应                                         │  │
│  │  - 返回结果通知                                         │  │
│  └────────────────────────────────────────────────────────┘  │
└──────────────────────┼──────────────────────────────────────┘
                       │ Service 同步调用
                       ⬇️
┌──────────────────────────────────────────────────────────────┐
│  bot_navigation (核心业务逻辑)                                │
│  - 服务接口（有状态、事务性、并发安全）                        │
│  - 任务状态机管理                                             │
│  - 导航目标生命周期控制                                        │
└──────────────────────────────────────────────────────────────┘
```

**优点**:
1. ✅ **状态一致性**: 核心服务保持同步调用，状态管理清晰
2. ✅ **多终端支持**: 多个客户端可以同时发送命令
3. ✅ **解耦合**: 添加新控制方式只需修改 `bot_cmd_interface`
4. ✅ **命令队列**: 自动处理并发请求，避免冲突
5. ✅ **统一接口**: 所有外部输入统一格式

---

### 1.4 设计原则

- **模块化**: 组件解耦，易于测试和维护
- **可扩展**: 支持未来添加新的巡航模式和功能
- **复用性**: 提取共用逻辑为基类，避免重复代码
- **事务性**: 严格的状态管理，避免并发冲突
- **容错性**: 完善的异常处理和恢复机制

---

## 2. 架构设计

### 2.1 分层架构

```
┌───────────────────────────────────────────────────────────────────┐
│                      应用层 (Application Layer)                     │
│  ┌─────────────────────────────────────────────────────────────┐  │
│  │         Mission Planner (任务规划器)                         │  │
│  │  - 任务类型选择 (探索/巡航)                                   │  │
│  │  - 任务优先级管理                                             │  │
│  │  - 任务监控和异常处理                                         │  │
│  │  - 服务接口暴露                                               │  │
│  └────────────────┬──────────────────┬─────────────────────────┘  │
└───────────────────┼──────────────────┼────────────────────────────┘
                    │                  │
┌───────────────────┼──────────────────┼────────────────────────────┐
│                   │  任务层 (Task Layer)  │                         │
│  ┌────────────────▼─────────┐  ┌────▼──────────────────────────┐  │
│  │   Patrol Manager         │  │  Exploration Manager          │  │
│  │   (巡航管理器)            │  │  (探索管理器-已实现)           │  │
│  │                          │  │                               │  │
│  │  - 路点管理               │  │  - Frontier发现               │  │
│  │  - 巡航路径生成           │  │  - 自主探索                   │  │
│  │  - 循环/往返/随机模式     │  │  - 地图构建                   │  │
│  │  - 停留时间控制           │  │  - 完成度检测                 │  │
│  │  - 路点录制功能           │  │                               │  │
│  └──────────────┬───────────┘  └───────────┬───────────────────┘  │
└─────────────────┼──────────────────────────┼────────────────────┘
                  │                          │
┌─────────────────┴──────────────────────────┴────────────────────┐
│                     执行层 (Execution Layer)                      │
│  ┌──────────────────────────────────────────────────────────┐   │
│  │         Navigation Executor (导航执行器基类)              │   │
│  │                                                           │   │
│  │  - Nav2 ActionClient封装                                  │   │
│  │  - 事务性状态管理 (IDLE/WAITING/EXECUTING/CANCELING)     │   │
│  │  - 目标超时和取消机制                                      │   │
│  │  - TF变换和位姿获取                                        │   │
│  │  - 目标发送和结果处理                                      │   │
│  └──────────────────────┬───────────────────────────────────┘   │
└─────────────────────────┼───────────────────────────────────────┘
                          │
┌─────────────────────────┴───────────────────────────────────────┐
│                     基础层 (Infrastructure Layer)                │
│  ┌─────────────┐  ┌─────────────┐  ┌──────────────────────┐    │
│  │   Nav2      │  │  TF2_ROS    │  │  ROS2 Services       │    │
│  │  - navigate │  │  - Buffer   │  │  - Trigger           │    │
│  │  - spin     │  │  - Listener │  │  - SetBool           │    │
│  └─────────────┘  └─────────────┘  └──────────────────────┘    │
└─────────────────────────────────────────────────────────────────┘
```

### 2.2 状态机设计

#### 2.2.1 导航执行器状态机

```
                    send_goal()
    ┌────────┐   ─────────────►   ┌─────────────────┐
    │  IDLE  │                    │ WAITING_ACCEPT  │
    └────────┘   ◄─────────────   └─────────────────┘
        ▲         timeout/reject           │
        │                                  │ accepted
        │                                  ▼
        │                          ┌─────────────┐
        │   goal complete/error    │  EXECUTING  │
        └──────────────────────────┤             │
                                   └──────┬──────┘
                                          │
                                  cancel_goal()
                                          │
                                          ▼
                                   ┌─────────────┐
                                   │  CANCELING  │
                                   └──────┬──────┘
                                          │ canceled
                                          ▼
                                      (回到IDLE)
```

#### 2.2.2 巡航管理器状态机

```
                  start_patrol()
    ┌────────┐  ────────────────►  ┌─────────────┐
    │  IDLE  │                     │  PATROLLING │
    └────────┘  ◄────────────────  └─────────────┘
                 stop_patrol()            │
                                          │
                    ┌─────────────────────┴──────────────────┐
                    │                                         │
                    ▼                                         ▼
            ┌──────────────┐                         ┌──────────────┐
            │  NAVIGATING  │────────────────────────►│   DWELLING   │
            │  (到达路点)   │   arrival_callback()    │  (停留等待)   │
            └──────────────┘                         └──────┬───────┘
                    ▲                                       │
                    │                                       │
                    └───────────────────────────────────────┘
                            timer_callback()
                           (选择下一路点)
```

### 2.3 数据流设计

```
┌──────────────┐
│  YAML配置文件 │
│  waypoints   │
└──────┬───────┘
       │ load
       ▼
┌─────────────────┐      select_next      ┌──────────────────┐
│ WaypointManager ├────────────────────────►│ Patrol Manager   │
│  - waypoints[]  │                        │  - current_wp    │
│  - current_idx  │                        │  - mode          │
└────────┬────────┘                        └────────┬─────────┘
         │                                          │
         │ get_pose()                               │ send_goal()
         ▼                                          ▼
┌─────────────────┐                        ┌──────────────────┐
│   TF2 Buffer    │                        │ Navigation       │
│  (robot pose)   │                        │ Executor         │
└─────────────────┘                        └────────┬─────────┘
                                                    │
                                                    │ Nav2 Goal
                                                    ▼
                                           ┌──────────────────┐
                                           │   Nav2 Stack     │
                                           │  - Planner       │
                                           │  - Controller    │
                                           └──────────────────┘
```

---

## 3. 核心组件详细设计

### 3.1 Map Library Manager (地图库管理器)

**文件**: `bot_navigation/map_library_manager.py`

#### 3.1.1 职责

- 地图文件的持久化存储和加载
- 地图版本管理和历史记录
- 多地图切换和管理
- 地图元数据管理

#### 3.1.2 核心接口

```python
@dataclass
class MapMetadata:
    """地图元数据 / Map metadata"""
    name: str                       # 地图名称
    version: str                    # 版本号 (如 "v1.0")
    created_time: str               # 创建时间
    map_file: str                   # 地图文件路径 (.yaml)
    pgm_file: str                   # PGM图像文件路径
    description: str = ""           # 描述
    tags: List[str] = None          # 标签 (如 ["floor1", "office"])
    resolution: float = 0.05        # 分辨率
    origin: Tuple[float, float, float] = (0.0, 0.0, 0.0)
    
class MapLibraryManager(Node):
    """地图库管理器 / Map library manager"""
    
    def __init__(self):
        super().__init__('map_library_manager')
        
        # 地图存储路径 / Map storage path
        self.maps_dir = "~/.ros/lekiwi_maps"
        self.current_map: Optional[MapMetadata] = None
        self.map_index: Dict[str, MapMetadata] = {}
        
    def save_map(
        self, 
        map_name: str, 
        map_data: OccupancyGrid,
        description: str = "",
        tags: List[str] = None
    ) -> bool:
        """
        保存地图到文件 / Save map to file
        
        Args:
            map_name: 地图名称
            map_data: 地图数据 (OccupancyGrid)
            description: 描述
            tags: 标签列表
            
        Returns:
            bool: 是否成功保存
            
        保存格式:
          ~/.ros/lekiwi_maps/
          ├── office_floor1_v1.0/
          │   ├── map.yaml          # ROS地图配置
          │   ├── map.pgm           # 地图图像
          │   └── metadata.json     # 元数据
        """
        
    def load_map(self, map_name: str, version: str = "latest") -> Optional[MapMetadata]:
        """
        加载地图 / Load map
        
        Args:
            map_name: 地图名称
            version: 版本号，"latest"表示最新版本
            
        Returns:
            MapMetadata: 地图元数据，失败返回None
        """
        
    def list_maps(self, tags: List[str] = None) -> List[MapMetadata]:
        """
        列出所有地图 / List all maps
        
        Args:
            tags: 标签过滤器，None表示不过滤
            
        Returns:
            地图元数据列表
        """
        
    def delete_map(self, map_name: str, version: str = None) -> bool:
        """
        删除地图 / Delete map
        
        Args:
            map_name: 地图名称
            version: 版本号，None表示删除所有版本
            
        Returns:
            bool: 是否成功删除
        """
        
    def merge_maps(
        self, 
        map1_name: str, 
        map2_name: str, 
        output_name: str
    ) -> bool:
        """
        合并两张地图 / Merge two maps
        
        Args:
            map1_name: 地图1名称
            map2_name: 地图2名称
            output_name: 输出地图名称
            
        Returns:
            bool: 是否成功合并
        """
        
    def export_map(self, map_name: str, export_path: str) -> bool:
        """导出地图到指定路径 / Export map to specified path"""
        
    def import_map(self, import_path: str, map_name: str) -> bool:
        """从文件导入地图 / Import map from file"""
        
    def get_map_statistics(self, map_name: str) -> dict:
        """
        获取地图统计信息 / Get map statistics
        
        Returns:
            {
                'total_cells': int,
                'free_cells': int,
                'occupied_cells': int,
                'unknown_cells': int,
                'size_mb': float
            }
        """
```

#### 3.1.3 地图版本控制

```python
class MapVersionControl:
    """地图版本控制 / Map version control"""
    
    def create_version(self, map_name: str, description: str = "") -> str:
        """
        创建新版本 / Create new version
        
        版本命名规则: v1.0, v1.1, v2.0
        - 主版本号: 重大变更（如重建地图）
        - 次版本号: 增量更新（如扩展区域）
        """
        
    def rollback_to_version(self, map_name: str, version: str) -> bool:
        """回滚到指定版本 / Rollback to specified version"""
        
    def compare_versions(self, map_name: str, v1: str, v2: str) -> dict:
        """
        比较两个版本差异 / Compare two versions
        
        Returns:
            {
                'added_area': float,      # 新增面积 (m²)
                'changed_cells': int,     # 变化的格子数
                'diff_image': str         # 差异图像路径
            }
        """
```

---

### 3.2 Task Manager (任务管理器)

**文件**: `bot_navigation/task_manager.py`

#### 3.2.1 职责

- 任务定义和持久化
- 任务队列管理
- 任务状态机控制
- 任务历史记录

#### 3.2.2 数据结构

```python
@dataclass
class Task:
    """任务数据结构 / Task data structure"""
    task_id: str                    # 唯一任务ID
    task_type: str                  # 任务类型 (exploration_save, patrol_config, etc.)
    name: str                       # 任务名称
    description: str = ""           # 描述
    
    # 任务参数 / Task parameters
    parameters: dict = None         # 任务特定参数
    
    # 任务状态 / Task state
    status: str = "PENDING"         # PENDING, RUNNING, PAUSED, COMPLETED, FAILED, CANCELED
    progress: float = 0.0           # 完成度 (0.0 - 1.0)
    
    # 时间信息 / Time information
    created_time: str = ""
    started_time: str = ""
    completed_time: str = ""
    
    # 优先级和依赖 / Priority and dependencies
    priority: int = 0               # 优先级 (0最低, 100最高)
    dependencies: List[str] = None  # 依赖的任务ID列表
    
    # 结果 / Result
    result: dict = None             # 任务执行结果
    error_message: str = ""         # 错误信息
    
    def to_dict(self) -> dict:
        """转换为字典 / Convert to dictionary"""
        
    @staticmethod
    def from_dict(data: dict) -> 'Task':
        """从字典创建 / Create from dictionary"""

# 任务类型示例 / Task type examples
class TaskType:
    # 探索任务 / Exploration tasks
    EXPLORATION_TEMP = "exploration_temp"
    EXPLORATION_SAVE = "exploration_save"
    EXPLORATION_INCREMENTAL = "exploration_incremental"
    EXPLORATION_REBUILD = "exploration_rebuild"
    
    # 巡航任务 / Patrol tasks
    PATROL_AUTONOMOUS = "patrol_autonomous"
    PATROL_CONFIG = "patrol_config"
    PATROL_RECORDED = "patrol_recorded"
    PATROL_HYBRID = "patrol_hybrid"
    
    # 导航任务 / Navigation tasks
    NAV_SINGLE = "nav_single"
    NAV_FOLLOW = "nav_follow"
    NAV_RETURN_HOME = "nav_return_home"
    NAV_EMERGENCY_STOP = "nav_emergency_stop"
    
    # 复合任务 / Composite tasks
    EXPLORE_THEN_PATROL = "explore_then_patrol"
    SCHEDULED_PATROL = "scheduled_patrol"
    AREA_CLEANING = "area_cleaning"
```

#### 3.2.3 核心接口

```python
class TaskManager(Node):
    """任务管理器 / Task manager"""
    
    def __init__(self):
        super().__init__('task_manager')
        
        # 任务存储 / Task storage
        self.tasks: Dict[str, Task] = {}
        self.task_queue: List[str] = []         # 任务队列 (按优先级排序)
        self.current_task: Optional[Task] = None
        
        # 持久化路径 / Persistence path
        self.tasks_file = "~/.ros/lekiwi_tasks/tasks.json"
        self.history_file = "~/.ros/lekiwi_tasks/history.json"
        
        # 加载已保存的任务 / Load saved tasks
        self.load_tasks()
        
    def create_task(
        self, 
        task_type: str, 
        name: str,
        parameters: dict = None,
        priority: int = 0,
        dependencies: List[str] = None
    ) -> str:
        """
        创建新任务 / Create new task
        
        Args:
            task_type: 任务类型
            name: 任务名称
            parameters: 任务参数
            priority: 优先级
            dependencies: 依赖任务列表
            
        Returns:
            str: 任务ID
        """
        
    def submit_task(self, task_id: str) -> bool:
        """
        提交任务到队列 / Submit task to queue
        
        Args:
            task_id: 任务ID
            
        Returns:
            bool: 是否成功提交
        """
        
    def cancel_task(self, task_id: str) -> bool:
        """取消任务 / Cancel task"""
        
    def pause_task(self, task_id: str) -> bool:
        """暂停任务 / Pause task"""
        
    def resume_task(self, task_id: str) -> bool:
        """恢复任务 / Resume task"""
        
    def get_task(self, task_id: str) -> Optional[Task]:
        """获取任务 / Get task"""
        
    def list_tasks(self, status: str = None) -> List[Task]:
        """
        列出任务 / List tasks
        
        Args:
            status: 状态过滤器，None表示所有任务
        """
        
    def save_tasks(self) -> bool:
        """保存所有任务到文件 / Save all tasks to file"""
        
    def load_tasks(self) -> bool:
        """从文件加载任务 / Load tasks from file"""
        
    def task_loop(self):
        """
        任务执行循环 / Task execution loop
        
        状态机:
        1. 从队列中取出最高优先级任务
        2. 检查依赖是否满足
        3. 执行任务
        4. 更新状态和进度
        5. 保存到历史记录
        """
        
    def _execute_task(self, task: Task) -> bool:
        """
        执行单个任务 / Execute single task
        
        根据task_type调度到对应的管理器:
        - exploration_* → ExplorationManager
        - patrol_* → PatrolManager
        - nav_* → NavigationExecutor
        """
```

#### 3.2.4 任务状态机

```python
class TaskStateMachine:
    """任务状态机 / Task state machine"""
    
    # 状态定义 / State definitions
    PENDING = "PENDING"         # 待执行
    RUNNING = "RUNNING"         # 执行中
    PAUSED = "PAUSED"           # 已暂停
    COMPLETED = "COMPLETED"     # 已完成
    FAILED = "FAILED"           # 失败
    CANCELED = "CANCELED"       # 已取消
    
    # 允许的状态转换 / Valid state transitions
    TRANSITIONS = {
        PENDING: [RUNNING, CANCELED],
        RUNNING: [PAUSED, COMPLETED, FAILED, CANCELED],
        PAUSED: [RUNNING, CANCELED],
        COMPLETED: [],
        FAILED: [PENDING],  # 失败后可重试
        CANCELED: []
    }
    
    @staticmethod
    def can_transition(from_state: str, to_state: str) -> bool:
        """检查状态转换是否合法 / Check if transition is valid"""
        return to_state in TaskStateMachine.TRANSITIONS.get(from_state, [])
```

#### 3.2.5 任务持久化格式

```json
{
  "tasks": [
    {
      "task_id": "task_20251222_001",
      "task_type": "exploration_save",
      "name": "探索1楼办公区",
      "description": "首次建图",
      "parameters": {
        "map_name": "office_floor1",
        "completion_threshold": 0.90,
        "max_exploration_time": 1800
      },
      "status": "COMPLETED",
      "progress": 1.0,
      "created_time": "2025-12-22T10:00:00",
      "started_time": "2025-12-22T10:01:00",
      "completed_time": "2025-12-22T10:25:00",
      "priority": 10,
      "dependencies": [],
      "result": {
        "map_file": "office_floor1_v1.0/map.yaml",
        "explored_area": 125.5,
        "execution_time": 1440
      }
    },
    {
      "task_id": "task_20251222_002",
      "task_type": "patrol_config",
      "name": "每日巡航",
      "parameters": {
        "waypoint_file": "office_waypoints.yaml",
        "patrol_mode": "loop",
        "max_loops": -1
      },
      "status": "RUNNING",
      "progress": 0.35,
      "created_time": "2025-12-22T10:30:00",
      "started_time": "2025-12-22T10:31:00",
      "priority": 5,
      "dependencies": ["task_20251222_001"]
    }
  ]
}
```

---

### 3.3 Navigation Executor (导航执行器基类)

**文件**: `bot_navigation/navigation_executor.py`

#### 3.1.1 职责

- 封装 Nav2 ActionClient 交互逻辑
- 管理导航目标生命周期（发送、等待、取消、结果）
- 提供事务性状态管理
- 处理超时和异常

#### 3.1.2 核心接口

```python
class NavigationExecutor(Node):
    """导航执行器基类 / Base class for navigation execution"""
    
    def __init__(self, node_name: str):
        """
        初始化导航执行器
        
        Args:
            node_name: 节点名称
        """
        
    def send_nav_goal(
        self, 
        x: float, 
        y: float, 
        yaw: float, 
        timeout: float = None
    ) -> bool:
        """
        发送导航目标 - 事务性（仅在IDLE状态可调用）
        
        Args:
            x, y: 目标位置 (世界坐标系)
            yaw: 目标朝向 (弧度)
            timeout: 超时时间 (秒)，None使用默认值
            
        Returns:
            bool: 是否成功发送
        """
        
    def cancel_current_goal(self) -> bool:
        """
        取消当前正在执行的目标
        
        Returns:
            bool: 是否成功发送取消请求
        """
        
    def is_goal_in_progress(self) -> bool:
        """
        检查是否有目标正在执行
        
        Returns:
            bool: True表示正在执行
        """
        
    def get_robot_pose(self) -> Tuple[float, float, float]:
        """
        获取机器人当前位姿
        
        Returns:
            (x, y, yaw): 位置和朝向
        """
        
    def spin_robot(self, angle_degrees: float) -> bool:
        """
        原地旋转指定角度
        
        Args:
            angle_degrees: 旋转角度（度）
            
        Returns:
            bool: 是否成功发送旋转请求
        """
        
    # 回调函数（子类可重写）
    def goal_response_callback(self, future):
        """目标响应回调 / Goal response callback"""
        
    def goal_feedback_callback(self, feedback_msg):
        """目标反馈回调 / Goal feedback callback"""
        
    def goal_result_callback(self, future):
        """目标结果回调 / Goal result callback"""
```

#### 3.1.3 状态管理

```python
class NavState:
    """导航状态枚举 / Navigation state enumeration"""
    IDLE = 'IDLE'                    # 空闲，可接受新目标
    WAITING_ACCEPT = 'WAITING_ACCEPT'  # 等待目标被接受
    EXECUTING = 'EXECUTING'          # 目标执行中
    CANCELING = 'CANCELING'          # 正在取消目标

class NavigationExecutor(Node):
    def __init__(self, node_name: str):
        # ...
        self.nav_state = NavState.IDLE
        self.goal_handle = None
        self.goal_start_time = None
        self.goal_timeout = 80.0  # 默认超时
        
    def _check_state_transition(self, from_state: str, to_state: str) -> bool:
        """检查状态转换是否合法 / Check if state transition is valid"""
        valid_transitions = {
            NavState.IDLE: [NavState.WAITING_ACCEPT],
            NavState.WAITING_ACCEPT: [NavState.IDLE, NavState.EXECUTING],
            NavState.EXECUTING: [NavState.IDLE, NavState.CANCELING],
            NavState.CANCELING: [NavState.IDLE],
        }
        return to_state in valid_transitions.get(from_state, [])
```

---

### 3.2 Waypoint Manager (路点管理器)

**文件**: `bot_navigation/utils/waypoint_manager.py`

#### 3.2.1 职责

- 存储和管理路点列表
- 支持从YAML文件加载和保存
- 提供多种遍历模式（循环、往返、随机）
- 路点查询和搜索

#### 3.2.2 数据结构

```python
@dataclass
class Waypoint:
    """路点数据结构 / Waypoint data structure"""
    x: float                    # X坐标 (米)
    y: float                    # Y坐标 (米)
    yaw: float                  # 朝向 (弧度)
    name: str = ""              # 名称
    dwell_time: float = 2.0     # 停留时间 (秒)
    priority: int = 0           # 优先级 (可选)
    
    def to_dict(self) -> dict:
        """转换为字典"""
        
    @staticmethod
    def from_dict(data: dict) -> 'Waypoint':
        """从字典创建"""
```

#### 3.2.3 核心接口

```python
class WaypointManager:
    """路点管理器 / Waypoint manager"""
    
    def __init__(self):
        self.waypoints: List[Waypoint] = []
        self.current_index: int = 0
        self.direction: int = 1  # 1=正向, -1=反向 (for ping_pong)
        
    def add_waypoint(self, waypoint: Waypoint):
        """添加路点 / Add waypoint"""
        
    def remove_waypoint(self, index: int) -> bool:
        """删除路点 / Remove waypoint"""
        
    def clear_waypoints(self):
        """清空所有路点 / Clear all waypoints"""
        
    def get_waypoint(self, index: int) -> Optional[Waypoint]:
        """获取指定路点 / Get waypoint by index"""
        
    def get_next_waypoint(self, mode: str = 'loop') -> Optional[Waypoint]:
        """
        获取下一个路点 / Get next waypoint
        
        Args:
            mode: 遍历模式
                - 'loop': 循环模式 (0→1→2→0→...)
                - 'ping_pong': 往返模式 (0→1→2→1→0→...)
                - 'once': 单次模式 (0→1→2→结束)
                - 'random': 随机模式
                
        Returns:
            下一个路点，如果遍历结束则返回None
        """
        
    def get_nearest_waypoint(self, x: float, y: float) -> Tuple[int, Waypoint]:
        """
        获取最近的路点 / Get nearest waypoint
        
        Args:
            x, y: 当前位置
            
        Returns:
            (index, waypoint): 最近路点的索引和对象
        """
        
    def load_from_yaml(self, filepath: str) -> bool:
        """
        从YAML文件加载路点 / Load waypoints from YAML file
        
        Args:
            filepath: 文件路径
            
        Returns:
            bool: 是否成功加载
        """
        
    def save_to_yaml(self, filepath: str) -> bool:
        """
        保存路点到YAML文件 / Save waypoints to YAML file
        
        Args:
            filepath: 文件路径
            
        Returns:
            bool: 是否成功保存
        """
        
    def reset_traversal(self):
        """重置遍历状态 / Reset traversal state"""
        self.current_index = 0
        self.direction = 1
```

#### 3.2.4 YAML配置格式

```yaml
# config/patrol_waypoints.yaml
waypoints:
  - name: "起点"
    x: 0.0
    y: 0.0
    yaw: 0.0
    dwell_time: 3.0
    priority: 0
    
  - name: "客厅"
    x: 2.5
    y: 1.0
    yaw: 0.0
    dwell_time: 5.0
    priority: 1
    
  - name: "厨房"
    x: 5.0
    y: 3.5
    yaw: 1.57
    dwell_time: 3.0
    priority: 1
    
  - name: "卧室"
    x: 8.0
    y: 1.5
    yaw: -1.57
    dwell_time: 5.0
    priority: 2

patrol_settings:
  mode: "loop"              # loop, ping_pong, once, random
  default_dwell_time: 2.0   # 默认停留时间
  max_patrol_loops: -1      # -1表示无限循环
  recovery_enabled: true    # 启用恢复行为
```

---

### 3.3 Patrol Manager (巡航管理器)

**文件**: `bot_navigation/patrol_manager.py`

#### 3.3.1 职责

- 管理巡航任务生命周期
- 根据模式选择和导航到下一个路点
- 处理到达后的停留时间
- 异常恢复和重试

#### 3.3.2 核心接口

```python
class PatrolManager(NavigationExecutor):
    """巡航管理器 / Patrol manager"""
    
    def __init__(self):
        super().__init__('patrol_manager')
        
        # 路点管理 / Waypoint management
        self.waypoint_mgr = WaypointManager()
        
        # 巡航状态 / Patrol state
        self.is_patrolling = False
        self.current_waypoint: Optional[Waypoint] = None
        self.patrol_count = 0
        self.max_patrol_loops = -1  # -1表示无限
        
        # 参数配置 / Parameters
        self.declare_parameters(
            namespace='',
            parameters=[
                ('waypoint_file', ''),
                ('patrol_mode', 'loop'),
                ('default_dwell_time', 2.0),
                ('max_patrol_loops', -1),
                ('recovery_enabled', True),
                ('arrival_tolerance', 0.3),  # 到达判定距离
            ]
        )
        
        # 定时器 / Timer
        self.patrol_timer = self.create_timer(1.0, self.patrol_loop)
        self.dwell_timer = None  # 停留定时器（动态创建）
        
    def start_patrol(self) -> bool:
        """
        开始巡航任务 / Start patrol mission
        
        Returns:
            bool: 是否成功启动
        """
        
    def stop_patrol(self):
        """停止巡航任务 / Stop patrol mission"""
        
    def pause_patrol(self):
        """暂停巡航 / Pause patrol"""
        
    def resume_patrol(self):
        """恢复巡航 / Resume patrol"""
        
    def skip_current_waypoint(self):
        """跳过当前路点 / Skip current waypoint"""
        
    def patrol_loop(self):
        """
        巡航主循环 / Main patrol loop
        
        状态机逻辑:
        1. 检查是否正在巡航
        2. 等待当前导航目标完成
        3. 检查停留定时器
        4. 选择下一个路点
        5. 发送导航目标
        """
        
    def goal_result_callback(self, future):
        """
        重写父类方法 - 添加停留时间逻辑
        Override parent method - add dwell time logic
        
        状态转换:
        EXECUTING -> DWELLING -> (继续巡航)
        """
```

#### 3.3.3 巡航模式实现

```python
def _select_next_waypoint_loop(self) -> Optional[Waypoint]:
    """循环模式：0→1→2→0→... / Loop mode"""
    if self.max_patrol_loops > 0:
        if self.patrol_count >= self.max_patrol_loops:
            return None  # 达到最大循环次数
            
    waypoint = self.waypoint_mgr.get_next_waypoint(mode='loop')
    
    # 检测是否完成一轮
    if self.waypoint_mgr.current_index == 0:
        self.patrol_count += 1
        self.get_logger().info(f'完成第 {self.patrol_count} 轮巡航')
        
    return waypoint

def _select_next_waypoint_ping_pong(self) -> Optional[Waypoint]:
    """往返模式：0→1→2→1→0→1→... / Ping-pong mode"""
    waypoint = self.waypoint_mgr.get_next_waypoint(mode='ping_pong')
    
    # 检测是否完成一个往返周期（回到起点）
    if self.waypoint_mgr.current_index == 0 and self.waypoint_mgr.direction == 1:
        self.patrol_count += 1
        
    return waypoint

def _select_next_waypoint_random(self) -> Optional[Waypoint]:
    """随机模式：随机选择路点 / Random mode"""
    return self.waypoint_mgr.get_next_waypoint(mode='random')
```

---

### 3.6 Mission Planner (任务规划器)

**文件**: `bot_navigation/mission_planner.py`

#### 3.6.1 职责

- 统一管理所有类型任务（探索、巡航、导航）
- 集成 TaskManager 进行任务调度
- 处理任务切换和优先级
- 提供完整的ROS2服务接口
- 监控任务状态和异常
- 与语音控制和Web界面集成

#### 3.6.2 核心接口

```python
class MissionPlanner(Node):
    """任务规划器 / Mission planner"""
    
    def __init__(self):
        super().__init__('mission_planner')
        
        # 核心组件 / Core components
        self.task_mgr = TaskManager()
        self.map_lib_mgr = MapLibraryManager()
        self.exploration_mgr: Optional[ExplorationManager] = None
        self.patrol_mgr: Optional[PatrolManager] = None
        
        # 当前状态 / Current state
        self.current_task: Optional[Task] = None
        self.is_active = False
        
        # 服务接口 / Service interfaces - 任务创建
        self.create_service(
            CreateTask,  # 自定义服务
            'mission/create_task',
            self.create_task_callback
        )
        
        self.create_service(
            SubmitTask,
            'mission/submit_task',
            self.submit_task_callback
        )
        
        # 服务接口 - 任务控制
        self.create_service(
            Trigger,
            'mission/pause',
            self.pause_task_callback
        )
        
        self.create_service(
            Trigger,
            'mission/resume',
            self.resume_task_callback
        )
        
        self.create_service(
            Trigger,
            'mission/cancel',
            self.cancel_task_callback
        )
        
        # 服务接口 - 任务查询
        self.create_service(
            GetTaskStatus,
            'mission/get_status',
            self.get_status_callback
        )
        
        self.create_service(
            ListTasks,
            'mission/list_tasks',
            self.list_tasks_callback
        )
        
        # 服务接口 - 快捷任务（兼容旧接口）
        self.create_service(
            StartExploration,  # 自定义服务
            'mission/start_exploration',
            self.start_exploration_callback
        )
        
        self.create_service(
            StartPatrol,
            'mission/start_patrol',
            self.start_patrol_callback
        )
        
        # 服务接口 - 地图管理
        self.create_service(
            SaveMap,
            'mission/save_map',
            self.save_map_callback
        )
        
        self.create_service(
            LoadMap,
            'mission/load_map',
            self.load_map_callback
        )
        
        self.create_service(
            ListMaps,
            'mission/list_maps',
            self.list_maps_callback
        )
        
        # 发布器 / Publishers
        self.status_pub = self.create_publisher(
            MissionStatus,  # 自定义消息
            '/mission/status',
            10
        )
        
        # 订阅器 / Subscribers - 监听子任务完成
        self.create_subscription(
            Bool,
            '/exploration/complete',
            self.exploration_complete_callback,
            10
        )
        
        self.create_subscription(
            Bool,
            '/patrol/complete',
            self.patrol_complete_callback,
            10
        )
        
        # 定时器 / Timers
        self.main_loop_timer = self.create_timer(1.0, self.main_loop)
        self.status_pub_timer = self.create_timer(2.0, self.publish_status)
        
        self.get_logger().info('🎯 任务规划器已启动')
    
    # ===== 任务创建接口 =====
    
    def create_task_callback(self, request, response):
        """
        创建任务服务 / Create task service
        
        Request:
            task_type: str
            name: str
            parameters: json_string
            priority: int
            
        Response:
            success: bool
            task_id: str
            message: str
        """
        try:
            params = json.loads(request.parameters) if request.parameters else {}
            
            task_id = self.task_mgr.create_task(
                task_type=request.task_type,
                name=request.name,
                parameters=params,
                priority=request.priority
            )
            
            response.success = True
            response.task_id = task_id
            response.message = f'任务创建成功: {task_id}'
            
        except Exception as e:
            response.success = False
            response.message = f'任务创建失败: {str(e)}'
            self.get_logger().error(response.message)
            
        return response
    
    def submit_task_callback(self, request, response):
        """提交任务到执行队列 / Submit task to execution queue"""
        task_id = request.task_id
        
        if self.task_mgr.submit_task(task_id):
            response.success = True
            response.message = f'任务已提交: {task_id}'
        else:
            response.success = False
            response.message = f'任务提交失败: {task_id}'
            
        return response
    
    # ===== 任务控制接口 =====
    
    def pause_task_callback(self, request, response):
        """暂停当前任务 / Pause current task"""
        if self.current_task is None:
            response.success = False
            response.message = '没有正在执行的任务'
            return response
            
        if self.task_mgr.pause_task(self.current_task.task_id):
            self._pause_current_executor()
            response.success = True
            response.message = f'任务已暂停: {self.current_task.name}'
        else:
            response.success = False
            response.message = '任务暂停失败'
            
        return response
    
    def resume_task_callback(self, request, response):
        """恢复任务 / Resume task"""
        if self.current_task is None:
            response.success = False
            response.message = '没有可恢复的任务'
            return response
            
        if self.task_mgr.resume_task(self.current_task.task_id):
            self._resume_current_executor()
            response.success = True
            response.message = f'任务已恢复: {self.current_task.name}'
        else:
            response.success = False
            response.message = '任务恢复失败'
            
        return response
    
    def cancel_task_callback(self, request, response):
        """取消任务 / Cancel task"""
        if self.current_task is None:
            response.success = False
            response.message = '没有正在执行的任务'
            return response
            
        if self.task_mgr.cancel_task(self.current_task.task_id):
            self._stop_current_executor()
            response.success = True
            response.message = f'任务已取消: {self.current_task.name}'
        else:
            response.success = False
            response.message = '任务取消失败'
            
        return response
    
    # ===== 任务查询接口 =====
    
    def get_status_callback(self, request, response):
        """获取任务状态 / Get task status"""
        if self.current_task:
            response.task_id = self.current_task.task_id
            response.task_type = self.current_task.task_type
            response.task_name = self.current_task.name
            response.status = self.current_task.status
            response.progress = self.current_task.progress
        else:
            response.task_id = ""
            response.status = "IDLE"
            response.progress = 0.0
            
        return response
    
    def list_tasks_callback(self, request, response):
        """列出所有任务 / List all tasks"""
        tasks = self.task_mgr.list_tasks(status=request.status_filter)
        
        for task in tasks:
            task_info = TaskInfo()
            task_info.task_id = task.task_id
            task_info.task_type = task.task_type
            task_info.name = task.name
            task_info.status = task.status
            task_info.progress = task.progress
            task_info.priority = task.priority
            response.tasks.append(task_info)
            
        response.count = len(tasks)
        return response
    
    # ===== 快捷任务接口 =====
    
    def start_exploration_callback(self, request, response):
        """
        快捷启动探索任务 / Quick start exploration
        
        Request:
            map_name: str
            save_map: bool
            completion_threshold: float
            max_time: int
        """
        # 创建探索任务
        task_id = self.task_mgr.create_task(
            task_type=TaskType.EXPLORATION_SAVE if request.save_map else TaskType.EXPLORATION_TEMP,
            name=f"探索_{request.map_name}",
            parameters={
                'map_name': request.map_name,
                'completion_threshold': request.completion_threshold,
                'max_exploration_time': request.max_time
            },
            priority=10
        )
        
        # 立即提交执行
        if self.task_mgr.submit_task(task_id):
            response.success = True
            response.message = f'探索任务已启动: {task_id}'
        else:
            response.success = False
            response.message = '探索任务启动失败'
            
        return response
    
    def start_patrol_callback(self, request, response):
        """
        快捷启动巡航任务 / Quick start patrol
        
        Request:
            waypoint_source: str  # config, recorded, autonomous
            waypoint_file: str
            patrol_mode: str
            max_loops: int
        """
        # 确定任务类型
        task_type_map = {
            'config': TaskType.PATROL_CONFIG,
            'recorded': TaskType.PATROL_RECORDED,
            'autonomous': TaskType.PATROL_AUTONOMOUS
        }
        task_type = task_type_map.get(request.waypoint_source, TaskType.PATROL_CONFIG)
        
        # 创建巡航任务
        task_id = self.task_mgr.create_task(
            task_type=task_type,
            name=f"巡航_{request.patrol_mode}",
            parameters={
                'waypoint_file': request.waypoint_file,
                'patrol_mode': request.patrol_mode,
                'max_loops': request.max_loops
            },
            priority=5
        )
        
        # 立即提交执行
        if self.task_mgr.submit_task(task_id):
            response.success = True
            response.message = f'巡航任务已启动: {task_id}'
        else:
            response.success = False
            response.message = '巡航任务启动失败'
            
        return response
    
    # ===== 地图管理接口 =====
    
    def save_map_callback(self, request, response):
        """保存当前地图 / Save current map"""
        # 从exploration_mgr或patrol_mgr获取当前地图
        # ...
        return response
    
    def load_map_callback(self, request, response):
        """加载地图 / Load map"""
        map_meta = self.map_lib_mgr.load_map(
            request.map_name,
            request.version
        )
        
        if map_meta:
            response.success = True
            response.message = f'地图已加载: {map_meta.name}'
            # TODO: 通知Nav2加载地图
        else:
            response.success = False
            response.message = '地图加载失败'
            
        return response
    
    def list_maps_callback(self, request, response):
        """列出所有地图 / List all maps"""
        maps = self.map_lib_mgr.list_maps(tags=request.tag_filter)
        
        for map_meta in maps:
            map_info = MapInfo()
            map_info.name = map_meta.name
            map_info.version = map_meta.version
            map_info.created_time = map_meta.created_time
            map_info.description = map_meta.description
            response.maps.append(map_info)
            
        response.count = len(maps)
        return response
    
    # ===== 主循环 =====
    
    def main_loop(self):
        """
        主执行循环 / Main execution loop
        
        职责:
        1. 从TaskManager获取待执行任务
        2. 调度到对应的执行器
        3. 监控执行状态
        4. 处理异常和恢复
        """
        # 如果有正在执行的任务，监控状态
        if self.current_task:
            self._monitor_current_task()
            return
        
        # 从队列中获取下一个任务
        next_task = self.task_mgr.get_next_task()
        if next_task is None:
            return
        
        # 执行任务
        self.current_task = next_task
        self._execute_task(next_task)
    
    def _execute_task(self, task: Task):
        """
        执行任务 / Execute task
        
        根据task_type调度到对应的执行器
        """
        self.get_logger().info(f'开始执行任务: {task.name} ({task.task_type})')
        
        if task.task_type.startswith('exploration'):
            self._execute_exploration_task(task)
        elif task.task_type.startswith('patrol'):
            self._execute_patrol_task(task)
        elif task.task_type.startswith('nav'):
            self._execute_navigation_task(task)
        else:
            self.get_logger().error(f'未知任务类型: {task.task_type}')
            task.status = 'FAILED'
            task.error_message = '未知任务类型'
    
    def _execute_exploration_task(self, task: Task):
        """执行探索任务 / Execute exploration task"""
        if self.exploration_mgr is None:
            self.exploration_mgr = ExplorationManager()
        
        # 配置参数
        params = task.parameters
        # ...
        
        # 启动探索
        self.exploration_mgr.start_exploration()
        task.status = 'RUNNING'
    
    def _execute_patrol_task(self, task: Task):
        """执行巡航任务 / Execute patrol task"""
        if self.patrol_mgr is None:
            self.patrol_mgr = PatrolManager()
        
        # 配置参数
        params = task.parameters
        # ...
        
        # 启动巡航
        self.patrol_mgr.start_patrol()
        task.status = 'RUNNING'
    
    def _execute_navigation_task(self, task: Task):
        """执行导航任务 / Execute navigation task"""
        # ...
    
    def _monitor_current_task(self):
        """监控当前任务执行状态 / Monitor current task execution"""
        # 更新进度
        # 检查是否完成
        # 处理异常
        pass
    
    def publish_status(self):
        """发布任务状态 / Publish mission status"""
        status_msg = MissionStatus()
        
        if self.current_task:
            status_msg.task_id = self.current_task.task_id
            status_msg.task_type = self.current_task.task_type
            status_msg.task_name = self.current_task.name
            status_msg.status = self.current_task.status
            status_msg.progress = self.current_task.progress
        else:
            status_msg.status = "IDLE"
            
        self.status_pub.publish(status_msg)
```

#### 3.6.3 任务切换逻辑

```python
def _switch_task(self, new_task: Task) -> bool:
    """
    任务切换逻辑 / Task switching logic
    
    Steps:
    1. 保存当前任务状态
    2. 停止当前执行器
    3. 清理资源
    4. 加载新任务配置
    5. 启动新执行器
    """
    # 1. 保存当前任务
    if self.current_task:
        self.task_mgr.save_task_state(self.current_task)
        
    # 2. 停止执行器
    self._stop_current_executor()
    
    # 3. 等待清理
    time.sleep(0.5)
    
    # 4. 执行新任务
    self.current_task = new_task
    self._execute_task(new_task)
    
    return True
```

#### 3.6.4 语音控制集成

```python
class VoiceCommandHandler:
    """语音命令处理器 / Voice command handler"""
    
    def __init__(self, mission_planner: MissionPlanner):
        self.planner = mission_planner
        
        # 订阅语音命令
        self.planner.create_subscription(
            String,
            '/voice/command',
            self.voice_command_callback,
            10
        )
    
    def voice_command_callback(self, msg):
        """
        处理语音命令 / Handle voice command
        
        支持的命令:
        - "开始探索": 启动探索任务
        - "开始巡航": 启动巡航任务
        - "暂停任务": 暂停当前任务
        - "继续任务": 恢复任务
        - "取消任务": 取消当前任务
        - "返回基地": 返回充电站
        - "紧急停止": 立即停止所有运动
        """
        command = msg.data.lower()
        
        if "探索" in command or "建图" in command:
            self._start_exploration_from_voice(command)
        elif "巡航" in command or "巡逻" in command:
            self._start_patrol_from_voice(command)
        elif "暂停" in command:
            self.planner.pause_task_callback(Trigger.Request(), Trigger.Response())
        elif "继续" in command or "恢复" in command:
            self.planner.resume_task_callback(Trigger.Request(), Trigger.Response())
        elif "取消" in command or "停止" in command:
            self.planner.cancel_task_callback(Trigger.Request(), Trigger.Response())
        elif "返回" in command or "回家" in command:
            self._return_home()
        elif "紧急" in command:
            self._emergency_stop()
```

---

### 3.7 Command Interface (命令接口适配器) - bot_cmd_interface

**文件**: `bot_cmd_interface/command_adapter.py`

#### 3.7.1 职责

- 统一所有外部输入接口（语音、Web、App、手柄等）
- 异步Topic命令 → 同步ROS2服务调用
- 命令队列管理和去重
- 多终端并发控制
- 命令状态反馈

#### 3.7.2 核心架构

```python
class CommandAdapter(Node):
    """命令适配器 - 异步到同步的桥梁 / Command adapter - async to sync bridge"""
    
    def __init__(self):
        super().__init__('command_adapter')
        
        # 命令队列 / Command queue
        self.command_queue = CommandQueue(max_size=100)
        
        # 服务客户端 / Service clients
        self.service_clients = {
            'create_task': self.create_client(CreateTask, 'mission/create_task'),
            'submit_task': self.create_client(SubmitTask, 'mission/submit_task'),
            'pause': self.create_client(Trigger, 'mission/pause'),
            'resume': self.create_client(Trigger, 'mission/resume'),
            'cancel': self.create_client(Trigger, 'mission/cancel'),
            'get_status': self.create_client(GetTaskStatus, 'mission/get_status'),
            # ... 更多服务
        }
        
        # 订阅通用命令Topic / Subscribe to generic command topic
        self.command_sub = self.create_subscription(
            Command,
            '/cmd/generic',
            self.generic_command_callback,
            10
        )
        
        # 发布命令响应 / Publish command responses
        self.response_pub = self.create_publisher(
            CommandResponse,
            '/cmd/response',
            10
        )
        
        # 发布命令状态 / Publish command status
        self.status_pub = self.create_publisher(
            CommandStatus,
            '/cmd/status',
            10
        )
        
        # 处理器管理 / Handlers
        self.handlers = {
            'voice': VoiceHandler(self),
            'web': WebHandler(self),
            # 'app': AppHandler(self),      # 未来扩展
            # 'joystick': JoystickHandler(self),  # 未来扩展
        }
        
        # 命令处理线程 / Command processing thread
        self.processing_timer = self.create_timer(0.1, self.process_command_queue)
        
        # 当前执行的命令 / Current executing command
        self.current_command: Optional[Command] = None
        self.command_lock = threading.Lock()
        
        self.get_logger().info('🎛️  命令接口适配器已启动')
    
    def generic_command_callback(self, msg: Command):
        """
        通用命令回调 / Generic command callback
        
        Command消息格式:
        - command_id: 唯一命令ID
        - source: 来源 (voice, web, app, joystick)
        - command_type: 命令类型 (start_exploration, start_patrol, pause, etc.)
        - parameters: JSON字符串参数
        - priority: 优先级 (0-100)
        - timestamp: 时间戳
        """
        self.get_logger().info(
            f'收到命令: [{msg.source}] {msg.command_type} (ID: {msg.command_id})'
        )
        
        # 验证命令 / Validate command
        if not self._validate_command(msg):
            self._send_response(msg.command_id, False, '命令验证失败')
            return
        
        # 添加到队列 / Add to queue
        if self.command_queue.add(msg):
            self._send_response(msg.command_id, True, '命令已加入队列')
        else:
            self._send_response(msg.command_id, False, '命令队列已满')
    
    def process_command_queue(self):
        """
        处理命令队列 / Process command queue
        
        逻辑:
        1. 检查是否有命令正在执行
        2. 从队列获取下一个命令
        3. 调用对应的服务
        4. 等待响应并反馈
        """
        # 如果有命令正在执行，等待完成
        with self.command_lock:
            if self.current_command is not None:
                return
            
            # 获取下一个命令
            next_command = self.command_queue.get_next()
            if next_command is None:
                return
            
            self.current_command = next_command
        
        # 执行命令
        self._execute_command(self.current_command)
        
        # 清空当前命令
        with self.command_lock:
            self.current_command = None
    
    def _execute_command(self, cmd: Command):
        """
        执行命令 / Execute command
        
        根据command_type调用对应的服务
        """
        self.get_logger().info(f'执行命令: {cmd.command_type}')
        
        try:
            # 解析参数
            params = json.loads(cmd.parameters) if cmd.parameters else {}
            
            # 根据命令类型调用服务
            if cmd.command_type == 'start_exploration':
                success, msg = self._call_start_exploration(params)
            elif cmd.command_type == 'start_patrol':
                success, msg = self._call_start_patrol(params)
            elif cmd.command_type == 'pause':
                success, msg = self._call_pause()
            elif cmd.command_type == 'resume':
                success, msg = self._call_resume()
            elif cmd.command_type == 'cancel':
                success, msg = self._call_cancel()
            elif cmd.command_type == 'return_home':
                success, msg = self._call_return_home()
            elif cmd.command_type == 'emergency_stop':
                success, msg = self._call_emergency_stop()
            else:
                success = False
                msg = f'未知命令类型: {cmd.command_type}'
            
            # 发送响应
            self._send_response(cmd.command_id, success, msg)
            
        except Exception as e:
            self.get_logger().error(f'命令执行失败: {str(e)}')
            self._send_response(cmd.command_id, False, f'执行异常: {str(e)}')
    
    def _call_start_exploration(self, params: dict) -> Tuple[bool, str]:
        """调用探索服务 / Call exploration service"""
        client = self.service_clients['start_exploration']
        if not client.wait_for_service(timeout_sec=2.0):
            return (False, '探索服务未响应')
        
        request = StartExploration.Request()
        request.map_name = params.get('map_name', 'unnamed_map')
        request.save_map = params.get('save_map', True)
        request.completion_threshold = params.get('completion_threshold', 0.9)
        request.max_time = params.get('max_time', 1800)
        
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        
        if future.result() is not None:
            response = future.result()
            return (response.success, response.message)
        else:
            return (False, '服务调用超时')
    
    def _call_start_patrol(self, params: dict) -> Tuple[bool, str]:
        """调用巡航服务 / Call patrol service"""
        # 类似实现...
        pass
    
    def _call_pause(self) -> Tuple[bool, str]:
        """调用暂停服务 / Call pause service"""
        client = self.service_clients['pause']
        if not client.wait_for_service(timeout_sec=2.0):
            return (False, '暂停服务未响应')
        
        request = Trigger.Request()
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        
        if future.result() is not None:
            response = future.result()
            return (response.success, response.message)
        else:
            return (False, '服务调用超时')
    
    # ... 更多服务调用方法
    
    def _validate_command(self, cmd: Command) -> bool:
        """验证命令有效性 / Validate command"""
        # 检查必需字段
        if not cmd.command_id or not cmd.command_type:
            return False
        
        # 检查来源是否支持
        if cmd.source not in self.handlers:
            self.get_logger().warn(f'未知命令来源: {cmd.source}')
            # 仍然允许执行，只是记录警告
        
        return True
    
    def _send_response(self, command_id: str, success: bool, message: str):
        """发送命令响应 / Send command response"""
        response = CommandResponse()
        response.command_id = command_id
        response.success = success
        response.message = message
        response.timestamp = self.get_clock().now().to_msg()
        
        self.response_pub.publish(response)
        
        self.get_logger().info(
            f'命令响应: {command_id} - {"成功" if success else "失败"}: {message}'
        )
```

#### 3.7.3 命令队列管理

```python
class CommandQueue:
    """命令队列 / Command queue"""
    
    def __init__(self, max_size: int = 100):
        self.max_size = max_size
        self.queue: List[Command] = []
        self.lock = threading.Lock()
        self.command_history: Set[str] = set()  # 去重
    
    def add(self, cmd: Command) -> bool:
        """
        添加命令到队列 / Add command to queue
        
        Features:
        - 去重：相同命令ID只保留最新的
        - 优先级排序
        - 队列大小限制
        """
        with self.lock:
            # 检查队列大小
            if len(self.queue) >= self.max_size:
                return False
            
            # 去重：移除相同ID的旧命令
            self.queue = [c for c in self.queue if c.command_id != cmd.command_id]
            
            # 添加新命令
            self.queue.append(cmd)
            
            # 按优先级排序（高优先级在前）
            self.queue.sort(key=lambda c: c.priority, reverse=True)
            
            return True
    
    def get_next(self) -> Optional[Command]:
        """获取下一个命令 / Get next command"""
        with self.lock:
            if self.queue:
                return self.queue.pop(0)
            return None
    
    def clear(self):
        """清空队列 / Clear queue"""
        with self.lock:
            self.queue.clear()
```

#### 3.7.4 语音命令处理器

```python
class VoiceHandler:
    """语音命令处理器 / Voice command handler"""
    
    def __init__(self, adapter: CommandAdapter):
        self.adapter = adapter
        
        # 订阅语音命令 / Subscribe to voice commands
        self.voice_sub = adapter.create_subscription(
            String,
            '/voice/command',
            self.voice_callback,
            10
        )
        
        # 加载命令映射 / Load command mappings
        self.command_map = self._load_command_map()
        
        adapter.get_logger().info('🎤 语音命令处理器已启动')
    
    def voice_callback(self, msg: String):
        """
        语音命令回调 / Voice command callback
        
        将自然语言转换为标准Command消息
        """
        text = msg.data.lower()
        self.adapter.get_logger().info(f'收到语音命令: "{text}"')
        
        # 解析命令
        cmd = self._parse_voice_command(text)
        if cmd is None:
            self.adapter.get_logger().warn(f'无法识别的语音命令: "{text}"')
            return
        
        # 发布到通用命令Topic
        self.adapter.command_sub.callback(cmd)
    
    def _parse_voice_command(self, text: str) -> Optional[Command]:
        """
        解析语音命令 / Parse voice command
        
        示例映射:
        - "开始探索" → start_exploration
        - "开始巡航" → start_patrol
        - "暂停任务" → pause
        - "继续任务" → resume
        - "取消任务" → cancel
        - "返回基地" → return_home
        - "紧急停止" → emergency_stop
        """
        cmd = Command()
        cmd.command_id = f"voice_{int(time.time()*1000)}"
        cmd.source = "voice"
        cmd.timestamp = self.adapter.get_clock().now().to_msg()
        cmd.priority = 50  # 默认优先级
        
        # 匹配命令类型
        if "探索" in text or "建图" in text:
            cmd.command_type = "start_exploration"
            # 提取参数
            if "保存" in text:
                params = {"map_name": "voice_map", "save_map": True}
            else:
                params = {"map_name": "temp_map", "save_map": False}
            cmd.parameters = json.dumps(params)
            
        elif "巡航" in text or "巡逻" in text:
            cmd.command_type = "start_patrol"
            params = {"patrol_mode": "loop", "max_loops": -1}
            cmd.parameters = json.dumps(params)
            
        elif "暂停" in text:
            cmd.command_type = "pause"
            
        elif "继续" in text or "恢复" in text:
            cmd.command_type = "resume"
            
        elif "取消" in text or "停止" in text:
            if "紧急" in text:
                cmd.command_type = "emergency_stop"
                cmd.priority = 100  # 最高优先级
            else:
                cmd.command_type = "cancel"
            
        elif "返回" in text or "回家" in text or "充电" in text:
            cmd.command_type = "return_home"
            
        else:
            return None
        
        return cmd
    
    def _load_command_map(self) -> dict:
        """从配置文件加载命令映射 / Load command map from config"""
        # TODO: 从voice_commands.yaml加载
        return {}
```

#### 3.7.5 消息定义

**Command.msg**:
```
# 通用命令消息 / Generic command message
string command_id           # 唯一ID
string source               # 来源: voice, web, app, joystick
string command_type         # 命令类型
string parameters           # JSON格式参数
uint8 priority              # 优先级 0-100
time timestamp              # 时间戳
```

**CommandResponse.msg**:
```
# 命令响应消息 / Command response message
string command_id
bool success
string message
time timestamp
```

**CommandStatus.msg**:
```
# 命令状态消息 / Command status message
string command_id
string status               # QUEUED, EXECUTING, COMPLETED, FAILED
float32 progress            # 0.0 - 1.0
time timestamp
```

---

### 3.8 Waypoint Recorder (路点录制器)

**文件**: `bot_navigation/waypoint_recorder.py`

#### 3.5.1 职责

- 实时录制机器人位置作为路点
- 支持手动添加和自动记录
- 提供交互式CLI界面

#### 3.5.2 核心接口

```python
class WaypointRecorder(Node):
    """路点录制器 / Waypoint recorder"""
    
    def __init__(self):
        super().__init__('waypoint_recorder')
        
        self.waypoint_mgr = WaypointManager()
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        
        # 服务接口 / Service interfaces
        self.create_service(
            Trigger,
            'waypoint/record_current',
            self.record_current_callback
        )
        
        self.create_service(
            SaveWaypoints,  # 自定义服务类型
            'waypoint/save',
            self.save_waypoints_callback
        )
        
    def record_current_position(self, name: str = None) -> Waypoint:
        """
        录制当前位置为路点 / Record current position as waypoint
        
        Args:
            name: 路点名称，None则自动生成
            
        Returns:
            创建的路点对象
        """
        
    def run_interactive_cli(self):
        """
        运行交互式CLI界面 / Run interactive CLI
        
        命令:
        - 'r' 或 'record': 录制当前位置
        - 'l' 或 'list': 列出所有路点
        - 'd <index>' 或 'delete <index>': 删除路点
        - 's <file>' 或 'save <file>': 保存到文件
        - 'q' 或 'quit': 退出
        """
```

---

## 4. 文件结构

### 4.1 bot_navigation 功能包

```
src/bot_navigation/
├── bot_navigation/
│   ├── __init__.py
│   │
│   ├── navigation_executor.py        # 新增：导航执行器基类
│   ├── exploration_manager.py        # 重构：探索管理器(从exploration_mapper.py)
│   ├── patrol_manager.py             # 新增：巡航管理器
│   ├── mission_planner.py            # 新增：任务规划器
│   ├── task_manager.py               # 新增：任务管理器
│   ├── waypoint_recorder.py          # 新增：路点录制器
│   │
│   └── utils/
│       ├── __init__.py
│       ├── waypoint_manager.py       # 新增：路点管理
│       ├── state_machine.py          # 可选：状态机基类
│       └── task_types.py             # 新增：任务类型定义
│
├── config/
│   ├── patrol_waypoints.yaml         # 新增：示例路点配置
│   ├── patrol_params.yaml            # 新增：巡航参数配置
│   ├── mission_planner.yaml          # 新增：任务规划器配置
│   └── task_manager.yaml             # 新增：任务管理器配置
│
├── launch/
│   ├── exploration.launch.py         # 更新：探索建图
│   ├── patrol.launch.py              # 新增：巡航任务
│   ├── mission_planner.launch.py     # 新增：任务规划器(总入口)
│   ├── record_waypoints.launch.py    # 新增：路点录制
│   └── test_patrol.launch.py         # 新增：测试脚本
│
├── srv/
│   ├── CreateTask.srv                # 任务服务
│   ├── SubmitTask.srv
│   ├── GetTaskStatus.srv
│   ├── ListTasks.srv
│   ├── SaveWaypoints.srv             # 路点服务
│   ├── LoadWaypoints.srv
│   ├── StartExploration.srv          # 快捷任务服务
│   └── StartPatrol.srv
│
├── msg/
│   ├── Task.msg                      # 任务消息
│   ├── TaskInfo.msg
│   └── MissionStatus.msg
│
├── data/                             # 数据目录
│   ├── waypoints/                    # 路点存储
│   │   ├── office_waypoints.yaml
│   │   └── home_waypoints.yaml
│   └── tasks/                        # 任务存储
│       ├── tasks.json
│       └── history.json
│
├── test/
│   ├── test_waypoint_manager.py      # 单元测试
│   ├── test_patrol_manager.py
│   ├── test_navigation_executor.py
│   ├── test_task_manager.py
│   └── integration/                  # 集成测试
│       ├── test_exploration_flow.py
│       ├── test_patrol_flow.py
│       └── test_task_switching.py
│
├── package.xml                       # 更新：添加新依赖
├── setup.py                          # 更新：注册新节点和脚本
└── README.md                         # 更新：功能包文档
```

### 4.2 bot_slam 功能包

```
src/bot_slam/
├── bot_slam/
│   ├── __init__.py
│   ├── map_library_manager.py        # 新增：地图库管理器
│   └── utils/
│       ├── __init__.py
│       └── map_analyzer.py           # 新增：地图分析工具
│
├── config/
│   ├── rtabmap_config.yaml           # RTABMap配置
│   ├── slam_toolbox_config.yaml      # SLAM Toolbox配置
│   ├── map_library.yaml              # 新增：地图库配置
│   └── robot_localization_*.yaml     # 定位配置（已有）
│
├── launch/
│   ├── rtabmap.launch.py             # 已有
│   ├── slam_toolbox.launch.py        # 已有
│   └── map_library_manager.launch.py # 新增：地图库管理器启动
│
├── srv/
│   ├── SaveMap.srv                   # 地图服务
│   ├── LoadMap.srv
│   ├── ListMaps.srv
│   └── MergeMaps.srv
│
├── msg/
│   ├── MapMetadata.msg               # 地图消息
│   └── MapInfo.msg
│
├── test/
│   └── test_map_library_manager.py
│
├── package.xml
├── setup.py
└── README.md
```

### 4.3 bot_cmd_interface 功能包（新建）

```
src/bot_cmd_interface/
├── bot_cmd_interface/
│   ├── __init__.py
│   │
│   ├── command_adapter.py            # 核心：命令适配器
│   ├── command_queue.py              # 命令队列管理
│   ├── service_adapter.py            # 服务调用封装
│   │
│   ├── handlers/                     # 各种输入处理器
│   │   ├── __init__.py
│   │   ├── voice_handler.py          # 语音命令处理
│   │   ├── web_handler.py            # Web界面命令处理
│   │   ├── app_handler.py            # App命令处理（未来）
│   │   └── joystick_handler.py       # 手柄命令处理（未来）
│   │
│   └── utils/
│       ├── __init__.py
│       ├── command_validator.py      # 命令验证
│       └── response_formatter.py     # 响应格式化
│
├── config/
│   ├── command_adapter.yaml          # 命令适配器配置
│   ├── voice_commands.yaml           # 语音命令映射
│   └── web_commands.yaml             # Web命令映射
│
├── launch/
│   ├── command_interface.launch.py   # 主启动文件
│   └── voice_interface.launch.py     # 语音接口启动
│
├── msg/
│   ├── Command.msg                   # 通用命令消息
│   ├── CommandResponse.msg           # 命令响应消息
│   └── CommandStatus.msg             # 命令状态消息
│
├── srv/
│   └── ExecuteCommand.srv            # 同步命令执行服务（备用）
│
├── test/
│   ├── test_command_adapter.py
│   ├── test_command_queue.py
│   └── test_voice_handler.py
│
├── package.xml
├── setup.py
└── README.md
```

### 4.4 数据存储目录结构

```
~/.ros/lekiwi_bot/
├── maps/                             # 地图库 (bot_slam管理)
│   ├── index.json                    # 地图索引
│   ├── office_floor1_v1.0/
│   │   ├── map.yaml
│   │   ├── map.pgm
│   │   └── metadata.json
│   ├── office_floor1_v1.1/
│   │   ├── map.yaml
│   │   ├── map.pgm
│   │   └── metadata.json
│   └── home_v1.0/
│       ├── map.yaml
│       ├── map.pgm
│       └── metadata.json
│
├── navigation/                       # 导航数据 (bot_navigation管理)
│   ├── waypoints/                    # 路点库
│   │   ├── office_patrol.yaml
│   │   ├── home_cleaning.yaml
│   │   └── recorded_2025-12-22.yaml
│   │
│   └── tasks/                        # 任务存储
│       ├── tasks.json                # 当前任务
│       ├── history.json              # 历史记录
│       └── templates/                # 任务模板
│           ├── daily_patrol.json
│           └── weekly_exploration.json
│
└── logs/                             # 日志
    ├── mission_2025-12-22.log
    ├── commands_2025-12-22.log       # 命令日志
    └── errors.log
```

---

## 5. 数据结构与接口

### 5.1 自定义消息类型

#### 5.1.1 SaveWaypoints.srv

```
# 请求 / Request
string filepath
bool append  # true=追加, false=覆盖
---
# 响应 / Response
bool success
string message
int32 waypoint_count
```

#### 5.1.2 LoadWaypoints.srv

```
# 请求 / Request
string filepath
---
# 响应 / Response
bool success
string message
int32 waypoint_count
```

### 5.2 参数配置

#### 5.2.1 patrol_params.yaml

```yaml
patrol_manager:
  ros__parameters:
    # 路点配置 / Waypoint configuration
    waypoint_file: "config/patrol_waypoints.yaml"
    
    # 巡航模式 / Patrol mode
    patrol_mode: "loop"  # loop, ping_pong, once, random
    max_patrol_loops: -1  # -1表示无限循环
    
    # 停留时间 / Dwell time
    default_dwell_time: 2.0  # 秒
    
    # 导航参数 / Navigation parameters
    arrival_tolerance: 0.3    # 到达判定距离 (米)
    goal_timeout: 80.0        # 目标超时 (秒)
    
    # 恢复行为 / Recovery behaviors
    recovery_enabled: true
    max_recovery_attempts: 3
    recovery_timeout: 30.0
    
    # 日志 / Logging
    verbose: true
    log_waypoint_arrivals: true
```

#### 5.2.2 mission_planner.yaml

```yaml
mission_planner:
  ros__parameters:
    # 任务优先级 / Mission priority
    allow_mission_switch: true
    default_mission: "patrol"
    
    # 探索配置 / Exploration configuration
    exploration_completion_threshold: 0.90
    
    # 巡航配置 / Patrol configuration
    patrol_waypoint_file: "config/patrol_waypoints.yaml"
    
    # 监控 / Monitoring
    status_publish_rate: 1.0  # Hz
    heartbeat_timeout: 10.0   # 秒
```

---

## 6. 实施计划

### 6.1 阶段划分

#### **阶段 1: 基础架构重构** (预计 2-3 天)

**目标**: 提取共用逻辑，建立基类

**任务清单**:
- [ ] 1.1 创建 `NavigationExecutor` 基类
  - [ ] 实现状态机管理
  - [ ] 实现Nav2 ActionClient封装
  - [ ] 实现TF变换和位姿获取
  - [ ] 添加超时和取消机制
  
- [ ] 1.2 重构 `exploration_mapper.py` → `exploration_manager.py`
  - [ ] 继承 `NavigationExecutor` 基类
  - [ ] 移除重复代码
  - [ ] 测试功能完整性
  
- [ ] 1.3 创建任务类型定义
  - [ ] 定义 `TaskType` 类
  - [ ] 定义所有任务类型常量
  
- [ ] 1.4 编写单元测试
  - [ ] 测试状态转换逻辑
  - [ ] 测试目标发送和取消
  - [ ] 测试异常处理

**验收标准**:
- ✅ 探索建图功能正常，无回归bug
- ✅ 单元测试覆盖率 > 80%
- ✅ 代码符合PEP8和双语注释规范

---

#### **阶段 2: 地图库管理系统** (预计 2-3 天)

**目标**: 实现地图持久化和版本管理

**任务清单**:
- [ ] 2.1 实现 `MapLibraryManager` 类
  - [ ] 地图元数据结构定义
  - [ ] 地图保存和加载功能
  - [ ] 地图列表和查询
  - [ ] 地图删除和导入导出
  
- [ ] 2.2 实现地图版本控制
  - [ ] 版本命名规则
  - [ ] 版本创建和回滚
  - [ ] 版本比较功能
  
- [ ] 2.3 集成RTABMap地图保存
  - [ ] 订阅RTABMap地图话题
  - [ ] 自动保存触发器
  - [ ] 地图合并功能
  
- [ ] 2.4 创建地图库配置
  - [ ] `map_library.yaml`
  - [ ] 存储路径配置
  - [ ] 自动清理策略
  
- [ ] 2.5 编写单元测试
  - [ ] 测试地图保存和加载
  - [ ] 测试版本管理
  - [ ] 测试地图合并

**验收标准**:
- ✅ 可以保存和加载地图
- ✅ 版本控制工作正常
- ✅ 地图元数据完整
- ✅ 单元测试通过

---

#### **阶段 3: 任务管理器** (预计 3-4 天)

**目标**: 实现任务定义、持久化和调度

**任务清单**:
- [ ] 3.1 实现 `Task` 数据结构
  - [ ] Task dataclass定义
  - [ ] 任务序列化和反序列化
  - [ ] 任务状态枚举
  
- [ ] 3.2 实现 `TaskManager` 类
  - [ ] 任务创建和提交
  - [ ] 任务队列管理
  - [ ] 任务优先级排序
  - [ ] 任务依赖检查
  
- [ ] 3.3 实现任务状态机
  - [ ] 状态转换逻辑
  - [ ] 状态转换验证
  - [ ] 状态持久化
  
- [ ] 3.4 实现任务持久化
  - [ ] JSON格式保存
  - [ ] 任务历史记录
  - [ ] 任务恢复机制
  
- [ ] 3.5 创建任务模板
  - [ ] 常用任务模板
  - [ ] 模板加载功能
  
- [ ] 3.6 编写单元测试
  - [ ] 测试任务创建和队列
  - [ ] 测试状态机转换
  - [ ] 测试持久化

**验收标准**:
- ✅ 可以创建和管理任务
- ✅ 任务队列按优先级执行
- ✅ 任务状态正确持久化
- ✅ 单元测试通过

---

#### **阶段 4: 路点管理系统** (预计 2-3 天)

**目标**: 实现路点存储、加载和遍历

**任务清单**:
- [ ] 4.1 实现 `WaypointManager` 类
  - [ ] 数据结构定义 (`Waypoint` dataclass)
  - [ ] 增删改查接口
  - [ ] YAML加载和保存
  - [ ] 遍历模式实现（loop, ping_pong, once, random, priority, coverage）
  
- [ ] 4.2 实现 `WaypointRecorder` 节点
  - [ ] TF监听和位姿获取
  - [ ] 交互式CLI界面
  - [ ] 服务接口实现
  - [ ] 实时录制功能
  
- [ ] 4.3 创建示例配置文件
  - [ ] `patrol_waypoints.yaml`
  - [ ] 多个测试场景的路点
  
- [ ] 4.4 编写单元测试
  - [ ] 测试所有遍历模式
  - [ ] 测试YAML读写
  - [ ] 测试最近路点查询

**验收标准**:
- ✅ 可以录制和保存路点
- ✅ YAML文件格式正确
- ✅ 所有遍历模式工作正常
- ✅ 单元测试通过

---

#### **阶段 5: 巡航管理器** (预计 2-3 天)

**目标**: 实现多模式巡航功能

**任务清单**:
- [ ] 5.1 实现 `PatrolManager` 类
  - [ ] 继承 `NavigationExecutor`
  - [ ] 集成 `WaypointManager`
  - [ ] 实现巡航主循环
  - [ ] 实现停留时间控制
  
- [ ] 5.2 实现所有巡航模式
  - [ ] loop: 循环模式
  - [ ] ping_pong: 往返模式
  - [ ] once: 单次模式
  - [ ] random: 随机模式
  - [ ] priority: 优先级模式
  - [ ] coverage: 覆盖模式
  
- [ ] 5.3 实现巡航来源切换
  - [ ] 配置文件路点
  - [ ] 录制路点
  - [ ] 自动生成路点（基于地图）
  
- [ ] 5.4 创建启动文件
  - [ ] `patrol.launch.py`
  - [ ] 参数配置传递
  
- [ ] 5.5 集成测试
  - [ ] 所有模式测试
  - [ ] 中断和恢复测试

**验收标准**:
- ✅ 所有巡航模式工作正常
- ✅ 可以切换路点来源
- ✅ 到达路点后正确停留
- ✅ 日志输出清晰

---

#### **阶段 6: 任务规划器** (预计 3-4 天)

**目标**: 统一管理所有任务类型

**任务清单**:
- [ ] 6.1 实现 `MissionPlanner` 类
  - [ ] 集成 TaskManager
  - [ ] 集成 MapLibraryManager
  - [ ] 任务调度逻辑
  - [ ] 任务切换逻辑
  
- [ ] 6.2 实现完整服务接口
  - [ ] 任务创建和提交接口
  - [ ] 任务控制接口（暂停/恢复/取消）
  - [ ] 任务查询接口
  - [ ] 地图管理接口
  - [ ] 快捷任务接口
  
- [ ] 6.3 定义自定义消息和服务
  - [ ] Task.msg, TaskInfo.msg, MissionStatus.msg
  - [ ] CreateTask.srv, SubmitTask.srv等
  - [ ] 更新package.xml和CMakeLists.txt
  
- [ ] 6.4 实现任务执行调度
  - [ ] 主执行循环
  - [ ] 任务分发到执行器
  - [ ] 状态监控
  
- [ ] 6.5 创建启动文件
  - [ ] `mission_planner.launch.py`（总入口）
  
- [ ] 6.6 集成测试
  - [ ] 任务创建和执行测试
  - [ ] 任务切换测试
  - [ ] 异常恢复测试

**验收标准**:
- ✅ 可以通过服务管理所有任务
- ✅ 任务切换正确且平滑
- ✅ 地图管理功能正常
- ✅ 异常处理完善

---

#### **阶段 7: 命令接口层 (bot_cmd_interface)** (预计 2-3 天)

**目标**: 构建统一的命令接口适配层

**任务清单**:
- [ ] 7.1 创建 bot_cmd_interface 功能包
  - [ ] 功能包初始化
  - [ ] 定义Command、CommandResponse、CommandStatus消息
  - [ ] 配置依赖
  
- [ ] 7.2 实现 CommandAdapter 核心
  - [ ] 命令队列管理
  - [ ] 服务客户端封装
  - [ ] 异步命令订阅
  - [ ] 同步服务调用
  - [ ] 响应发布
  
- [ ] 7.3 实现 VoiceHandler
  - [ ] 订阅语音命令Topic
  - [ ] 语音命令解析
  - [ ] 转换为标准Command
  - [ ] 加载voice_commands.yaml配置
  
- [ ] 7.4 实现 WebHandler（可选）
  - [ ] Web命令格式定义
  - [ ] 命令验证
  
- [ ] 7.5 集成测试
  - [ ] 语音命令测试
  - [ ] 多终端并发测试
  - [ ] 命令队列测试
  
- [ ] 7.6 创建启动文件
  - [ ] command_interface.launch.py

**验收标准**:
- ✅ 可以通过Topic发送命令
- ✅ 命令正确转换为服务调用
- ✅ 多终端并发控制正常
- ✅ 语音命令识别准确

---

#### **阶段 8: 地图库管理 (bot_slam)** (预计 2-3 天)

**目标**: 实现地图持久化和管理

**任务清单**:
- [ ] 8.1 实现 MapLibraryManager
  - [ ] 地图元数据定义
  - [ ] 地图保存功能
  - [ ] 地图加载功能
  - [ ] 地图列表和查询
  
- [ ] 8.2 实现版本控制
  - [ ] 版本命名规则
  - [ ] 版本创建
  - [ ] 版本比较
  
- [ ] 8.3 集成RTABMap
  - [ ] 订阅RTABMap地图话题
  - [ ] 自动保存触发器
  
- [ ] 8.4 定义服务接口
  - [ ] SaveMap.srv
  - [ ] LoadMap.srv
  - [ ] ListMaps.srv
  
- [ ] 8.5 创建启动文件
  - [ ] map_library_manager.launch.py
  
- [ ] 8.6 测试
  - [ ] 地图保存和加载测试
  - [ ] 版本管理测试

**验收标准**:
- ✅ 可以保存和加载地图
- ✅ 版本控制工作正常
- ✅ 与RTABMap集成无问题

---

#### **阶段 9: 工具和文档** (预计 2-3 天)

**目标**: 完善工具脚本和文档

**任务清单**:
- [ ] 8.1 命令行工具
  - [ ] `create_task.py` - 创建任务
  - [ ] `list_maps.py` - 列出地图
  - [ ] `export_map.py` - 导出地图
  - [ ] `import_waypoints.py` - 导入路点
  
- [ ] 8.2 Web控制界面（可选）
  - [ ] 地图可视化
  - [ ] 任务管理面板
  - [ ] 实时状态监控
  
- [ ] 8.3 完善文档
  - [ ] README_PATROL.md - 巡航系统使用指南
  - [ ] README_TASK_SYSTEM.md - 任务系统文档
  - [ ] API文档
  - [ ] 故障排查指南
  
- [ ] 8.4 性能优化
  - [ ] 内存占用优化
  - [ ] 响应延迟优化
  - [ ] 日志输出优化
  
- [ ] 8.5 完整系统测试
  - [ ] 长时间运行测试（8小时+）
  - [ ] 异常场景测试
  - [ ] 用户验收测试

**验收标准**:
- ✅ 工具脚本易用
- ✅ 文档完整清晰
- ✅ 系统稳定运行
- ✅ 用户满意

---

### 6.2 时间估算

| 阶段 | 任务 | 预计时间 | 依赖 |
|------|------|---------|------|
| 阶段1 | 基础架构重构 | 2-3天 | - |
| 阶段2 | 地图库管理系统 (bot_slam) | 2-3天 | 阶段1 |
| 阶段3 | 任务管理器 (bot_navigation) | 3-4天 | 阶段1 |
| 阶段4 | 路点管理系统 (bot_navigation) | 2-3天 | 阶段1 |
| 阶段5 | 巡航管理器 (bot_navigation) | 2-3天 | 阶段1, 阶段4 |
| 阶段6 | 任务规划器 (bot_navigation) | 3-4天 | 阶段2, 阶段3, 阶段5 |
| 阶段7 | 命令接口层 (bot_cmd_interface) | 2-3天 | 阶段6 |
| 阶段8 | 地图库管理 (bot_slam) | 2-3天 | 阶段2 |
| 阶段9 | 工具和文档 | 2-3天 | 所有阶段 |
| **总计** | | **20-29天** | |

### 6.3 并行开发建议

为了加快开发速度，某些阶段可以并行进行：

**第一批并行** (阶段1完成后):
- 阶段2: 地图库管理系统 (bot_slam)
- 阶段3: 任务管理器 (bot_navigation)
- 阶段4: 路点管理系统 (bot_navigation)

**第二批并行** (第一批完成后):
- 阶段5: 巡航管理器 (bot_navigation)
- 文档编写开始

**第三批** (阶段5完成后):
- 阶段6: 任务规划器 (bot_navigation)
- 阶段8: 地图库完善 (bot_slam)

**第四批** (阶段6完成后):
- 阶段7: 命令接口层 (bot_cmd_interface)
- 阶段9: 工具和文档完善

**最快完成时间**: 约 **15-18 天** (并行开发)

---
- [ ] 5.1 实现 `MissionPlanner` 类
  - [ ] 任务状态管理
  - [ ] 任务切换逻辑
  - [ ] 异常处理
  
- [ ] 5.2 实现服务接口
  - [ ] 启动探索任务
  - [ ] 启动巡航任务
  - [ ] 停止任务
  - [ ] 查询状态
  
- [ ] 5.3 创建启动文件
  - [ ] `mission_planner.launch.py`
  
- [ ] 5.4 集成测试
  - [ ] 任务切换测试
  - [ ] 异常恢复测试

**验收标准**:
- ✅ 可以通过服务控制任务
- ✅ 任务切换正确
- ✅ 异常处理完善

---

#### **阶段 6: 集成与优化** (预计 2 天)

**目标**: 系统集成和性能优化

**任务清单**:
- [ ] 6.1 与语音控制集成
  - [ ] 添加语音命令
  - [ ] 语音反馈
  
- [ ] 6.2 性能优化
  - [ ] 内存占用优化
  - [ ] 路径规划优化
  - [ ] 响应延迟优化
  
- [ ] 6.3 文档完善
  - [ ] API文档
  - [ ] 使用教程
  - [ ] 故障排查指南
  
- [ ] 6.4 完整系统测试
  - [ ] 长时间运行测试
  - [ ] 异常场景测试
  - [ ] 用户验收测试

**验收标准**:
- ✅ 语音控制工作正常
- ✅ 系统稳定运行 > 1小时
- ✅ 文档完整
- ✅ 用户满意

---

### 6.2 时间估算

| 阶段 | 任务 | 预计时间 | 依赖 |
|------|------|---------|------|
| 阶段1 | 基础架构重构 | 1-2天 | - |
| 阶段2 | 路点管理系统 | 2-3天 | 阶段1 |
| 阶段3 | 巡航管理器 | 2-3天 | 阶段1, 阶段2 |
| 阶段4 | 扩展巡航模式 | 2天 | 阶段3 |
| 阶段5 | 任务规划器 | 1-2天 | 阶段1, 阶段3 |
| 阶段6 | 集成与优化 | 2天 | 所有阶段 |
| **总计** | | **10-14天** | |

---

## 7. 测试方案

### 7.1 单元测试

**测试框架**: `pytest` + `unittest`

#### 7.1.1 WaypointManager 测试

```python
# test/test_waypoint_manager.py
def test_add_waypoint():
    """测试添加路点"""
    
def test_remove_waypoint():
    """测试删除路点"""
    
def test_loop_mode():
    """测试循环模式遍历"""
    
def test_ping_pong_mode():
    """测试往返模式遍历"""
    
def test_yaml_save_load():
    """测试YAML文件读写"""
    
def test_nearest_waypoint():
    """测试最近路点查询"""
```

#### 7.1.2 NavigationExecutor 测试

```python
# test/test_navigation_executor.py
def test_state_transitions():
    """测试状态转换"""
    
def test_goal_send():
    """测试目标发送"""
    
def test_goal_cancel():
    """测试目标取消"""
    
def test_timeout_handling():
    """测试超时处理"""
```

### 7.2 集成测试

#### 7.2.1 巡航循环测试

```bash
# 测试场景：3个路点循环巡航
ros2 launch bot_navigation test_patrol.launch.py \
    waypoint_file:=test_waypoints_simple.yaml \
    patrol_mode:=loop \
    max_patrol_loops:=2
```

**预期结果**:
- 机器人依次访问3个路点
- 完成2轮循环后自动停止
- 日志输出清晰

#### 7.2.2 任务切换测试

```bash
# 启动任务规划器
ros2 launch bot_navigation mission_planner.launch.py

# 启动探索任务
ros2 service call /mission/start_exploration std_srvs/srv/Trigger

# 等待5秒后切换到巡航
sleep 5
ros2 service call /mission/stop std_srvs/srv/Trigger
ros2 service call /mission/start_patrol std_srvs/srv/Trigger
```

**预期结果**:
- 探索任务正常启动
- 停止命令正确响应
- 巡航任务正常启动

### 7.3 压力测试

#### 7.3.1 长时间运行测试

```bash
# 无限循环巡航，运行8小时
ros2 launch bot_navigation patrol.launch.py \
    max_patrol_loops:=-1 &

# 监控资源占用
watch -n 5 "ps aux | grep patrol_manager"
```

**监控指标**:
- 内存占用是否稳定
- CPU使用率是否正常
- 是否有内存泄漏

#### 7.3.2 异常场景测试

**场景1**: 路径被堵
- 人为在路径上放置障碍物
- 观察Nav2恢复行为
- 检查巡航是否继续

**场景2**: 网络中断
- 断开WiFi连接
- 检查节点是否崩溃
- 重连后是否恢复

**场景3**: 错误的路点
- 配置无法到达的路点
- 检查超时和重试机制
- 确认日志输出

---

## 8. 未来扩展

### 8.1 高级功能

#### 8.1.1 动态路点调整

```python
class DynamicWaypointManager(WaypointManager):
    """动态路点管理器 / Dynamic waypoint manager"""
    
    def add_temporary_waypoint(self, waypoint: Waypoint, priority: int):
        """插入临时路点（如充电站、检查点）"""
        
    def reroute_to_waypoint(self, index: int):
        """重新规划到指定路点"""
        
    def optimize_route(self):
        """基于TSP算法优化路径顺序"""
```

#### 8.1.2 地图感知巡航

```python
class MapAwarePatrol(PatrolManager):
    """地图感知巡航 / Map-aware patrol"""
    
    def analyze_coverage(self) -> float:
        """分析巡航覆盖率"""
        
    def generate_waypoints_from_map(self, spacing: float):
        """从地图自动生成路点网格"""
        
    def avoid_crowded_areas(self):
        """避开拥挤区域（基于代价地图）"""
```

#### 8.1.3 多机器人协作

```python
class MultiRobotCoordinator:
    """多机器人协调器 / Multi-robot coordinator"""
    
    def allocate_waypoints(self, robots: List[str]):
        """分配路点给不同机器人"""
        
    def synchronize_positions(self):
        """同步多个机器人位置，避免碰撞"""
```

### 8.2 用户界面

#### 8.2.1 Web控制面板

**功能**:
- 地图可视化
- 实时位置显示
- 路点拖拽编辑
- 任务控制按钮
- 日志查看

**技术栈**:
- 前端: React + ROS2 Web Bridge
- 后端: FastAPI
- 通信: WebSocket

#### 8.2.2 移动端APP

**功能**:
- 远程监控
- 语音控制
- 路点快速添加
- 紧急停止

### 8.3 智能优化

#### 8.3.1 学习优化

```python
class PatrolOptimizer:
    """巡航优化器 / Patrol optimizer"""
    
    def learn_optimal_speed(self):
        """学习最优速度（基于历史数据）"""
        
    def predict_battery_consumption(self):
        """预测电池消耗"""
        
    def schedule_maintenance(self):
        """智能安排维护时间"""
```

#### 8.3.2 异常预测

```python
class AnomalyDetector:
    """异常检测器 / Anomaly detector"""
    
    def detect_navigation_anomaly(self):
        """检测导航异常（如频繁重规划）"""
        
    def predict_failure(self):
        """预测可能的故障"""
```

---

## 9. 参考资料

### 9.1 ROS2 文档

- [Nav2 Documentation](https://navigation.ros.org/)
- [ROS2 Actions](https://docs.ros.org/en/humble/Tutorials/Intermediate/Writing-an-Action-Server-Client/Py.html)
- [TF2 Tutorials](https://docs.ros.org/en/humble/Tutorials/Intermediate/Tf2/Introduction-To-Tf2.html)

### 9.2 算法参考

- Frontier-based Exploration: Yamauchi (1997)
- TSP优化: Google OR-Tools
- 状态机设计: UML State Machine

### 9.3 相关项目

- [explore_lite](https://github.com/robo-friends/explore_lite) - ROS探索包
- [patrol_robot](https://github.com/ct2034/patrol_robot) - 巡航机器人参考实现
- [nav2_simple_commander](https://github.com/ros-planning/navigation2/tree/main/nav2_simple_commander) - Nav2 Python API

---

## 10. 附录

### 10.1 词汇表

| 术语 | 英文 | 说明 |
|------|------|------|
| 路点 | Waypoint | 预定义的导航目标位置 |
| 巡航 | Patrol | 按预定路线重复移动 |
| 停留时间 | Dwell Time | 到达路点后的等待时间 |
| 遍历模式 | Traversal Mode | 路点访问顺序策略 |
| 往返模式 | Ping-pong Mode | 往返遍历路点列表 |
| 事务性 | Transactional | 状态转换的原子性保证 |

### 10.2 常见问题

**Q: 为什么不直接使用Nav2的路点跟随功能？**

A: Nav2的 `FollowWaypoints` action只支持简单的顺序遍历，缺少：
- 多种遍历模式（往返、随机）
- 停留时间控制
- 任务管理和切换
- 与探索功能的集成

**Q: 巡航过程中如何处理动态障碍物？**

A: 依赖Nav2的局部规划器和控制器自动避障。如果无法绕过，会触发恢复行为。

**Q: 路点录制时如何保证精度？**

A: 使用TF系统获取位姿，精度取决于定位系统（里程计+SLAM）的质量。

---

## 变更记录

| 版本 | 日期 | 作者 | 变更内容 |
|------|------|------|---------|
| v1.0 | 2025-12-22 | GitHub Copilot | 初始版本 |

---

**文档状态**: ✅ 设计阶段完成（v2.0 - 扩展版）

**主要更新** (2025-12-22):
- ✅ 新增详细的任务类型定义系统
- ✅ 新增地图库管理器（地图持久化、版本控制）
- ✅ 新增任务管理器（任务持久化、队列、状态机）
- ✅ 增强任务规划器（完整服务接口、语音集成）
- ✅ 扩展巡航模式（新增priority和coverage模式）
- ✅ 完善数据存储结构和文件组织
- ✅ 更新实施计划（从6阶段扩展到8阶段，17-25天）

**与v1.0的主要差异**:
1. **任务系统**：从简单的探索/巡航切换 → 完整的任务管理系统
2. **持久化**：从内存临时 → 完整的文件持久化和恢复
3. **地图管理**：从无 → 完整的地图库和版本控制
4. **任务类型**：从2种 → 13种任务类型
5. **控制方式**：从服务接口 → 服务+语音+Web多通道

**下一步行动**: 根据团队反馈调整设计，开始阶段1实施
