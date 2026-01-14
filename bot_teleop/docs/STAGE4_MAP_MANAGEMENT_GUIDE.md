# 阶段4：地图管理功能开发指南

**开始日期**: 2026-01-14  
**预计完成**: 2026-01-15  
**负责人**: AI Assistant + User

---

## 📋 任务概览

### 目标
实现完整的地图管理功能，包括列表、加载、保存、删除，支持多版本管理。

### 优先级
1. 🔴 高优先级：地图列表API（4.1.4）- 后端基础
2. 🔴 高优先级：地图列表组件（4.1.2）- 前端展示
3. 🟡 中优先级：地图加载功能（4.1.5）
4. 🟢 低优先级：缩略图服务（4.1.6）- 可选增强

---

## 🚀 快速开始：4.1 地图列表与加载

### 第一步：后端API开发 (2小时)

#### 1.1 创建地图管理API文件

**文件**: `src/bot_teleop/web/backend/api/maps.py`

```python
"""
地图管理API
提供地图列表、加载、保存、删除功能
"""
from fastapi import APIRouter, HTTPException, Depends
from pydantic import BaseModel
from typing import List, Optional
from datetime import datetime
import os
from pathlib import Path

from ..dependencies import get_node, get_settings
from ..managers.map_manager import MapManager

router = APIRouter(prefix="/api/maps", tags=["maps"])

# ========== Pydantic Models ==========

class MapInfo(BaseModel):
    """地图信息"""
    name: str
    version: int
    size: str  # "10x10m"
    resolution: float  # 0.05
    created_at: str  # ISO 8601
    tags: List[str] = []
    thumbnail_url: Optional[str] = None
    has_rtabmap_db: bool = False

class MapListResponse(BaseModel):
    """地图列表响应"""
    success: bool
    maps: List[MapInfo]
    total: int

class LoadMapRequest(BaseModel):
    """加载地图请求"""
    map_name: str
    version: Optional[int] = None  # 如果为None，加载最新版本

class LoadMapResponse(BaseModel):
    """加载地图响应"""
    success: bool
    message: str
    map_name: str
    version: int

# ========== API Endpoints ==========

@router.get("", response_model=MapListResponse)
async def list_maps(settings = Depends(get_settings)):
    """
    获取地图列表
    
    返回所有保存的地图及其元数据
    """
    try:
        maps_dir = settings.get('paths', {}).get('maps_dir', '~/lododo_bot/maps')
        maps_dir = os.path.expanduser(maps_dir)
        
        # 读取map_library.yaml
        library_file = Path(maps_dir) / 'map_library.yaml'
        
        if not library_file.exists():
            return MapListResponse(success=True, maps=[], total=0)
        
        import yaml
        with open(library_file, 'r') as f:
            library_data = yaml.safe_load(f) or {}
        
        maps_list = []
        for map_name, metadata in library_data.items():
            # 构造MapInfo
            map_info = MapInfo(
                name=map_name,
                version=metadata.get('version', 1),
                size=f"{metadata.get('width', 0)}x{metadata.get('height', 0)}",
                resolution=metadata.get('resolution', 0.05),
                created_at=metadata.get('created_at', ''),
                tags=metadata.get('tags', []),
                thumbnail_url=f"/api/maps/{map_name}/thumbnail",
                has_rtabmap_db=metadata.get('has_rtabmap_db', False)
            )
            maps_list.append(map_info)
        
        return MapListResponse(
            success=True,
            maps=maps_list,
            total=len(maps_list)
        )
    
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))


@router.post("/load", response_model=LoadMapResponse)
async def load_map(
    req: LoadMapRequest,
    node = Depends(get_node)
):
    """
    加载地图
    
    触发ROS地图加载服务
    """
    try:
        # 调用WebTerminalNode的load_map方法
        # TODO: 需要在WebTerminalNode中添加load_map方法
        request_id = node.load_map(
            map_name=req.map_name,
            version=req.version
        )
        
        return LoadMapResponse(
            success=True,
            message=f"Map load request sent: {request_id}",
            map_name=req.map_name,
            version=req.version or 1
        )
    
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))


@router.get("/{map_name}/thumbnail")
async def get_map_thumbnail(
    map_name: str,
    settings = Depends(get_settings)
):
    """
    获取地图缩略图
    
    返回PNG可视化图
    """
    try:
        from fastapi.responses import FileResponse
        
        maps_dir = settings.get('paths', {}).get('maps_dir', '~/lododo_bot/maps')
        maps_dir = os.path.expanduser(maps_dir)
        
        # 查找最新版本的PNG文件
        map_dir = Path(maps_dir) / map_name
        png_files = list(map_dir.glob(f"{map_name}_v*.png"))
        
        if not png_files:
            # 如果没有PNG，返回占位图或404
            raise HTTPException(status_code=404, detail="Thumbnail not found")
        
        # 返回最新版本
        latest_png = sorted(png_files)[-1]
        return FileResponse(latest_png, media_type="image/png")
    
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))
```

#### 1.2 注册路由

**文件**: `src/bot_teleop/web/backend/main.py`

在`app.include_router`部分添加：
```python
from .api import maps
app.include_router(maps.router)
```

#### 1.3 添加WebTerminalNode的load_map方法

**文件**: `src/bot_teleop/web/backend/nodes/web_terminal_node.py`

```python
def load_map(
    self,
    map_name: str,
    version: Optional[int] = None
) -> str:
    """
    加载地图
    
    Args:
        map_name: 地图名称
        version: 地图版本（可选，默认最新）
        
    Returns:
        request_id: 请求ID
    """
    # TODO: 需要在bot_cmd_interface SDK中添加load_map action
    # 当前可以先使用自定义action
    from bot_cmd_interface.sdk import CommandRequest, ActionType
    
    request = CommandRequest(
        action="load_map",  # 自定义action
        params={
            "map_name": map_name,
            "version": version
        }
    )
    
    return self._publish_request(request)
```

---

### 第二步：前端组件开发 (3小时)

#### 2.1 创建MapManager组件

**文件**: `src/bot_teleop/web_frontend/src/components/MapManager/MapManager.tsx`

```typescript
import React, { useState, useEffect } from 'react';
import { Table, Button, Space, message, Modal, Tag, Image } from 'antd';
import {
  ReloadOutlined,
  EyeOutlined,
  DeleteOutlined,
  DownloadOutlined
} from '@ant-design/icons';
import { useTranslation } from 'react-i18next';
import apiService from '../../services/api';

interface MapInfo {
  name: string;
  version: number;
  size: string;
  resolution: number;
  created_at: string;
  tags: string[];
  thumbnail_url?: string;
  has_rtabmap_db: boolean;
}

const MapManager: React.FC = () => {
  const { t } = useTranslation();
  const [maps, setMaps] = useState<MapInfo[]>([]);
  const [loading, setLoading] = useState(false);
  const [loadingMap, setLoadingMap] = useState<string | null>(null);

  useEffect(() => {
    loadMaps();
  }, []);

  const loadMaps = async () => {
    setLoading(true);
    try {
      const response = await apiService.maps.list();
      setMaps(response.maps);
    } catch (error: any) {
      message.error(t('maps.loadFailed') + ': ' + error.message);
    } finally {
      setLoading(false);
    }
  };

  const handleLoadMap = async (mapName: string) => {
    Modal.confirm({
      title: t('maps.loadConfirmTitle'),
      content: t('maps.loadConfirmContent', { mapName }),
      onOk: async () => {
        setLoadingMap(mapName);
        try {
          const response = await apiService.maps.load({ map_name: mapName });
          message.success(t('maps.loadSuccess', { mapName }));
        } catch (error: any) {
          message.error(t('maps.loadFailed') + ': ' + error.message);
        } finally {
          setLoadingMap(null);
        }
      }
    });
  };

  const columns = [
    {
      title: t('maps.thumbnail'),
      dataIndex: 'thumbnail_url',
      key: 'thumbnail',
      width: 100,
      render: (url: string) => (
        <Image
          width={60}
          height={60}
          src={url}
          placeholder={true}
          fallback="/placeholder-map.png"
        />
      )
    },
    {
      title: t('maps.name'),
      dataIndex: 'name',
      key: 'name',
      sorter: (a: MapInfo, b: MapInfo) => a.name.localeCompare(b.name),
    },
    {
      title: t('maps.version'),
      dataIndex: 'version',
      key: 'version',
      width: 80,
      render: (version: number) => `v${version}`
    },
    {
      title: t('maps.size'),
      dataIndex: 'size',
      key: 'size',
      width: 120
    },
    {
      title: t('maps.resolution'),
      dataIndex: 'resolution',
      key: 'resolution',
      width: 100,
      render: (res: number) => `${res}m`
    },
    {
      title: t('maps.createdAt'),
      dataIndex: 'created_at',
      key: 'created_at',
      width: 180,
      render: (date: string) => new Date(date).toLocaleString()
    },
    {
      title: t('maps.tags'),
      dataIndex: 'tags',
      key: 'tags',
      render: (tags: string[]) => (
        <>
          {tags.map(tag => (
            <Tag key={tag} color="blue">{tag}</Tag>
          ))}
        </>
      )
    },
    {
      title: t('common.actions'),
      key: 'actions',
      width: 200,
      render: (_: any, record: MapInfo) => (
        <Space>
          <Button
            type="primary"
            icon={<EyeOutlined />}
            size="small"
            onClick={() => handleLoadMap(record.name)}
            loading={loadingMap === record.name}
          >
            {t('maps.load')}
          </Button>
          <Button
            icon={<DeleteOutlined />}
            size="small"
            danger
          >
            {t('common.delete')}
          </Button>
        </Space>
      )
    }
  ];

  return (
    <div style={{ padding: 24 }}>
      <div style={{ marginBottom: 16, display: 'flex', justifyContent: 'space-between' }}>
        <h2>{t('maps.title')}</h2>
        <Button
          icon={<ReloadOutlined />}
          onClick={loadMaps}
          loading={loading}
        >
          {t('common.refresh')}
        </Button>
      </div>

      <Table
        columns={columns}
        dataSource={maps}
        rowKey="name"
        loading={loading}
        pagination={{
          pageSize: 10,
          showSizeChanger: true,
          pageSizeOptions: ['10', '20', '50']
        }}
      />
    </div>
  );
};

export default MapManager;
```

#### 2.2 添加API服务方法

**文件**: `src/bot_teleop/web_frontend/src/services/api.ts`

```typescript
const maps = {
  // 获取地图列表
  list: () =>
    apiClient.get<any, { success: boolean; maps: any[]; total: number }>('/maps'),

  // 加载地图
  load: (data: { map_name: string; version?: number }) =>
    apiClient.post<any, any>('/maps/load', data),

  // 删除地图
  delete: (mapName: string) =>
    apiClient.delete<any, any>(`/maps/${mapName}`),
};

export default {
  ...existing,
  maps,
};
```

#### 2.3 添加翻译

**zh-CN.json**:
```json
"maps": {
  "title": "地图管理",
  "name": "地图名称",
  "version": "版本",
  "size": "尺寸",
  "resolution": "分辨率",
  "createdAt": "创建时间",
  "tags": "标签",
  "thumbnail": "缩略图",
  "load": "加载",
  "loadSuccess": "地图 {{mapName}} 加载成功",
  "loadFailed": "加载地图失败",
  "loadConfirmTitle": "确认加载地图",
  "loadConfirmContent": "加载地图 {{mapName}} 将中断当前任务，是否继续？"
}
```

---

## ✅ 验收清单

### 后端
- [ ] `/api/maps` 返回正确的地图列表
- [ ] 地图元数据正确解析（版本、尺寸、分辨率）
- [ ] `/api/maps/{name}/thumbnail` 返回PNG图片
- [ ] WebTerminalNode.load_map()正确构造请求

### 前端
- [ ] MapManager组件正确显示地图列表
- [ ] 缩略图正确加载（或显示占位图）
- [ ] "加载"按钮触发确认对话框
- [ ] 加载成功后显示成功消息
- [ ] 表格支持排序和分页

### 集成
- [ ] 点击"加载"后ROS地图切换
- [ ] 前端地图显示更新
- [ ] 错误情况正确提示

---

## 🐛 常见问题

### 问题1：map_library.yaml不存在
**解决**：首次使用时返回空列表，引导用户先进行探索建图

### 问题2：缩略图404
**解决**：返回占位图或自动生成缩略图

### 问题3：加载地图后前端不更新
**解决**：重新订阅/map话题，触发MapCanvas刷新

---

## 📝 下一步

完成4.1后，继续：
- 4.2 地图保存与删除功能
- 集成到顶部导航栏
- 测试多版本地图管理
