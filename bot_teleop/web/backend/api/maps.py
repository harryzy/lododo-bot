"""
地图管理 API
提供地图库的查询、加载、保存等功能
"""

from fastapi import APIRouter, HTTPException
from pydantic import BaseModel
from typing import List, Optional
import yaml
from pathlib import Path
import os

router = APIRouter()


# ============================================
# 数据模型
# ============================================

class MapInfo(BaseModel):
    """地图信息"""
    name: str
    path: str
    has_rtabmap_db: bool
    has_pgm: bool
    has_yaml: bool
    metadata: Optional[dict] = None


class MapListResponse(BaseModel):
    """地图列表响应"""
    success: bool
    maps: List[MapInfo]


class MapLoadRequest(BaseModel):
    """地图加载请求"""
    map_name: str


class MapSaveRequest(BaseModel):
    """地图保存请求"""
    map_name: str
    description: Optional[str] = None
    tags: Optional[List[str]] = None


# ============================================
# 工具函数
# ============================================

def get_maps_directory() -> Path:
    """获取地图目录"""
    # 默认地图目录
    maps_dir = Path.home() / "lododo_bot" / "maps"
    if not maps_dir.exists():
        maps_dir.mkdir(parents=True, exist_ok=True)
    return maps_dir


def scan_map_library() -> List[MapInfo]:
    """扫描地图库"""
    maps_dir = get_maps_directory()
    map_list = []
    
    if not maps_dir.exists():
        return map_list
    
    # 遍历地图目录
    for map_path in maps_dir.iterdir():
        if not map_path.is_dir():
            continue
        
        # 检查必要文件
        has_rtabmap_db = (map_path / "rtabmap.db").exists()
        has_pgm = (map_path / "map.pgm").exists()
        has_yaml = (map_path / "map.yaml").exists()
        
        # 读取元数据
        metadata_file = map_path / "metadata.yaml"
        metadata = None
        if metadata_file.exists():
            try:
                with open(metadata_file, 'r', encoding='utf-8') as f:
                    metadata = yaml.safe_load(f)
            except Exception:
                pass
        
        map_info = MapInfo(
            name=map_path.name,
            path=str(map_path),
            has_rtabmap_db=has_rtabmap_db,
            has_pgm=has_pgm,
            has_yaml=has_yaml,
            metadata=metadata
        )
        
        map_list.append(map_info)
    
    return map_list


# ============================================
# API 端点
# ============================================

@router.get("/maps", response_model=MapListResponse)
async def list_maps():
    """
    获取地图列表
    
    扫描地图库，返回所有可用地图
    """
    try:
        maps = scan_map_library()
        return MapListResponse(
            success=True,
            maps=maps
        )
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))


@router.get("/maps/{map_name}")
async def get_map_info(map_name: str):
    """
    获取指定地图的详细信息
    
    Args:
        map_name: 地图名称
    """
    try:
        maps_dir = get_maps_directory()
        map_path = maps_dir / map_name
        
        if not map_path.exists():
            raise HTTPException(status_code=404, detail=f"Map '{map_name}' not found")
        
        # 读取地图配置
        map_yaml_path = map_path / "map.yaml"
        map_config = None
        if map_yaml_path.exists():
            with open(map_yaml_path, 'r', encoding='utf-8') as f:
                map_config = yaml.safe_load(f)
        
        # 读取元数据
        metadata_path = map_path / "metadata.yaml"
        metadata = None
        if metadata_path.exists():
            with open(metadata_path, 'r', encoding='utf-8') as f:
                metadata = yaml.safe_load(f)
        
        return {
            "success": True,
            "name": map_name,
            "path": str(map_path),
            "config": map_config,
            "metadata": metadata,
            "files": {
                "rtabmap_db": (map_path / "rtabmap.db").exists(),
                "pgm": (map_path / "map.pgm").exists(),
                "yaml": (map_path / "map.yaml").exists()
            }
        }
        
    except HTTPException:
        raise
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))


@router.post("/maps/load")
async def load_map(req: MapLoadRequest):
    """
    加载地图
    
    切换到指定的地图（需要重启定位节点）
    """
    try:
        maps_dir = get_maps_directory()
        map_path = maps_dir / req.map_name
        
        if not map_path.exists():
            raise HTTPException(status_code=404, detail=f"Map '{req.map_name}' not found")
        
        if not (map_path / "rtabmap.db").exists():
            raise HTTPException(
                status_code=400,
                detail=f"Map '{req.map_name}' does not have rtabmap.db"
            )
        
        # 实际加载地图需要重启 RTABMap 节点
        # 这里返回成功，实际加载由用户手动重启 launch 文件完成
        return {
            "success": True,
            "message": f"To load map '{req.map_name}', restart launch file with map_name:={req.map_name}",
            "map_name": req.map_name,
            "map_path": str(map_path)
        }
        
    except HTTPException:
        raise
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))


@router.post("/maps/save")
async def save_map(req: MapSaveRequest):
    """
    保存当前地图
    
    保存当前 SLAM 会话的地图到地图库
    """
    try:
        maps_dir = get_maps_directory()
        map_path = maps_dir / req.map_name
        
        # 创建地图目录
        map_path.mkdir(parents=True, exist_ok=True)
        
        # 保存元数据
        if req.description or req.tags:
            metadata = {
                "description": req.description,
                "tags": req.tags or [],
                "created_at": None  # TODO: 添加时间戳
            }
            
            metadata_path = map_path / "metadata.yaml"
            with open(metadata_path, 'w', encoding='utf-8') as f:
                yaml.dump(metadata, f, allow_unicode=True)
        
        # 实际保存由 MissionPlanner 的 save_on_completion 功能完成
        return {
            "success": True,
            "message": f"Map directory created: {map_path}. Map will be saved by exploration task.",
            "map_name": req.map_name,
            "map_path": str(map_path)
        }
        
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))


@router.delete("/maps/{map_name}")
async def delete_map(map_name: str):
    """
    删除地图
    
    从地图库中删除指定地图
    """
    try:
        maps_dir = get_maps_directory()
        map_path = maps_dir / map_name
        
        if not map_path.exists():
            raise HTTPException(status_code=404, detail=f"Map '{map_name}' not found")
        
        # 删除整个地图目录
        import shutil
        shutil.rmtree(map_path)
        
        return {
            "success": True,
            "message": f"Map '{map_name}' deleted successfully"
        }
        
    except HTTPException:
        raise
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))
