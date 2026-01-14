"""
地图管理 API
提供地图库的查询、加载、保存等功能
"""

from fastapi import APIRouter, HTTPException
from fastapi.responses import FileResponse
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
    """地图信息 - 支持多版本管理"""
    name: str
    version: int  # 当前活动版本
    versions: List[int] = []  # 所有可用版本列表
    size: str
    resolution: float
    created_at: str
    updated_at: Optional[str] = None
    description: Optional[str] = None
    tags: List[str] = []
    thumbnail_url: Optional[str] = None
    has_rtabmap_db: bool = False
    has_pgm: bool = False
    has_yaml: bool = False


class MapListResponse(BaseModel):
    """地图列表响应"""
    success: bool
    maps: List[MapInfo]
    total: int


class MapLoadRequest(BaseModel):
    """地图加载请求"""
    map_name: str
    version: Optional[int] = None


class MapSaveRequest(BaseModel):
    """地图保存请求"""
    map_name: str
    description: Optional[str] = None
    tags: Optional[List[str]] = None


class DeleteMapResponse(BaseModel):
    """删除地图响应"""
    success: bool
    message: str


class VersionInfo(BaseModel):
    """版本信息"""
    version: int
    is_current: bool
    size: str
    has_rtabmap_db: bool
    has_pgm: bool
    has_yaml: bool
    created_at: Optional[str] = None


class VersionListResponse(BaseModel):
    """版本列表响应"""
    success: bool
    map_name: str
    current_version: int
    versions: List[VersionInfo]


class SwitchVersionRequest(BaseModel):
    """切换版本请求"""
    version: int


class SwitchVersionResponse(BaseModel):
    """切换版本响应"""
    success: bool
    message: str
    new_version: int


class VersionInfo(BaseModel):
    """版本信息"""
    version: int
    is_current: bool
    size: str
    has_rtabmap_db: bool
    has_pgm: bool
    has_yaml: bool
    created_at: Optional[str] = None


class VersionListResponse(BaseModel):
    """版本列表响应"""
    success: bool
    map_name: str
    current_version: int
    versions: List[VersionInfo]


class SwitchVersionRequest(BaseModel):
    """切换版本请求"""
    version: int


class SwitchVersionResponse(BaseModel):
    """切换版本响应"""
    success: bool
    message: str
    new_version: int


# ============================================
# 工具函数
# ============================================

def get_maps_directory() -> Path:
    """获取地图目录 - 使用工作空间相对路径"""
    # 优先使用环境变量
    maps_dir_str = os.environ.get('LODODO_MAPS_DIR', '')
    if maps_dir_str:
        maps_dir = Path(maps_dir_str).expanduser().resolve()
        print(f"[MapAPI] Using LODODO_MAPS_DIR: {maps_dir}")
        return maps_dir
    
    # 从当前文件向上查找工作空间根目录
    # 当前文件在: workspace/src/bot_teleop/web/backend/api/maps.py
    current_file = Path(__file__).resolve()
    
    # 向上查找，直到找到包含 src/ 和 install/ 的目录（ROS2工作空间特征）
    workspace_root = current_file
    for _ in range(10):  # 最多向上10层
        workspace_root = workspace_root.parent
        if (workspace_root / 'src').exists() and (workspace_root / 'install').exists():
            print(f"[MapAPI] Found workspace root: {workspace_root}")
            break
        if workspace_root == workspace_root.parent:  # 到达根目录
            break
    else:
        # 未找到，使用当前工作目录
        workspace_root = Path.cwd()
        print(f"[MapAPI] Using current working directory: {workspace_root}")
    
    # 地图目录在工作空间根目录下的 maps/
    maps_dir = workspace_root / "maps"
    
    if not maps_dir.exists():
        maps_dir.mkdir(parents=True, exist_ok=True)
    
    print(f"[MapAPI] Maps directory: {maps_dir}")
    return maps_dir


def scan_map_library() -> List[MapInfo]:
    """扫描地图库 - 支持版本管理"""
    maps_dir = get_maps_directory()
    map_list = []
    
    library_file = maps_dir / "map_library.yaml"
    
    if not library_file.exists():
        return map_list
    
    try:
        with open(library_file, 'r', encoding='utf-8') as f:
            library_data = yaml.safe_load(f) or {}
        
        print(f"[MapAPI] 原始library_data: {library_data}")
        maps_data = library_data.get('maps', {})
        print(f"[MapAPI] 找到 {len(maps_data)} 个地图: {list(maps_data.keys())}")
        
        for map_name, metadata in maps_data.items():
            map_dir = maps_dir / map_name
            version = metadata.get('version', 1)
            print(f"[MapAPI] 处理地图: {map_name}, 版本: {version}")
            
            # 支持多种数据库文件命名
            has_rtabmap_db = (
                (map_dir / f"rtabmap_v{version}.db").exists() or 
                (map_dir / "rtabmap.db").exists() or
                (map_dir / f"{map_name}_v{version}.db").exists()
            )
            has_pgm = (map_dir / f"{map_name}_v{version}.pgm").exists() or (map_dir / "map.pgm").exists()
            has_yaml = (map_dir / f"{map_name}_v{version}.yaml").exists() or (map_dir / "map.yaml").exists()
            print(f"[MapAPI]   - DB: {has_rtabmap_db}, PGM: {has_pgm}, YAML: {has_yaml}")
            
            # 尝试从PGM文件读取实际尺寸
            width = metadata.get('width', 0)
            height = metadata.get('height', 0)
            
            if (width == 0 or height == 0) and has_pgm:
                try:
                    pgm_file = map_dir / f"{map_name}_v{version}.pgm"
                    if not pgm_file.exists():
                        pgm_file = map_dir / "map.pgm"
                    
                    if pgm_file.exists():
                        with open(pgm_file, 'rb') as f:
                            # 读取PGM头
                            header = f.readline().decode('ascii').strip()  # P5
                            if header == 'P5':
                                # 跳过注释行
                                line = f.readline().decode('ascii').strip()
                                while line.startswith('#'):
                                    line = f.readline().decode('ascii').strip()
                                # 读取宽高
                                parts = line.split()
                                if len(parts) >= 2:
                                    width = int(parts[0])
                                    height = int(parts[1])
                except Exception as e:
                    print(f"[MapAPI] 读取PGM尺寸失败 {map_name}: {e}")
            
            # 获取所有可用版本列表
            versions = metadata.get('versions', [version])
            
            map_info = MapInfo(
                name=map_name,
                version=version,
                versions=versions,
                size=f"{width}x{height}",
                resolution=metadata.get('resolution', 0.05),
                created_at=metadata.get('created_at', ''),
                updated_at=metadata.get('updated_at'),
                description=metadata.get('description', ''),
                tags=metadata.get('tags', []),
                thumbnail_url=f"/api/maps/{map_name}/thumbnail",
                has_rtabmap_db=has_rtabmap_db,
                has_pgm=has_pgm,
                has_yaml=has_yaml
            )
            
            map_list.append(map_info)
            print(f"[MapAPI]   -> 已添加地图: {map_name}")
        
        print(f"[MapAPI] 最终返回 {len(map_list)} 个地图")
        return map_list
    
    except Exception as e:
        print(f"[MapAPI] 扫描地图库失败: {e}")
        return map_list


# ============================================
# API 端点
# ============================================

@router.get("/maps", response_model=MapListResponse)
async def list_maps():
    """获取地图列表"""
    try:
        maps = scan_map_library()
        return MapListResponse(
            success=True,
            maps=maps,
            total=len(maps)
        )
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))


@router.get("/maps/{map_name}")
async def get_map_info(map_name: str):
    """获取指定地图的详细信息"""
    try:
        maps_dir = get_maps_directory()
        map_path = maps_dir / map_name
        
        if not map_path.exists():
            raise HTTPException(status_code=404, detail=f"Map '{map_name}' not found")
        
        map_yaml_path = map_path / "map.yaml"
        map_config = None
        if map_yaml_path.exists():
            with open(map_yaml_path, 'r', encoding='utf-8') as f:
                map_config = yaml.safe_load(f)
        
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
    """加载地图"""
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
    """保存当前地图"""
    try:
        maps_dir = get_maps_directory()
        map_path = maps_dir / req.map_name
        
        map_path.mkdir(parents=True, exist_ok=True)
        
        if req.description or req.tags:
            metadata = {
                "description": req.description,
                "tags": req.tags or [],
                "created_at": None
            }
            
            metadata_path = map_path / "metadata.yaml"
            with open(metadata_path, 'w', encoding='utf-8') as f:
                yaml.dump(metadata, f, allow_unicode=True)
        
        return {
            "success": True,
            "message": f"Map directory created: {map_path}. Map will be saved by exploration task.",
            "map_name": req.map_name,
            "map_path": str(map_path)
        }
        
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))


@router.delete("/maps/{map_name}", response_model=DeleteMapResponse)
async def delete_map(map_name: str):
    """删除地图"""
    try:
        maps_dir = get_maps_directory()
        map_path = maps_dir / map_name
        
        if not map_path.exists():
            raise HTTPException(status_code=404, detail=f"Map '{map_name}' not found")
        
        import shutil
        shutil.rmtree(map_path)
        
        library_file = maps_dir / "map_library.yaml"
        if library_file.exists():
            with open(library_file, 'r', encoding='utf-8') as f:
                library_data = yaml.safe_load(f) or {}
            
            if 'maps' in library_data and map_name in library_data['maps']:
                del library_data['maps'][map_name]
                
                with open(library_file, 'w', encoding='utf-8') as f:
                    yaml.dump(library_data, f, default_flow_style=False, allow_unicode=True)
        
        return DeleteMapResponse(
            success=True,
            message=f"Map '{map_name}' deleted successfully"
        )
        
    except HTTPException:
        raise
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))


@router.get("/maps/{map_name}/versions", response_model=VersionListResponse)
async def get_map_versions(map_name: str):
    """获取地图的所有版本详情"""
    try:
        maps_dir = get_maps_directory()
        library_file = maps_dir / "map_library.yaml"
        
        if not library_file.exists():
            raise HTTPException(status_code=404, detail="Map library not found")
        
        with open(library_file, 'r', encoding='utf-8') as f:
            library_data = yaml.safe_load(f) or {}
        
        maps_data = library_data.get('maps', {})
        if map_name not in maps_data:
            raise HTTPException(status_code=404, detail=f"Map '{map_name}' not found")
        
        metadata = maps_data[map_name]
        current_version = metadata.get('version', 1)
        versions = metadata.get('versions', [current_version])
        
        map_dir = maps_dir / map_name
        version_infos = []
        
        for ver in versions:
            # 检查每个版本的文件
            has_db = (
                (map_dir / f"rtabmap_v{ver}.db").exists() or
                (map_dir / f"{map_name}_v{ver}.db").exists()
            )
            has_pgm = (map_dir / f"{map_name}_v{ver}.pgm").exists()
            has_yaml = (map_dir / f"{map_name}_v{ver}.yaml").exists()
            
            # 读取尺寸
            width, height = 0, 0
            if has_pgm:
                try:
                    pgm_file = map_dir / f"{map_name}_v{ver}.pgm"
                    with open(pgm_file, 'rb') as f:
                        header = f.readline().decode('ascii').strip()
                        if header == 'P5':
                            line = f.readline().decode('ascii').strip()
                            while line.startswith('#'):
                                line = f.readline().decode('ascii').strip()
                            parts = line.split()
                            if len(parts) >= 2:
                                width, height = int(parts[0]), int(parts[1])
                except Exception:
                    pass
            
            version_infos.append(VersionInfo(
                version=ver,
                is_current=(ver == current_version),
                size=f"{width}x{height}",
                has_rtabmap_db=has_db,
                has_pgm=has_pgm,
                has_yaml=has_yaml
            ))
        
        return VersionListResponse(
            success=True,
            map_name=map_name,
            current_version=current_version,
            versions=version_infos
        )
    
    except HTTPException:
        raise
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))


@router.post("/maps/{map_name}/switch_version", response_model=SwitchVersionResponse)
async def switch_map_version(map_name: str, req: SwitchVersionRequest):
    """切换地图的当前版本"""
    try:
        maps_dir = get_maps_directory()
        library_file = maps_dir / "map_library.yaml"
        
        if not library_file.exists():
            raise HTTPException(status_code=404, detail="Map library not found")
        
        with open(library_file, 'r', encoding='utf-8') as f:
            library_data = yaml.safe_load(f) or {}
        
        maps_data = library_data.get('maps', {})
        if map_name not in maps_data:
            raise HTTPException(status_code=404, detail=f"Map '{map_name}' not found")
        
        metadata = maps_data[map_name]
        versions = metadata.get('versions', [])
        
        if req.version not in versions:
            raise HTTPException(
                status_code=400,
                detail=f"Version {req.version} not found for map '{map_name}'"
            )
        
        # 更新当前版本
        metadata['version'] = req.version
        metadata['file_path'] = f"{maps_dir / map_name / map_name}_v{req.version}"
        
        # 保存更新
        with open(library_file, 'w', encoding='utf-8') as f:
            yaml.dump(library_data, f, default_flow_style=False, allow_unicode=True)
        
        return SwitchVersionResponse(
            success=True,
            message=f"Switched to version {req.version}",
            new_version=req.version
        )
    
    except HTTPException:
        raise
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))


@router.delete("/maps/{map_name}/versions/{version}")
async def delete_map_version(map_name: str, version: int):
    """删除地图的特定版本"""
    try:
        maps_dir = get_maps_directory()
        library_file = maps_dir / "map_library.yaml"
        
        if not library_file.exists():
            raise HTTPException(status_code=404, detail="Map library not found")
        
        with open(library_file, 'r', encoding='utf-8') as f:
            library_data = yaml.safe_load(f) or {}
        
        maps_data = library_data.get('maps', {})
        if map_name not in maps_data:
            raise HTTPException(status_code=404, detail=f"Map '{map_name}' not found")
        
        metadata = maps_data[map_name]
        current_version = metadata.get('version', 1)
        versions = metadata.get('versions', [])
        
        if version not in versions:
            raise HTTPException(status_code=404, detail=f"Version {version} not found")
        
        if version == current_version and len(versions) > 1:
            raise HTTPException(
                status_code=400,
                detail="Cannot delete current version. Please switch to another version first."
            )
        
        # 删除文件
        map_dir = maps_dir / map_name
        deleted_files = []
        for pattern in [f"{map_name}_v{version}.db", f"{map_name}_v{version}.pgm", 
                       f"{map_name}_v{version}.yaml", f"{map_name}_v{version}_preview.png",
                       f"rtabmap_v{version}.db"]:
            file_path = map_dir / pattern
            if file_path.exists():
                file_path.unlink()
                deleted_files.append(pattern)
        
        # 从versions列表移除
        versions.remove(version)
        metadata['versions'] = versions
        
        # 如果是最后一个版本，删除整个地图
        if len(versions) == 0:
            del maps_data[map_name]
            import shutil
            shutil.rmtree(map_dir)
        
        # 保存更新
        with open(library_file, 'w', encoding='utf-8') as f:
            yaml.dump(library_data, f, default_flow_style=False, allow_unicode=True)
        
        return {
            "success": True,
            "message": f"Deleted version {version}, removed {len(deleted_files)} files",
            "deleted_files": deleted_files
        }
    
    except HTTPException:
        raise
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))


@router.get("/maps/{map_name}/versions", response_model=VersionListResponse)
async def get_map_versions(map_name: str):
    """获取地图的所有版本详情"""
    try:
        maps_dir = get_maps_directory()
        library_file = maps_dir / "map_library.yaml"
        
        if not library_file.exists():
            raise HTTPException(status_code=404, detail="Map library not found")
        
        with open(library_file, 'r', encoding='utf-8') as f:
            library_data = yaml.safe_load(f) or {}
        
        maps_data = library_data.get('maps', {})
        if map_name not in maps_data:
            raise HTTPException(status_code=404, detail=f"Map '{map_name}' not found")
        
        metadata = maps_data[map_name]
        current_version = metadata.get('version', 1)
        versions = metadata.get('versions', [current_version])
        
        map_dir = maps_dir / map_name
        version_infos = []
        
        for ver in versions:
            # 检查每个版本的文件
            has_db = (
                (map_dir / f"rtabmap_v{ver}.db").exists() or
                (map_dir / f"{map_name}_v{ver}.db").exists()
            )
            has_pgm = (map_dir / f"{map_name}_v{ver}.pgm").exists()
            has_yaml = (map_dir / f"{map_name}_v{ver}.yaml").exists()
            
            # 读取尺寸
            width, height = 0, 0
            if has_pgm:
                try:
                    pgm_file = map_dir / f"{map_name}_v{ver}.pgm"
                    with open(pgm_file, 'rb') as f:
                        header = f.readline().decode('ascii').strip()
                        if header == 'P5':
                            line = f.readline().decode('ascii').strip()
                            while line.startswith('#'):
                                line = f.readline().decode('ascii').strip()
                            parts = line.split()
                            if len(parts) >= 2:
                                width, height = int(parts[0]), int(parts[1])
                except Exception:
                    pass
            
            version_infos.append(VersionInfo(
                version=ver,
                is_current=(ver == current_version),
                size=f"{width}x{height}",
                has_rtabmap_db=has_db,
                has_pgm=has_pgm,
                has_yaml=has_yaml
            ))
        
        return VersionListResponse(
            success=True,
            map_name=map_name,
            current_version=current_version,
            versions=version_infos
        )
    
    except HTTPException:
        raise
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))


@router.post("/maps/{map_name}/switch_version", response_model=SwitchVersionResponse)
async def switch_map_version(map_name: str, req: SwitchVersionRequest):
    """切换地图的当前版本"""
    try:
        maps_dir = get_maps_directory()
        library_file = maps_dir / "map_library.yaml"
        
        if not library_file.exists():
            raise HTTPException(status_code=404, detail="Map library not found")
        
        with open(library_file, 'r', encoding='utf-8') as f:
            library_data = yaml.safe_load(f) or {}
        
        maps_data = library_data.get('maps', {})
        if map_name not in maps_data:
            raise HTTPException(status_code=404, detail=f"Map '{map_name}' not found")
        
        metadata = maps_data[map_name]
        versions = metadata.get('versions', [])
        
        if req.version not in versions:
            raise HTTPException(
                status_code=400,
                detail=f"Version {req.version} not found for map '{map_name}'"
            )
        
        # 更新当前版本
        metadata['version'] = req.version
        metadata['file_path'] = f"{maps_dir / map_name / map_name}_v{req.version}"
        
        # 保存更新
        with open(library_file, 'w', encoding='utf-8') as f:
            yaml.dump(library_data, f, default_flow_style=False, allow_unicode=True)
        
        return SwitchVersionResponse(
            success=True,
            message=f"Switched to version {req.version}",
            new_version=req.version
        )
    
    except HTTPException:
        raise
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))


@router.delete("/maps/{map_name}/versions/{version}")
async def delete_map_version(map_name: str, version: int):
    """删除地图的特定版本"""
    try:
        maps_dir = get_maps_directory()
        library_file = maps_dir / "map_library.yaml"
        
        if not library_file.exists():
            raise HTTPException(status_code=404, detail="Map library not found")
        
        with open(library_file, 'r', encoding='utf-8') as f:
            library_data = yaml.safe_load(f) or {}
        
        maps_data = library_data.get('maps', {})
        if map_name not in maps_data:
            raise HTTPException(status_code=404, detail=f"Map '{map_name}' not found")
        
        metadata = maps_data[map_name]
        current_version = metadata.get('version', 1)
        versions = metadata.get('versions', [])
        
        if version not in versions:
            raise HTTPException(status_code=404, detail=f"Version {version} not found")
        
        if version == current_version and len(versions) > 1:
            raise HTTPException(
                status_code=400,
                detail="Cannot delete current version. Please switch to another version first."
            )
        
        # 删除文件
        map_dir = maps_dir / map_name
        deleted_files = []
        for pattern in [f"{map_name}_v{version}.db", f"{map_name}_v{version}.pgm", 
                       f"{map_name}_v{version}.yaml", f"{map_name}_v{version}_preview.png",
                       f"rtabmap_v{version}.db"]:
            file_path = map_dir / pattern
            if file_path.exists():
                file_path.unlink()
                deleted_files.append(pattern)
        
        # 从versions列表移除
        versions.remove(version)
        metadata['versions'] = versions
        
        # 如果是最后一个版本，删除整个地图
        if len(versions) == 0:
            del maps_data[map_name]
            import shutil
            shutil.rmtree(map_dir)
        
        # 保存更新
        with open(library_file, 'w', encoding='utf-8') as f:
            yaml.dump(library_data, f, default_flow_style=False, allow_unicode=True)
        
        return {
            "success": True,
            "message": f"Deleted version {version}, removed {len(deleted_files)} files",
            "deleted_files": deleted_files
        }
    
    except HTTPException:
        raise
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))


@router.get("/maps/{map_name}/thumbnail")
async def get_map_thumbnail(map_name: str):
    """获取地图缩略图"""
    try:
        maps_dir = get_maps_directory()
        map_dir = maps_dir / map_name
        
        if not map_dir.exists():
            raise HTTPException(status_code=404, detail="Map not found")
        
        # 查找PNG文件（支持_preview.png和普通.png）
        preview_files = list(map_dir.glob(f"{map_name}_v*_preview.png"))
        png_files = list(map_dir.glob(f"{map_name}_v*.png"))
        
        # 优先使用preview文件
        all_png_files = preview_files if preview_files else png_files
        
        if not all_png_files:
            # 尝试旧格式
            legacy_png = map_dir / f"{map_name}.png"
            if legacy_png.exists():
                return FileResponse(legacy_png, media_type="image/png")
            
            raise HTTPException(status_code=404, detail="Thumbnail not found")
        
        # 返回最新版本（按文件名排序）
        latest_png = sorted(all_png_files, key=lambda p: p.name)[-1]
        return FileResponse(latest_png, media_type="image/png")
        
    except HTTPException:
        raise
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))
