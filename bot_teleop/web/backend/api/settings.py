"""
设置管理 API
提供系统配置的读取和修改接口
"""

from fastapi import APIRouter, HTTPException
from pydantic import BaseModel
from typing import Optional, Dict, Any
import yaml
from pathlib import Path

router = APIRouter()


# ============================================
# 数据模型
# ============================================

class SettingsResponse(BaseModel):
    """设置响应"""
    success: bool
    settings: Dict[str, Any]


class SettingsUpdateRequest(BaseModel):
    """设置更新请求"""
    settings: Dict[str, Any]


# ============================================
# 工具函数
# ============================================

def get_config_path() -> Path:
    """获取配置文件路径"""
    # __file__ = .../web/backend/api/settings.py
    # parent: .../web/backend/api
    # parent: .../web/backend
    # parent: .../web
    # parent: .../bot_teleop
    return Path(__file__).parent.parent.parent.parent / "config" / "web_config.yaml"


def load_settings() -> Dict[str, Any]:
    """加载设置"""
    config_path = get_config_path()
    
    if not config_path.exists():
        # 返回默认设置
        return {
            "server": {
                "host": "0.0.0.0",
                "port": 8000
            },
            "ros": {
                "use_sim_time": False
            },
            "ui": {
                "language": "zh-CN",
                "theme": "light"
            }
        }
    
    try:
        with open(config_path, 'r', encoding='utf-8') as f:
            settings = yaml.safe_load(f)
            return settings or {}
    except Exception as e:
        raise ValueError(f"Failed to load settings: {e}")


def save_settings(settings: Dict[str, Any]):
    """保存设置"""
    config_path = get_config_path()
    
    # 确保目录存在
    config_path.parent.mkdir(parents=True, exist_ok=True)
    
    try:
        with open(config_path, 'w', encoding='utf-8') as f:
            yaml.dump(settings, f, allow_unicode=True, sort_keys=False)
    except Exception as e:
        raise ValueError(f"Failed to save settings: {e}")


# ============================================
# API 端点
# ============================================

@router.get("/settings", response_model=SettingsResponse)
async def get_settings():
    """
    获取所有设置
    
    返回当前系统配置
    """
    try:
        settings = load_settings()
        return SettingsResponse(
            success=True,
            settings=settings
        )
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))


@router.get("/settings/{category}")
async def get_settings_by_category(category: str):
    """
    获取指定类别的设置
    
    Args:
        category: 设置类别（如 server, ros, ui）
    """
    try:
        settings = load_settings()
        
        if category not in settings:
            raise HTTPException(status_code=404, detail=f"Category '{category}' not found")
        
        return {
            "success": True,
            "category": category,
            "settings": settings[category]
        }
        
    except HTTPException:
        raise
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))


@router.put("/settings")
async def update_settings(req: SettingsUpdateRequest):
    """
    更新设置
    
    修改系统配置（需要重启服务器生效）
    """
    try:
        # 加载现有设置
        current_settings = load_settings()
        
        # 合并新设置
        for key, value in req.settings.items():
            if isinstance(value, dict) and key in current_settings:
                # 递归合并字典
                current_settings[key].update(value)
            else:
                current_settings[key] = value
        
        # 保存设置
        save_settings(current_settings)
        
        return {
            "success": True,
            "message": "Settings updated successfully. Restart server to apply changes.",
            "settings": current_settings
        }
        
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))


@router.put("/settings/{category}")
async def update_settings_by_category(category: str, settings: Dict[str, Any]):
    """
    更新指定类别的设置
    
    Args:
        category: 设置类别
        settings: 新的设置值
    """
    try:
        # 加载现有设置
        current_settings = load_settings()
        
        # 更新指定类别
        current_settings[category] = settings
        
        # 保存设置
        save_settings(current_settings)
        
        return {
            "success": True,
            "message": f"Settings for '{category}' updated successfully",
            "category": category,
            "settings": settings
        }
        
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))


@router.post("/settings/reset")
async def reset_settings():
    """
    重置设置为默认值
    
    恢复所有设置到初始状态
    """
    try:
        default_settings = {
            "server": {
                "host": "0.0.0.0",
                "port": 8000
            },
            "ros": {
                "use_sim_time": False
            },
            "ui": {
                "language": "zh-CN",
                "theme": "light"
            }
        }
        
        save_settings(default_settings)
        
        return {
            "success": True,
            "message": "Settings reset to default values",
            "settings": default_settings
        }
        
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))


@router.get("/settings/system/info")
async def get_system_info():
    """
    获取系统信息
    
    返回系统运行状态和环境信息
    """
    import platform
    import psutil
    
    try:
        return {
            "success": True,
            "system": {
                "platform": platform.system(),
                "platform_release": platform.release(),
                "platform_version": platform.version(),
                "architecture": platform.machine(),
                "hostname": platform.node(),
                "processor": platform.processor(),
                "cpu_count": psutil.cpu_count(),
                "memory_total": psutil.virtual_memory().total,
                "memory_available": psutil.virtual_memory().available,
                "disk_usage": psutil.disk_usage('/').percent
            }
        }
    except Exception as e:
        raise HTTPException(status_code=500, detail=str(e))
