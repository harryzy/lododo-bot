"""
Settings API - System Settings Management
Provides system configuration read, save, and reset functionality
"""
from fastapi import APIRouter, HTTPException
from pydantic import BaseModel
from typing import Dict, Any, Optional
import yaml
import os
import time
from datetime import datetime, timedelta
from pathlib import Path

# psutil is optional dependency for system information
try:
    import psutil
    HAS_PSUTIL = True
except ImportError:
    HAS_PSUTIL = False
    print("Warning: psutil not available, system info features will be limited")

router = APIRouter()

# Configuration file path
CONFIG_DIR = Path.home() / "lododo_bot" / "config"
USER_PREFS_FILE = CONFIG_DIR / "user_preferences.yaml"

# Ensure config directory exists
CONFIG_DIR.mkdir(parents=True, exist_ok=True)


class SettingsData(BaseModel):
    """Settings data model"""
    basic: Dict[str, Any]
    navigation: Dict[str, Any]
    debug: Dict[str, Any]
    performance: Dict[str, Any]


class SystemInfo(BaseModel):
    """System information data model"""
    robot_name: str
    ros_version: str
    system_uptime: str
    websocket_status: str
    rosbridge_status: str
    api_endpoint: str
    disk_usage: Dict[str, str]


# Default configuration
DEFAULT_SETTINGS = {
    "basic": {
        "language": "zh-CN",
        "theme": "light",
        "map_resolution": 0.05,
        "update_rates": {
            "map": 5,
            "pose": 10,
            "costmap": 2
        }
    },
    "navigation": {
        "max_linear_vel": 0.5,
        "max_angular_vel": 1.0,
        "obstacle_safety_dist": 0.3,
        "docking_dist": 0.1,
        "navigation_timeout": 300
    },
    "debug": {
        "show_trajectory": False,
        "show_costmap": True,
        "show_planned_path": True,
        "log_level": "INFO"
    },
    "performance": {
        "canvas_fps_limit": 30,
        "ws_queue_size": 100
    }
}


def load_settings() -> Dict[str, Any]:
    """Load settings from configuration file"""
    if USER_PREFS_FILE.exists():
        try:
            with open(USER_PREFS_FILE, 'r', encoding='utf-8') as f:
                settings = yaml.safe_load(f)
                return settings if settings else DEFAULT_SETTINGS
        except Exception as e:
            print(f"Error loading settings: {e}")
            return DEFAULT_SETTINGS
    return DEFAULT_SETTINGS


def save_settings(settings: Dict[str, Any]) -> bool:
    """Save settings to configuration file"""
    try:
        with open(USER_PREFS_FILE, 'w', encoding='utf-8') as f:
            yaml.dump(settings, f, default_flow_style=False, allow_unicode=True)
        return True
    except Exception as e:
        print(f"Error saving settings: {e}")
        return False


def get_system_uptime() -> str:
    """Get system uptime"""
    if not HAS_PSUTIL:
        return "N/A (psutil not installed)"
    
    try:
        boot_time = psutil.boot_time()
        uptime_seconds = time.time() - boot_time
        uptime = timedelta(seconds=int(uptime_seconds))
        
        days = uptime.days
        hours, remainder = divmod(uptime.seconds, 3600)
        minutes, seconds = divmod(remainder, 60)
        
        if days > 0:
            return f"{days} days {hours} hours {minutes} minutes"
        elif hours > 0:
            return f"{hours} hours {minutes} minutes"
        else:
            return f"{minutes} minutes {seconds} seconds"
    except:
        return "Unknown"


def get_disk_usage(path: str) -> str:
    """Get disk usage for specified path"""
    try:
        if os.path.exists(path):
            total_size = 0
            for dirpath, dirnames, filenames in os.walk(path):
                for filename in filenames:
                    filepath = os.path.join(dirpath, filename)
                    if os.path.exists(filepath):
                        total_size += os.path.getsize(filepath)
            
            # Convert to readable format
            for unit in ['B', 'KB', 'MB', 'GB']:
                if total_size < 1024.0:
                    return f"{total_size:.2f} {unit}"
                total_size /= 1024.0
            return f"{total_size:.2f} TB"
        return "0 B"
    except:
        return "Unknown"


@router.get("/settings")
async def get_settings() -> Dict[str, Any]:
    """Get current settings"""
    settings = load_settings()
    return {
        "success": True,
        "data": settings
    }


@router.post("/settings/save")
async def save_user_settings(settings: SettingsData) -> Dict[str, Any]:
    """Save user settings"""
    settings_dict = settings.dict()
    
    if save_settings(settings_dict):
        return {
            "success": True,
            "message": "Settings saved successfully"
        }
    else:
        raise HTTPException(status_code=500, detail="Failed to save settings")


@router.post("/settings/reset")
async def reset_settings() -> Dict[str, Any]:
    """Reset to default settings"""
    if save_settings(DEFAULT_SETTINGS):
        return {
            "success": True,
            "message": "Settings reset to default",
            "data": DEFAULT_SETTINGS
        }
    else:
        raise HTTPException(status_code=500, detail="Failed to reset settings")


@router.get("/settings/system_info")
async def get_system_info() -> Dict[str, Any]:
    """Get system information"""
    try:
        # Get map and waypoint directory paths
        maps_dir = Path.home() / "lododo_bot" / "maps"
        waypoints_dir = Path.home() / "lododo_bot" / "waypoints"
        
        system_info = {
            "robot_name": "Lododo Robot",
            "ros_version": "ROS2 Humble",
            "system_uptime": get_system_uptime(),
            "websocket_status": "online",  # Frontend will update in real-time
            "rosbridge_status": "online",  # Frontend will update in real-time
            "api_endpoint": "http://localhost:8000",
            "disk_usage": {
                "maps": get_disk_usage(str(maps_dir)),
                "waypoints": get_disk_usage(str(waypoints_dir))
            }
        }
        
        return {
            "success": True,
            "data": system_info
        }
    except Exception as e:
        raise HTTPException(status_code=500, detail=f"Failed to get system info: {str(e)}")
