#!/usr/bin/env python3
"""
配置管理 API
提供前端访问配置文件的接口
"""

from fastapi import APIRouter
from typing import Dict, Any
import yaml
from pathlib import Path

router = APIRouter()

# 缓存配置
_config_cache: Dict[str, Any] = {}


def load_config() -> Dict[str, Any]:
    """加载配置文件"""
    global _config_cache
    
    if _config_cache:
        return _config_cache
    
    config_path = Path(__file__).parent.parent.parent.parent / "config" / "web_config.yaml"
    
    if config_path.exists():
        with open(config_path, 'r', encoding='utf-8') as f:
            _config_cache = yaml.safe_load(f)
    else:
        # 默认配置
        _config_cache = {
            'server': {'host': '0.0.0.0', 'port': 8000},
            'websocket': {'ping_interval': 30, 'ping_timeout': 10},
            'rosbridge': {'url': 'ws://localhost:9090'},
            'ui': {'language': 'zh-CN', 'theme': 'light'}
        }
    
    return _config_cache


@router.get("/api/config", response_model=Dict[str, Any])
async def get_config():
    """
    获取前端配置
    
    Returns:
        配置字典（仅返回前端需要的配置项）
    """
    config = load_config()
    
    # 只返回前端需要的配置
    return {
        'server': config.get('server', {}),
        'websocket': config.get('websocket', {}),
        'rosbridge': config.get('rosbridge', {}),
        'ui': config.get('ui', {}),
    }
