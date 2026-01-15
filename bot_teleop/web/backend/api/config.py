#!/usr/bin/env python3
"""
Configuration Management API
Provides interface for frontend to access configuration files
"""

from fastapi import APIRouter
from typing import Dict, Any
import yaml
from pathlib import Path

router = APIRouter()

# Configuration cache
_config_cache: Dict[str, Any] = {}


def load_config() -> Dict[str, Any]:
    """Load configuration file"""
    global _config_cache
    
    if _config_cache:
        return _config_cache
    
    config_path = Path(__file__).parent.parent.parent.parent / "config" / "web_config.yaml"
    
    if config_path.exists():
        with open(config_path, 'r', encoding='utf-8') as f:
            _config_cache = yaml.safe_load(f)
    else:
        # Default configuration
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
    Get frontend configuration
    
    Returns:
        Configuration dictionary (only returns configuration items needed by frontend)
    """
    config = load_config()
    
    # Only return configuration needed by frontend
    return {
        'server': config.get('server', {}),
        'websocket': config.get('websocket', {}),
        'rosbridge': config.get('rosbridge', {}),
        'ui': config.get('ui', {}),
    }
