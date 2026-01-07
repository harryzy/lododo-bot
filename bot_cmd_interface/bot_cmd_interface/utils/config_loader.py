"""
配置文件加载工具 / Configuration file loader utilities

提供命令配置加载功能 / Provides loading functionality for command config
"""

import os
import yaml
from typing import Dict, Any
from pathlib import Path


def load_command_config(config_path: str) -> Dict[str, Any]:
    """
    加载命令配置文件 / Load command configuration file
    
    Args:
        config_path: 配置文件路径 / Configuration file path
        
    Returns:
        Dict[str, Any]: 配置字典 / Configuration dictionary
        
    Raises:
        FileNotFoundError: 配置文件不存在 / Config file not found
        yaml.YAMLError: YAML格式错误 / Invalid YAML format
    """
    config_path = os.path.expanduser(config_path)
    
    if not os.path.exists(config_path):
        raise FileNotFoundError(f"Command config file not found: {config_path}")
    
    with open(config_path, 'r', encoding='utf-8') as f:
        config = yaml.safe_load(f)
    
    if not config:
        raise ValueError(f"Empty command config file: {config_path}")
    
    # 应用默认值 / Apply default values
    config = _apply_default_config(config)
    
    return config


def _apply_default_config(config: Dict[str, Any]) -> Dict[str, Any]:
    """
    应用默认配置值 / Apply default configuration values
    
    Args:
        config: 用户配置字典 / User configuration dictionary
        
    Returns:
        Dict[str, Any]: 合并默认值后的配置 / Configuration with defaults merged
    """
    defaults = {
        'queue': {
            'max_size': 100,
            'dedup_window': 5.0,
            'priority_levels': 5
        },
        'timeout': {
            'default': 300.0,
            'navigation': 300.0,
            'exploration': 3600.0,
            'patrol': 3600.0,
            'query': 5.0,
            'control': 5.0,
            'emergency_stop': 5.0
        },
        'persistence': {
            'enabled': True,
            'mapping_file': '~/.bot_cmd_interface/request_task_map.json',
            'save_interval': 1.0,
            'backup_enabled': True,
            'backup_file': '~/.bot_cmd_interface/request_task_map.json.bak',
            'format_version': '1.0.0'
        },
        'services': {
            'wait_timeout': 10.0,
            'call_timeout': 30.0,
            'retry': {
                'enabled': True,
                'max_attempts': 3,
                'interval': 2.0,
                'retryable_codes': [503, 504]
            }
        },
        'logging': {
            'level': 'INFO',
            'request_logging': True,
            'response_logging': True,
            'queue_stats_interval': 60.0,
            'performance_logging': True
        },
        'security': {
            'max_request_size': 10240,
            'rate_limiting': {
                'enabled': False,
                'max_requests_per_second': 10,
                'burst_size': 20
            }
        },
        'ros2': {
            'qos_profile': {
                'request_topic': 10,
                'response_topic': 10
            },
            'spin_rate': 10.0
        },
        'debug': {
            'enabled': False,
            'verbose_validation': False,
            'save_raw_messages': False,
            'message_dump_dir': '~/.bot_cmd_interface/debug/'
        }
    }
    
    # 深度合并配置 / Deep merge configuration
    return _deep_merge(defaults, config)


def _deep_merge(default: Dict, user: Dict) -> Dict:
    """
    深度合并两个字典 / Deep merge two dictionaries
    
    Args:
        default: 默认配置 / Default configuration
        user: 用户配置 / User configuration
        
    Returns:
        Dict: 合并后的配置 / Merged configuration
    """
    result = default.copy()
    
    for key, value in user.items():
        if key in result and isinstance(result[key], dict) and isinstance(value, dict):
            result[key] = _deep_merge(result[key], value)
        else:
            result[key] = value
    
    return result


def create_default_config_file(config_path: str):
    """
    创建默认配置文件 / Create default configuration file
    
    如果配置文件不存在，创建一个包含默认值的配置文件 /
    If config file doesn't exist, create one with default values
    
    Args:
        config_path: 配置文件路径 / Configuration file path
    """
    config_path = os.path.expanduser(config_path)
    
    if os.path.exists(config_path):
        return  # 文件已存在，不覆盖 / File exists, don't overwrite
    
    # 确保目录存在 / Ensure directory exists
    config_dir = os.path.dirname(config_path)
    if config_dir:
        Path(config_dir).mkdir(parents=True, exist_ok=True)
    
    # 获取默认配置 / Get default configuration
    default_config = _apply_default_config({})
    
    # 写入文件 / Write to file
    with open(config_path, 'w', encoding='utf-8') as f:
        yaml.dump(default_config, f, default_flow_style=False, allow_unicode=True)
