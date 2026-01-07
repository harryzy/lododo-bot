"""
配置文件加载工具 / Configuration file loader utilities

提供命令配置和地点映射配置的加载功能 /
Provides loading functionality for command config and location map config
"""

import os
import yaml
from typing import Dict, List, Optional, Any, Tuple
from pathlib import Path


class LocationMapLoader:
    """
    地点映射配置加载器 / Location map configuration loader
    
    从YAML文件加载地点映射，将命名地点转换为坐标 /
    Loads location mappings from YAML file, converts named locations to coordinates
    
    Example:
        >>> loader = LocationMapLoader("config/location_map.yaml")
        >>> pose = loader.get_pose("厨房")
        >>> print(pose)  # {'x': 2.5, 'y': 3.0, 'yaw': 1.57}
    """
    
    def __init__(self, config_path: str):
        """
        初始化地点映射加载器 / Initialize location map loader
        
        Args:
            config_path: 配置文件路径 / Configuration file path
            
        Raises:
            FileNotFoundError: 配置文件不存在 / Config file not found
            yaml.YAMLError: YAML格式错误 / Invalid YAML format
        """
        self.config_path = os.path.expanduser(config_path)
        self.locations: Dict[str, Dict[str, Any]] = {}
        self.metadata: Dict[str, Any] = {}
        self._load_config()
    
    def _load_config(self):
        """
        加载配置文件 / Load configuration file
        
        Raises:
            FileNotFoundError: 配置文件不存在 / Config file not found
            yaml.YAMLError: YAML格式错误 / Invalid YAML format
        """
        if not os.path.exists(self.config_path):
            raise FileNotFoundError(f"Location map config file not found: {self.config_path}")
        
        with open(self.config_path, 'r', encoding='utf-8') as f:
            config = yaml.safe_load(f)
        
        if not config:
            raise ValueError(f"Empty location map config file: {self.config_path}")
        
        # 加载地点列表 / Load location list
        locations_list = config.get('locations', [])
        for location in locations_list:
            name = location.get('name')
            if not name:
                continue  # 跳过没有名称的条目 / Skip entries without name
            
            self.locations[name] = {
                'x': float(location.get('x', 0.0)),
                'y': float(location.get('y', 0.0)),
                'yaw': float(location.get('yaw')) if location.get('yaw') is not None else None,
                'description': location.get('description', ''),
                'tags': location.get('tags', [])
            }
        
        # 加载元数据 / Load metadata
        self.metadata = config.get('metadata', {})
    
    def get_pose(self, location_name: str) -> Optional[Dict[str, float]]:
        """
        根据地点名称获取坐标 / Get pose by location name
        
        Args:
            location_name: 地点名称 / Location name
            
        Returns:
            Optional[Dict[str, float]]: 坐标字典 {'x', 'y', 'yaw'（可选）}，
                                       未找到返回None /
                                       Coordinate dict {'x', 'y', 'yaw'(optional)},
                                       None if not found
        """
        location = self.locations.get(location_name)
        if not location:
            return None
        
        pose = {
            'x': location['x'],
            'y': location['y']
        }
        
        if location['yaw'] is not None:
            pose['yaw'] = location['yaw']
        
        return pose
    
    def get_location_info(self, location_name: str) -> Optional[Dict[str, Any]]:
        """
        获取地点完整信息 / Get complete location information
        
        Args:
            location_name: 地点名称 / Location name
            
        Returns:
            Optional[Dict[str, Any]]: 地点信息字典，包含坐标、描述、标签 /
                                     Location info dict with coordinates, description, tags
        """
        return self.locations.get(location_name)
    
    def list_locations(self) -> List[str]:
        """
        列出所有可用地点 / List all available locations
        
        Returns:
            List[str]: 地点名称列表 / List of location names
        """
        return list(self.locations.keys())
    
    def list_locations_by_tag(self, tag: str) -> List[str]:
        """
        根据标签筛选地点 / Filter locations by tag
        
        Args:
            tag: 标签名称 / Tag name
            
        Returns:
            List[str]: 包含该标签的地点名称列表 / List of location names with the tag
        """
        return [
            name for name, info in self.locations.items()
            if tag in info.get('tags', [])
        ]
    
    def validate_location(self, location_name: str) -> Tuple[bool, str]:
        """
        验证地点名称是否存在 / Validate if location name exists
        
        Args:
            location_name: 地点名称 / Location name
            
        Returns:
            Tuple[bool, str]: (是否存在, 错误信息) / (exists, error_message)
        """
        if location_name in self.locations:
            return True, "OK"
        else:
            return False, f"Location '{location_name}' not found in location map"
    
    def get_coordinate_frame(self) -> str:
        """
        获取坐标系名称 / Get coordinate frame name
        
        Returns:
            str: 坐标系名称，默认"map" / Coordinate frame name, default "map"
        """
        return self.metadata.get('coordinate_frame', 'map')
    
    def reload(self):
        """
        重新加载配置文件 / Reload configuration file
        
        用于配置文件更新后刷新 / Used to refresh after config file update
        """
        self.locations.clear()
        self.metadata.clear()
        self._load_config()


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
