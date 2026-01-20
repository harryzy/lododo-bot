#!/usr/bin/env python3
"""
PathManager - 相对路径管理工具类 / Relative path management utility

本模块提供统一的路径解析功能，确保所有配置文件中的路径都是相对路径。
This module provides unified path resolution to ensure all paths in config files are relative.

参考设计文档 / Reference: HARDWARE_DEPLOYMENT_DESIGN.md §1.4.4 (Line 1075-1422)

Author: Hurry
Created: 2026-01-19
"""

import os
from pathlib import Path
from typing import Dict, List, Optional, Union
from ament_index_python.packages import get_package_share_directory


class PathManager:
    """
    路径管理器类 / Path Manager Class
    
    职责 / Responsibilities:
    1. 解析相对路径到绝对路径 / Resolve relative paths to absolute paths
    2. 验证配置文件中的路径 / Validate paths in config files
    3. 检测绝对路径使用 / Detect absolute path usage
    4. 提供工作空间根目录访问 / Provide workspace root access
    """
    
    def __init__(self, package_name: str = 'bot_hardware'):
        """
        初始化路径管理器 / Initialize PathManager
        
        Args:
            package_name: ROS2包名 / ROS2 package name
        """
        # 获取包的share目录 / Get package share directory
        try:
            self._package_share_dir = get_package_share_directory(package_name)
        except Exception:
            # 开发环境回退方案：使用源码目录 / Development fallback: use source directory
            current_file = Path(__file__).resolve()
            self._package_share_dir = str(current_file.parent.parent.parent)
        
        # 计算工作空间根目录 / Calculate workspace root directory
        # 假设结构: workspace_root/src/bot_hardware/... 或 workspace_root/install/bot_hardware/...
        # Assume structure: workspace_root/src/bot_hardware/... or workspace_root/install/bot_hardware/...
        share_path = Path(self._package_share_dir)
        
        if 'install' in share_path.parts:
            # 安装后环境 / Installed environment
            # .../install/bot_hardware/share/bot_hardware -> .../
            self._workspace_root = share_path.parents[3]
        else:
            # 源码环境 / Source environment
            # .../src/bot_hardware -> .../
            self._workspace_root = share_path.parents[1]
    
    @property
    def workspace_root(self) -> Path:
        """
        获取工作空间根目录 / Get workspace root directory
        
        Returns:
            Path: 工作空间根目录路径对象 / Workspace root path object
        """
        return self._workspace_root
    
    @property
    def package_share_dir(self) -> str:
        """
        获取包的share目录 / Get package share directory
        
        Returns:
            str: 包share目录的绝对路径 / Absolute path of package share directory
        """
        return self._package_share_dir
    
    def resolve_path(self, relative_path: str, base: str = 'workspace') -> Path:
        """
        解析相对路径到绝对路径 / Resolve relative path to absolute path
        
        Args:
            relative_path: 相对路径字符串 / Relative path string
            base: 基准目录，可选值 'workspace' | 'package' / Base directory, options: 'workspace' | 'package'
        
        Returns:
            Path: 解析后的绝对路径对象 / Resolved absolute path object
        
        Raises:
            ValueError: 如果输入是绝对路径 / If input is absolute path
        
        Examples:
            >>> pm = PathManager()
            >>> # 解析相对于工作空间根目录的路径 / Resolve path relative to workspace root
            >>> pm.resolve_path('maps/office_floor1')
            PosixPath('/home/user/lododo_bot/maps/office_floor1')
            
            >>> # 解析相对于包share目录的路径 / Resolve path relative to package share
            >>> pm.resolve_path('config/hardware_config.yaml', base='package')
            PosixPath('/home/user/lododo_bot/install/bot_hardware/share/bot_hardware/config/hardware_config.yaml')
        """
        # 检查是否为绝对路径 / Check if absolute path
        path_obj = Path(relative_path)
        if path_obj.is_absolute():
            raise ValueError(
                f"Absolute path detected: '{relative_path}'. "
                f"Please use relative path in configuration files. "
                f"检测到绝对路径: '{relative_path}'。请在配置文件中使用相对路径。"
            )
        
        # 检查是否使用了~扩展 / Check if using ~ expansion
        if relative_path.startswith('~'):
            raise ValueError(
                f"Home directory expansion '~' detected in: '{relative_path}'. "
                f"Please use relative path without '~'. "
                f"检测到家目录扩展符'~': '{relative_path}'。请使用不含'~'的相对路径。"
            )
        
        # 选择基准目录 / Select base directory
        if base == 'workspace':
            base_dir = self._workspace_root
        elif base == 'package':
            base_dir = Path(self._package_share_dir)
        else:
            raise ValueError(
                f"Invalid base directory: '{base}'. Must be 'workspace' or 'package'. "
                f"无效的基准目录: '{base}'。必须为'workspace'或'package'。"
            )
        
        # 解析路径 / Resolve path
        resolved = (base_dir / relative_path).resolve()
        return resolved
    
    def validate_config_paths(
        self, 
        config: Dict, 
        path_keys: Optional[List[str]] = None
    ) -> Dict[str, Union[str, Exception]]:
        """
        验证配置字典中的所有路径 / Validate all paths in config dictionary
        
        Args:
            config: 配置字典 / Configuration dictionary
            path_keys: 需要验证的路径键列表，None表示自动检测 / Path keys to validate, None for auto-detect
        
        Returns:
            Dict[str, Union[str, Exception]]: 验证结果字典 / Validation result dictionary
            键为路径键名，值为错误信息(str)或异常对象 / Key is path key, value is error message or exception
        
        Examples:
            >>> pm = PathManager()
            >>> config = {
            ...     'maps_directory': 'maps',
            ...     'logs_directory': '/tmp/logs',  # 绝对路径，错误 / Absolute path, error
            ...     'calibration_directory': 'calibration'
            ... }
            >>> results = pm.validate_config_paths(config)
            >>> print(results)
            {'logs_directory': ValueError("Absolute path detected: '/tmp/logs'. ...")}
        """
        if path_keys is None:
            # 自动检测路径键（通常包含'directory', 'path', 'file'等关键词）
            # Auto-detect path keys (usually contain 'directory', 'path', 'file' keywords)
            path_keys = [
                k for k in config.keys() 
                if any(keyword in k.lower() for keyword in ['directory', 'path', 'file'])
            ]
        
        validation_results = {}
        
        for key in path_keys:
            if key not in config:
                continue
            
            path_value = config[key]
            
            # 跳过非字符串值 / Skip non-string values
            if not isinstance(path_value, str):
                continue
            
            # 跳过空字符串 / Skip empty strings
            if not path_value.strip():
                continue
            
            try:
                # 尝试解析路径 / Try to resolve path
                self.resolve_path(path_value, base='workspace')
            except ValueError as e:
                # 记录验证失败 / Record validation failure
                validation_results[key] = e
        
        return validation_results
    
    def ensure_directory_exists(self, relative_path: str, base: str = 'workspace') -> Path:
        """
        确保目录存在，不存在则创建 / Ensure directory exists, create if not
        
        Args:
            relative_path: 相对路径 / Relative path
            base: 基准目录 / Base directory
        
        Returns:
            Path: 目录的绝对路径 / Absolute path of directory
        
        Examples:
            >>> pm = PathManager()
            >>> log_dir = pm.ensure_directory_exists('logs/hardware')
            >>> print(log_dir.exists())
            True
        """
        resolved = self.resolve_path(relative_path, base=base)
        resolved.mkdir(parents=True, exist_ok=True)
        return resolved
    
    def get_config_file_path(self, filename: str) -> Path:
        """
        获取配置文件的绝对路径（在包的config目录中） / Get absolute path of config file (in package config dir)
        
        Args:
            filename: 配置文件名 / Config filename
        
        Returns:
            Path: 配置文件绝对路径 / Absolute path of config file
        
        Examples:
            >>> pm = PathManager()
            >>> config_path = pm.get_config_file_path('hardware_config.yaml')
            >>> print(config_path.exists())
            True
        """
        return self.resolve_path(f'config/{filename}', base='package')
    
    def list_files_in_directory(
        self, 
        relative_path: str, 
        pattern: str = '*',
        base: str = 'workspace'
    ) -> List[Path]:
        """
        列出目录中的文件 / List files in directory
        
        Args:
            relative_path: 相对目录路径 / Relative directory path
            pattern: 文件匹配模式（glob） / File matching pattern (glob)
            base: 基准目录 / Base directory
        
        Returns:
            List[Path]: 文件路径列表 / List of file paths
        
        Examples:
            >>> pm = PathManager()
            >>> yaml_files = pm.list_files_in_directory('config', '*.yaml', base='package')
            >>> print([f.name for f in yaml_files])
            ['hardware_config.yaml', 'another_config.yaml']
        """
        directory = self.resolve_path(relative_path, base=base)
        
        if not directory.exists():
            return []
        
        if not directory.is_dir():
            return []
        
        return list(directory.glob(pattern))


def main():
    """
    测试函数 / Test function
    """
    print("PathManager Test / PathManager测试")
    print("=" * 60)
    
    # 创建PathManager实例 / Create PathManager instance
    pm = PathManager('bot_hardware')
    
    # 测试1: 获取工作空间根目录 / Test 1: Get workspace root
    print(f"\n1. Workspace Root / 工作空间根目录:")
    print(f"   {pm.workspace_root}")
    
    # 测试2: 获取包share目录 / Test 2: Get package share directory
    print(f"\n2. Package Share Directory / 包Share目录:")
    print(f"   {pm.package_share_dir}")
    
    # 测试3: 解析相对路径 / Test 3: Resolve relative path
    print(f"\n3. Resolve Relative Path / 解析相对路径:")
    try:
        maps_path = pm.resolve_path('maps/test_map', base='workspace')
        print(f"   'maps/test_map' -> {maps_path}")
    except Exception as e:
        print(f"   Error: {e}")
    
    # 测试4: 检测绝对路径 / Test 4: Detect absolute path
    print(f"\n4. Detect Absolute Path / 检测绝对路径:")
    try:
        pm.resolve_path('/tmp/test', base='workspace')
        print(f"   Should raise ValueError!")
    except ValueError as e:
        print(f"   ✓ Caught: {str(e)[:60]}...")
    
    # 测试5: 验证配置路径 / Test 5: Validate config paths
    print(f"\n5. Validate Config Paths / 验证配置路径:")
    test_config = {
        'maps_directory': 'maps',
        'logs_directory': '/tmp/logs',  # 绝对路径 / Absolute path
        'calibration_directory': '~/calibration',  # 使用~ / Using ~
        'waypoints_directory': 'waypoints'
    }
    results = pm.validate_config_paths(test_config)
    if results:
        print(f"   Found {len(results)} invalid path(s) / 发现{len(results)}个无效路径:")
        for key, error in results.items():
            print(f"   - {key}: {str(error)[:50]}...")
    else:
        print(f"   All paths valid / 所有路径有效")
    
    print("\n" + "=" * 60)
    print("Test completed / 测试完成")


if __name__ == '__main__':
    main()
