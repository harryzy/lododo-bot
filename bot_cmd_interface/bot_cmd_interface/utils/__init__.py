"""
bot_cmd_interface工具模块 / Utility module

包含配置加载、日志工具等辅助功能 /
Contains utility functions for configuration loading, logging, etc.
"""

from .config_loader import load_command_config

__all__ = [
    'load_command_config',
]
