#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Mock hardware_interface for testing
用于测试的hardware_interface模拟模块

This module provides mock implementations of ros2_control's hardware_interface
for unit testing purposes without requiring full ROS2 installation.
"""


class return_type:
    """Mock return_type enum"""
    OK = 0
    ERROR = 1


class SystemInterface:
    """Mock SystemInterface base class"""
    
    def __init__(self):
        self.logger = None
    
    def on_init(self, hardware_info):
        """Initialize hardware interface"""
        return return_type.OK
    
    def on_configure(self, previous_state):
        """Configure hardware"""
        return return_type.OK
    
    def on_activate(self, previous_state):
        """Activate hardware"""
        return return_type.OK
    
    def on_deactivate(self, previous_state):
        """Deactivate hardware"""
        return return_type.OK
    
    def on_cleanup(self, previous_state):
        """Cleanup resources"""
        return return_type.OK
    
    def on_shutdown(self, previous_state):
        """Shutdown hardware"""
        return return_type.OK
    
    def read(self, time, duration):
        """Read hardware state"""
        return return_type.OK
    
    def write(self, time, duration):
        """Write control commands"""
        return return_type.OK
