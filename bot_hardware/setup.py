from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'bot_hardware'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # 安装配置文件 / Install config files
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml') if os.path.exists('config') else []),
        # 安装launch文件 / Install launch files
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.py') if os.path.exists('launch') else []),
        # 安装URDF文件 / Install URDF files
        (os.path.join('share', package_name, 'urdf'),
            glob('urdf/*.xacro') if os.path.exists('urdf') else []),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hurry',
    maintainer_email='jeladeer@msn.com',
    description='Hardware interface package for LeKiwi omnidirectional robot / LeKiwi全向机器人硬件接口包',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            # 硬件控制节点 / Hardware control node
            'omni_hardware_node = bot_hardware.omni_hardware_node:main',
            
            # 工具命令 / Utility commands
            # 'test_imu_coordinate = bot_hardware.tools.test_imu_coordinate:main',
            # 'check_timestamp_sync = bot_hardware.tools.check_timestamp_sync:main',
        ],
    },
)
