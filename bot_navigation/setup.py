import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'bot_navigation'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # 启动文件 / Launch files
        (os.path.join('share', package_name, 'launch'), 
            glob(os.path.join('launch', '*.py'))),
        # Nav2 配置文件 / Nav2 config files
        (os.path.join('share', package_name, 'config', 'nav2'), 
            glob(os.path.join('config', 'nav2', '*.yaml'))),
        (os.path.join('share', package_name, 'config', 'nav2'), 
            glob(os.path.join('config', 'nav2', '*.xml'))),
        # 定位融合配置文件 / Localization config files
        (os.path.join('share', package_name, 'config', 'localization'), 
            glob(os.path.join('config', 'localization', '*.yaml'))),
        # 探索配置文件 / Exploration config files (预留)
        (os.path.join('share', package_name, 'config', 'exploration'), 
            glob(os.path.join('config', 'exploration', '*.yaml'))),
        # RViz 配置文件 / RViz config files
        (os.path.join('share', package_name, 'config', 'rviz'), 
            glob(os.path.join('config', 'rviz', '*.rviz'))),
        # 地图文件 / Map files
        (os.path.join('share', package_name, 'maps'), 
            glob(os.path.join('maps', '*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hurry',
    maintainer_email='jeladeer@msn.com',
    description='Navigation package for LeKiwi robot / LeKiwi机器人导航包',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'exploration_mapper = bot_navigation.exploration_mapper:main',
            'mission_planner = bot_navigation.mission_planner:main',
        ],
    },
)
