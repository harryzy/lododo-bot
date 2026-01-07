from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'bot_cmd_interface'

setup(
    name=package_name,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # 安装配置文件 / Install configuration files
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        # 安装launch文件 / Install launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=[
        'setuptools',
        'jsonschema>=4.0.0',  # JSON验证 / JSON validation
        'pyyaml>=5.0.0',      # YAML配置解析 / YAML config parsing
    ],
    zip_safe=True,
    maintainer='LeKiwi Robot Team',
    maintainer_email='lekiwi@robot.com',
    description='LeKiwi Robot Unified Command Interface - 统一命令接口',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'command_adapter = bot_cmd_interface.command_adapter_node:main',
        ],
    },
)
