from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'bot_perception'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # 安装launch文件
        (os.path.join('share', package_name, 'launch'), 
            glob('launch/*.launch.py')),
        # 安装脚本
        (os.path.join('share', package_name, 'scripts'), 
            glob('scripts/*.sh')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hurry',
    maintainer_email='jeladeer@msn.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'depth_to_laserscan = bot_perception.depth_to_laserscan:main',
        ],
    },
)
