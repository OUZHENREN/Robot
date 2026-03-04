# 文件路径: /home/yff/elite_ros_ws/src/tcp_bridge/setup.py

from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'tcp_bridge'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # 关键：安装 launch 目录下的所有 .py 文件
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools', 'scipy'],  # 明确需要scipy
    zip_safe=True,
    maintainer='yff',
    maintainer_email='yff@todo.todo',
    description='ROS 2 nodes for Elite Robot communication bridge.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # 主通信节点
            'tcp_server = tcp_bridge.tcp_server:main',
            'robot_commander = tcp_bridge.robot_commander:main',

            # GLB模型显示节点
            'static_model_publisher = tcp_bridge.static_model_publisher:main',

            # 辅助与测试节点
            'pose_to_tf_broadcaster = tcp_bridge.pose_to_tf_broadcaster:main',
            'pose_sender_node = tcp_bridge.pose_sender_node:main',
        ],
    },
)
