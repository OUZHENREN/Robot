# 文件路径: ~/elite_ros_ws/src/model_visualizer/setup.py

from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'model_visualizer'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # 添加下面这行，以安装launch目录下的所有.py文件
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='yff', # 你的名字
    maintainer_email='yff@todo.todo', # 你的邮箱
    description='A package to display a static 3D model in RViz at a specified pose.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # 定义可执行节点名，链接到我们的Python文件和main函数
            'static_model_publisher = model_visualizer.static_model_publisher:main',
        ],
    },
)

