from setuptools import setup

package_name = 'cs625_state_monitor'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/state_monitor.launch.py']),
        ('share/' + package_name + '/config', [
            'config/state_monitor.yaml',
            'config/csv_logger.yaml',
        ]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='yff',
    maintainer_email='yff@example.com',
    description='CS625 primary port state monitor and CSV logger',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'raw_state_receiver_node = cs625_state_monitor.raw_state_receiver_node:main',
            'state_csv_logger_node = cs625_state_monitor.state_csv_logger_node:main',
            'sim_state_publisher = cs625_state_monitor.sim_state_publisher:main',
        ],
    },
)
