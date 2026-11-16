from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'quadruped_control'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml', 'README.md']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    package_data={package_name: ['imu.html']},
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='csablev2',
    maintainer_email='csablev2@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'command_emit = quadruped_control.command_emit:main',
            'serial_bridge = quadruped_control.serial_bridge:main',
            'reception_gyroscope = quadruped_control.reception_gyroscope:main',
            'reception_lidar = quadruped_control.reception_lidar:main',
        ],
    },
)
