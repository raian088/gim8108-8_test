from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'gim8108_driver'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools', 'python-can'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@example.com',
    description='ROS2 driver for GIM8108-8 motor',
    license='MIT',
    entry_points={
        'console_scripts': [
            'gim8108_motor_node = gim8108_driver.motor_node:main',
            'gim8108_motor_node_usb = gim8108_driver.motor_node_usb:main',
        ],
    },
)
