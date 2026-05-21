from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'gim8108_gui'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@example.com',
    description='PyQt5 GUI for GIM8108-8 motor control',
    license='MIT',
    entry_points={
        'console_scripts': [
            'gim8108_control_gui  = gim8108_gui.control_gui:main',
            'gim8108_motion_gui   = gim8108_gui.motion_gui:main',
        ],
    },
)
