from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'vision_speed_guard'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        # Install package resource index
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        # Install package manifest
        ('share/' + package_name, ['package.xml']),
        # Install launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='william.engel@mdynamix.de',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'speed_guard = vision_speed_guard.speed_guard_node:main'
        ],
    },
)
