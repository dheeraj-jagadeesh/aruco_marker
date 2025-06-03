from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'aruco_maintenance_board'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include config files
        (os.path.join('share', package_name, 'config'), glob('config/*.json')),
        # Include launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your_email@example.com',
    description='ArUco maintenance board detection package',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'aruco_detector = aruco_maintenance_board.aruco_detector:main',
            'aruco_standalone = aruco_maintenance_board.aruco_standalone:main',
        ],
    },
)