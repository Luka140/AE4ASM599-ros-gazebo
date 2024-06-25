from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'navigate'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='justin dubois',
    maintainer_email='j.p.g.dubois@student.tudelft.nl',
    description='custom navigation nodes',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'nav_pose_server = navigate.nav_pose_server:main',
            'nav_pose_client = navigate.nav_pose_client:main',
            'nav_controller = navigate.nav_controller:main',
            'nav_pose_avoidance_server = navigate.nav_pose_avoidance_server:main'
        ],
    },
)
