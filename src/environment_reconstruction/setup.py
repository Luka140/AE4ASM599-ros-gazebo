from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'environment_reconstruction'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ros',
    maintainer_email='luka-groot@hotmail.nl',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'stereo_reconstruction = environment_reconstruction.stereo_reconstruction:main',
            'total_reconstruction = environment_reconstruction.pointcloud_aggregation:main',
            'positioning = environment_reconstruction.positioning:main',
            'keyboard_capture = environment_reconstruction.keyboard_capture:main'
        ],
    },
)
