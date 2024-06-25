from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'image2map'

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
            f'stereo_reconstruction = {package_name}.stereo_reconstruction:main',
            f'total_reconstruction = {package_name}.pointcloud_aggregation:main',
            f'positioning = {package_name}.positioning:main',
            f'filter = {package_name}.filter_pcl:main',
            f'utils = {package_name}.utils:main',
            f'coordinator = {package_name}.timer:main',
            f'mapper = {package_name}.grid_map:main'
        ],
    },
)
