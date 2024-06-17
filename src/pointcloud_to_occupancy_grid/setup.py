from setuptools import find_packages, setup

package_name = 'pointcloud_to_occupancy_grid'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ros',
    maintainer_email='luka-groot@hotmail.nl',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': ['pointcloud_to_grid_node = pointcloud_to_occupancy_grid.pointcloud_to_grid_node:main',
        ],
    },
)
