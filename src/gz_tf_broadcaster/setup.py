from setuptools import find_packages, setup

package_name = 'gz_tf_broadcaster'

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
    maintainer_email='j.p.g.dubois@student.tudelft.nl',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'tf_broadcaster = gz_tf_broadcaster.tf_broadcaster:main',
            'tf_to_odom = gz_tf_broadcaster.tf_to_odom:main',
        ],
    },
)
