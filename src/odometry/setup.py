from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'odometry'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='piero',
    maintainer_email='piero.vega-gutierrez@univ-tlse3.fr',
    description='Odométrie avec DualMotorsController ou GPS',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'gps_odometry_node = odometry.gps_odometry_node:main',
            'fake_odom = odometry.fake_odom:main',
        ],
    },
)
