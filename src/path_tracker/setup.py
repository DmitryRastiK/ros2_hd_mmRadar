from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'path_tracker'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='dmitrytorov',
    maintainer_email='torovdmitrii@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'tracker_path_node = path_tracker.tracker_path_node:main',
            'kalman_filter_node = path_tracker.kalman_filter_node:main',
            'corridor_node = path_tracker.corridor_node:main',
            'DBSCAN_node = path_tracker.DBSCAN_node:main',
        ],
    },
)
