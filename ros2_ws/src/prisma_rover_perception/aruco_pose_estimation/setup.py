from setuptools import setup
import os
from glob import glob

package_name = 'aruco_pose_estimation'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', [os.path.join('resource', package_name)]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),  # Aggiungi questa linea
    ],
    install_requires=['setuptools'],
    zip_safe=False,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='Aruco pose estimation package',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'aruco_node = aruco_pose_estimation.aruco_node:main',
        ],
    },
)