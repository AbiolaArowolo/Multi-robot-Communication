from setuptools import setup
from glob import glob
import os

package_name = 'multi_robot_comm'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name] if os.path.isdir(package_name) else [],
    data_files=[
         ('share/' + package_name + '/launch', glob('launch/*.py')),  # <-- add this
    ('share/' + package_name + '/urdf', glob('urdf/*.xacro') + glob('urdf/*.urdf')),
    ('share/' + package_name + '/worlds', glob('worlds/*.world')),
    ('share/' + package_name, ['package.xml']),

  ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='abiola',
    maintainer_email='your_email@example.com',
    description='Multi-robot communication and Gazebo simulation',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # If you add Python ROS2 nodes, e.g.:
            # 'robot_spawner = multi_robot_comm.robot_spawner:main',
        ],
    },
)

