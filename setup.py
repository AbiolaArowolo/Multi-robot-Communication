from setuptools import setup
import os
from glob import glob

package_name = 'task3_launcher'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # CRITICAL: This line installs your launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='abiola',
    maintainer_email='abayo83@gmail.com',
    description='Task 3 Multi-Robot Communication',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            # CRITICAL: This creates the "commands" for your scripts
            'husky_talker = task3_launcher.husky_talker:main',
            'turtle_talker = task3_launcher.turtle_talker:main',
        ],
    },
)
