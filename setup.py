from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'orca_fsm'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Launch files
        (os.path.join('share', package_name, 'launch'), 
            glob('launch/*.launch.py')),
        # Config files
        (os.path.join('share', package_name, 'config'), 
            glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='ycchiu226',
    maintainer_email='yc.ee12@nycu.edu.tw',
    description='Finite state machine for SAUVC 2026',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'fsm = orca_fsm.fsm:main',
            'fsm25 = orca_fsm.fsm25:main',
            'bridge = orca_fsm.bridge:main',
            'teleop = orca_fsm.teleop:main',
        ],
    },
)
