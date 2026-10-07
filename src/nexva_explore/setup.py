import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'nexva_explore'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='PRAMOTH-S',
    maintainer_email='pramothsekar@gmail.com',
    description='Frontier exploration and coverage cleaning for the Nexva robot, '
                'ported from vac_main1',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'frontier_explorer = nexva_explore.frontier_explorer:main',
            'auto_clean = nexva_explore.frontier_explorer:main',
            'map_autosaver = nexva_explore.map_autosaver:main',
            'map_saver = nexva_explore.map_saver:main',
            'pose_store = nexva_explore.pose_store:main',
            'initial_pose_seeder = nexva_explore.initial_pose_seeder:main',
            'map_updater = nexva_explore.map_updater:main',
            'mode_manager = nexva_explore.mode_manager:main',
        ],
    },
)
