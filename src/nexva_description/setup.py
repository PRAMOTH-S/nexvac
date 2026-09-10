from setuptools import find_packages, setup

import os
from glob import glob
package_name = 'nexva_description'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
                ('share/' + package_name, ['package.xml']),
                        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py*')),
        # Install the worlds files
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*')),
        # Install the RViz configuration files
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*')),
        # Install the URDF files
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*')),
        # Install the meshes
        (os.path.join('share', package_name, 'meshes'), glob('meshes/*')),
        # Install the config files
        (os.path.join('share', package_name, 'config'), glob('config/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='pmod',
    maintainer_email='pmod@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
        ],
    },
)
