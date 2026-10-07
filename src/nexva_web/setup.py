import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'nexva_web'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'web'), glob('web/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='PRAMOTH-S',
    maintainer_email='pramothsekar@gmail.com',
    description='Web waypoint control for the Nexva robot.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'waypoint_cli = nexva_web.cli:main',
            'web_bridge = nexva_web.web_bridge:main',
            # Standalone, for when the page is the thing that is broken:
            #   ros2 run nexva_web pi_health
            'pi_health = nexva_web.pi_health:main',
        ],
    },
)
