import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'nexva_coverage'

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
    description='Boustrophedon coverage planning for the Nexva vacuum robot.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'coverage_planner = nexva_coverage.coverage_planner_node:main',
            'coverage_estimator = nexva_coverage.coverage_estimator_node:main',
            'zone_coverage = nexva_coverage.zone_coverage_node:main',
        ],
    },
)
