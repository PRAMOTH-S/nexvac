from glob import glob

from setuptools import find_packages, setup

package_name = 'nexva_sensor'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
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
            "cam = nexva_sensor.cam:main",
            "bno055_imu = nexva_sensor.bno055_imu:main",
            "imu_monitor = nexva_sensor.imu_monitor:main",
            "motion_check = nexva_sensor.motion_check:main",
        ],
    },
)
