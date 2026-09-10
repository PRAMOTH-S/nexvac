from setuptools import setup

package_name = 'esp32_rviz_control'

setup(
    name=package_name,
    version='0.0.1',

    packages=[package_name],

    install_requires=[
        'setuptools',
    ],

    zip_safe=True,

    entry_points={
        'console_scripts': [
            'esp32_bridge = esp32_rviz_control.esp32_bridge:main',
        ],
    },
)
