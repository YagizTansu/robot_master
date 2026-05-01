from glob import glob
import os
from setuptools import setup

package_name = 'robot_slam'

setup(
    name=package_name,
    version='0.0.0',
    packages=[],
    py_modules=[],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='yagiz',
    maintainer_email='yagiz.tansu99@gmail.com',
    description='SLAM mapping and localization package using slam_toolbox',
    license='TODO: License declaration',
    tests_require=['pytest'],
    data_files=[
        ('share/robot_slam', ['package.xml']),
        (os.path.join('share', 'robot_slam', 'launch'),
            glob(os.path.join('launch', '*.py'))),
        (os.path.join('share', 'robot_slam', 'config'),
            glob(os.path.join('config', '*.yaml'))),
        (os.path.join('share', 'robot_slam', 'maps'),
            glob(os.path.join('maps', '*'))),
        (os.path.join('share', 'robot_slam', 'rviz'),
            glob(os.path.join('rviz', '*.rviz'))),
    ],
    entry_points={
        'console_scripts': [],
    },
)
