from glob import glob
from setuptools import setup

package_name = 'robot_vda5050_adapter'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*launch.[pxy][yma]*')),
        ('share/' + package_name + '/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='yagiz',
    maintainer_email='yagiz@todo.com',
    description='VDA5050 v2.0.0 compliant adapter for mecanum/omni-drive robot',
    license='BSD-3-Clause',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'robot_adapter = robot_vda5050_adapter.robot_adapter:main'
        ],
    },
)
