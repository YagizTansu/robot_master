from setuptools import setup

package_name = 'robot_teleop'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='tech',
    maintainer_email='qaz9517532846@gmail.com',
    description='Teleoperation node using keyboard for robot.',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'robot_teleop_key = robot_teleop.robot_teleop_key:main'
        ],
    },
)
