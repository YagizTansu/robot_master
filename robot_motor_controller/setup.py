from setuptools import setup

package_name = 'robot_motor_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'pyserial'],
    zip_safe=True,
    maintainer='yagiz',
    maintainer_email='yagiz.tansu99@gmail.com',
    description='Kinco motor driver bridge — serial CANopen over RS-232 for tricycle drive',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'kinco_bridge     = robot_motor_controller.kinco_bridge:main',
            'sim_kinco_bridge = robot_motor_controller.sim_kinco_bridge:main',
            'kinco_drive_node = robot_motor_controller.kinco_drive_node:main',
        ],
    },
)
