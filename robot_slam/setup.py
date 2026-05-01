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
    ],
    entry_points={
        'console_scripts': [],
    },
)
