import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'fortest'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
	(os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*')))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='adan',
    maintainer_email='adan@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        "motor = fortest.turtle_motor:main",
        "teleop = fortest.turtle_keyboard:main",
        "odom_dect = fortest.odom_dect:main",
        "motor_back = fortest.turtle_motor_back:main",
        "imu = fortest.imu_pub:main"
        ],
    },
)

