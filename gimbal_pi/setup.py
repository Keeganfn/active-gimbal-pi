from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'gimbal_pi'

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
    maintainer='keegan',
    maintainer_email='keegan.nave@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'tof_pub_all = gimbal_pi.tof_pub_all:main',
            'tof_pub_depth = gimbal_pi.tof_pub_depth:main',
            'gripper_control = gimbal_pi.gripper_control:main',
            'motor_control = gimbal_pi.motor_control:main',
        ],
    },
)
