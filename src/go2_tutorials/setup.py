from glob import glob
from setuptools import setup

package_name = 'go2_tutorials'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'imu_yaw = go2_tutorials.imu_yaw:main',
            'yaw_check = go2_tutorials.yaw_check:main',
        ],
    },
)
