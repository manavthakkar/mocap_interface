from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'mocap_interface'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.py'))
    ],
    install_requires=['setuptools','rclpy','mocap4r2_msgs'],
    zip_safe=True,
    maintainer='manav',
    maintainer_email='manav.thakkar@tuhh.de',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        'qualisys_pubsub = mocap_interface.qualisys_pubsub:main',
        'pose_to_odom = mocap_interface.pose_to_odom:main'
        ],
    },
)
