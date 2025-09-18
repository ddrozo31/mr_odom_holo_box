from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'mr_odom_holo_box'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.py')),
        (os.path.join('share', package_name,'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='davidrozoosorio',
    maintainer_email='david.rozo31@eia.edu.co',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'mr_odom_node_p1=mr_odom_holo_box.mr_odom_node_p1:main',
            'mr_odom_node_p1_teleop=mr_odom_holo_box.mr_odom_node_p1_teleop:main',
            'joy_bridge_node=mr_odom_holo_box.joy_bridge_node:main',
        ],
    },
)
