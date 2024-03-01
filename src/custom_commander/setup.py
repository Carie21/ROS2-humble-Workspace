from glob import glob
import os

from setuptools import setup


package_name = 'custom_commander'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        # ('share/ament_index/resource_index/packages',
        #     ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='steve',
    maintainer_email='stevenmacenski@gmail.com',
    description='An importable library for writing mobile robot applications in python3',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
                # 'example_nav_to_pose = custom_commander.example_nav_to_pose:main',
                # 'example_nav_through_poses = custom_commander.example_nav_through_poses:main',
                # 'example_waypoint_follower = custom_commander.example_waypoint_follower:main',
                # 'example_follow_path = custom_commander.example_follow_path:main',
                # 'demo_picking = custom_commander.demo_picking:main',
                'demo_inspection = custom_commander.demo_inspection:main',
                'demo_security = custom_commander.demo_security:main',
                # 'demo_recoveries = custom_commander.demo_recoveries:main',
                # 'example_assisted_teleop = custom_commander.example_assisted_teleop:main',
        ],
    },
)
