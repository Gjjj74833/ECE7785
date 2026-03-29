from setuptools import setup
import os
from glob import glob

package_name = 'team1_navigate_to_goal'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='you@gatech.edu',
    description='Lab 4 waypoint navigation with obstacle avoidance',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'get_object_range = team1_navigate_to_goal.get_object_range:main',
            'go_to_goal       = team1_navigate_to_goal.go_to_goal:main',
        ],
    },
)
