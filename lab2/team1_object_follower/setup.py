from setuptools import setup
import os
from glob import glob

package_name = 'team1_object_follower'

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
    description='Lab 2 object follower for TurtleBot3',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'find_object  = team1_object_follower.find_object:main',
            'rotate_robot = team1_object_follower.rotate_robot:main',
        ],
    },
)
