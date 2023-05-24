from setuptools import setup
import os
from glob import glob

package_name = 'tello'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.py'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='mimi',
    maintainer_email='101798755+MikkelineHavgaard@users.noreply.github.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': ["connector=tello.connector:main",
        "image_processor=tello.image_processor:main",
        "movement=tello.movement:main",
        ],
    },
)
import launch
import launch_ros.actions

def generate_launch_description():
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='tello',
            executable='connector',
            name='connector'),
  ])