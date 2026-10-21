from glob import glob
from setuptools import setup

package_name = 'bai06_turtlebot4'
setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*')),
        ('share/' + package_name + '/worlds', glob('worlds/*')),
    ],
    install_requires=['setuptools'], zip_safe=True,
    maintainer='Đỗ Việt Anh', maintainer_email='vanhhhhh512@users.noreply.github.com',
    description='Webcam gesture control for TurtleBot 4 in Gazebo',
    license='MIT',
    entry_points={'console_scripts': [
        'gesture_recognition_node = bai06_turtlebot4.gesture_recognition_node:main',
        'robot_behavior_node = bai06_turtlebot4.robot_behavior_node:main',
    ]},
)
