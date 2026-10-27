from glob import glob
from setuptools import setup, Extension
import shlex
import subprocess

package_name = 'bai06_turtlebot4'
ign_cflags = shlex.split(subprocess.check_output(
    ['pkg-config', '--cflags', 'ignition-transport11', 'ignition-msgs8'], text=True))
ign_ldflags = shlex.split(subprocess.check_output(
    ['pkg-config', '--libs', 'ignition-transport11', 'ignition-msgs8'], text=True))
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
    ext_modules=[Extension(package_name+'._ign_markers', ['bai06_turtlebot4/ign_markers.cpp'],
                           extra_compile_args=ign_cflags, extra_link_args=ign_ldflags,
                           language='c++')],
    install_requires=['setuptools'], zip_safe=False,
    maintainer='Đỗ Việt Anh', maintainer_email='vanhhhhh512@users.noreply.github.com',
    description='Webcam gesture control for TurtleBot 4 in Gazebo',
    license='MIT',
    entry_points={'console_scripts': [
        'gesture_recognition_node = bai06_turtlebot4.gesture_recognition_node:main',
        'robot_behavior_node = bai06_turtlebot4.robot_behavior_node:main',
        'gazebo_visualization_node = bai06_turtlebot4.gazebo_visualization_node:main',
        'simulation_ready_node = bai06_turtlebot4.simulation_ready_node:main',
    ]},
)
