"""Isolated TurtleBot 4 gesture arena, webcam and behavior controller."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription, SetEnvironmentVariable, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetParametersFromFile
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = get_package_share_directory('bai06_turtlebot4')
    sim = get_package_share_directory('turtlebot4_ignition_bringup')
    world = os.path.join(share, 'worlds', 'gesture_arena.sdf')
    model = LaunchConfiguration('model_path')
    output = LaunchConfiguration('output_dir')
    config = os.path.join(share, 'config', 'gestures.yaml')
    nodes = [
        Node(package='bai06_turtlebot4', executable='gazebo_visualization_node',
             output='screen', parameters=[config],
             condition=IfCondition(LaunchConfiguration('show_gazebo_visuals'))),
        Node(package='bai06_turtlebot4', executable='robot_behavior_node',
             output='screen', parameters=[config, {'output_dir': output,
                                                  'require_sim_ready': True}]),
        Node(package='bai06_turtlebot4', executable='simulation_ready_node', output='screen'),
        Node(package='bai06_turtlebot4', executable='gesture_recognition_node',
             output='screen', parameters=[config, {
                 'model_path': ParameterValue(model, value_type=str),
                 'device': ParameterValue(LaunchConfiguration('device'), value_type=int),
                 'show_window': ParameterValue(LaunchConfiguration('show_webcam'), value_type=bool),
                 'output_dir': output}],
             condition=IfCondition(LaunchConfiguration('recognition'))),
    ]
    return LaunchDescription([
        DeclareLaunchArgument('output_dir', default_value='/tmp/bai06_run'),
        DeclareLaunchArgument('model_path', default_value=''),
        DeclareLaunchArgument('device', default_value='0'),
        DeclareLaunchArgument('show_webcam', default_value='true'),
        DeclareLaunchArgument('recognition', default_value='true'),
        DeclareLaunchArgument('show_gazebo_gui', default_value='true'),
        DeclareLaunchArgument('show_gazebo_visuals', default_value='true'),
        SetParametersFromFile(os.path.join(share, 'config', 'simulation.yaml')),
        SetEnvironmentVariable('IGN_GAZEBO_RESOURCE_PATH', ':'.join([
            os.path.join(sim, 'worlds'), '/opt/ros/humble/share',
            os.environ.get('IGN_GAZEBO_RESOURCE_PATH', '')])),
        SetEnvironmentVariable('IGN_GUI_PLUGIN_PATH', ':'.join([
            os.path.join(get_package_share_directory('turtlebot4_ignition_gui_plugins'), 'lib'),
            os.path.join(get_package_share_directory('irobot_create_ignition_plugins'), 'lib')])),
        SetEnvironmentVariable('IGN_GAZEBO_SYSTEM_PLUGIN_PATH', ':'.join([
            '/opt/ros/humble/lib', os.environ.get('LD_LIBRARY_PATH', ''),
            os.environ.get('IGN_GAZEBO_SYSTEM_PLUGIN_PATH', '')])),
        ExecuteProcess(cmd=['ign', 'gazebo', '-s', world, '-r', '-v', '2'], output='screen'),
        TimerAction(period=5.0, actions=[ExecuteProcess(
            cmd=['ign', 'gazebo', '-g', '-v', '2'], output='screen',
            condition=IfCondition(LaunchConfiguration('show_gazebo_gui')))]),
        Node(package='ros_gz_bridge', executable='parameter_bridge',
             name='bai06_clock_bridge',
             arguments=['/clock@rosgraph_msgs/msg/Clock[ignition.msgs.Clock']),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(sim, 'launch', 'turtlebot4_spawn.launch.py')),
            launch_arguments={'model': 'standard', 'world': 'gesture_arena',
                              'x': '0.0', 'y': '0.0',
                              'yaw': '0.0', 'rviz': 'false'}.items()),
        TimerAction(period=5.0, actions=nodes),
    ])
