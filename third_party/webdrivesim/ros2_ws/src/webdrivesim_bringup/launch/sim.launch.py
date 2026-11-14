"""
Start the WebDriveSim stack:
  1. rosbridge_websocket  (ws://localhost:9090)  - the browser <-> ROS 2 gateway
  2. webdrivesim_web_server      (http://localhost:8080) - serves the simulator page + the world files

Usage:
  ros2 launch webdrivesim_bringup sim.launch.py                                   # empty highway, driven by ROS
  ros2 launch webdrivesim_bringup sim.launch.py world:=rural_overtake.json control:=auto
  ros2 launch webdrivesim_bringup sim.launch.py world:=my_world.json sync:=true open_browser:=true
Then open http://localhost:8080 once: the page loads the world and follows any later change.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    args = [
        DeclareLaunchArgument('world', default_value='empty_highway.json', description='file in the worlds folder'),
        DeclareLaunchArgument('worlds_dir', default_value='', description='folder with the world .json files (default: $WEBDRIVESIM_WORLDS or /ros2_ws/worlds)'),
        DeclareLaunchArgument('control', default_value='ros', description='ros | auto | manual'),
        DeclareLaunchArgument('sync', default_value='false', description='CARLA-like synchronous mode'),
        DeclareLaunchArgument('web_port', default_value='8080', description='HTTP port of the simulator page'),
        DeclareLaunchArgument('rosbridge_port', default_value='9090', description='rosbridge websocket port'),
        DeclareLaunchArgument('open_browser', default_value='false', description='open the page automatically (host install only)'),
    ]
    lc = LaunchConfiguration
    # scoped group: the 'port' argument given to rosbridge must not leak into this launch file
    rosbridge = GroupAction(scoped=True, actions=[IncludeLaunchDescription(
        AnyLaunchDescriptionSource(os.path.join(get_package_share_directory('rosbridge_server'), 'launch', 'rosbridge_websocket_launch.xml')),
        launch_arguments={'port': lc('rosbridge_port')}.items())])
    web = Node(package='webdrivesim_bringup', executable='web_server', name='webdrivesim_web_server', output='screen',
               parameters=[{'world': lc('world'), 'worlds_dir': lc('worlds_dir'), 'control': lc('control'),
                            'sync': ParameterValue(lc('sync'), value_type=bool),
                            'port': ParameterValue(lc('web_port'), value_type=int),
                            'rosbridge_port': ParameterValue(lc('rosbridge_port'), value_type=int),
                            'open_browser': ParameterValue(lc('open_browser'), value_type=bool)}])
    return LaunchDescription(args + [rosbridge, web])
