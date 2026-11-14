"""
WebDriveSim (our simulator) — the counterpart of carla.launch.py.
Starts the simulator's ROS side (third_party/webdrivesim): the web server that sends the page to your
browser (port WEBDRIVESIM_WEB_PORT) and rosbridge, the page <-> ROS link (port WEBDRIVESIM_ROS_PORT).
All settings come from docker/.env.
"""
import os

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare

env = os.environ.get


def generate_launch_description():
    sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([FindPackageShare('webdrivesim_bringup'), '/launch/sim.launch.py']),
        launch_arguments={'world': env('WEBDRIVESIM_WORLD', 'empty_highway.json'),
                          'worlds_dir': env('WEBDRIVESIM_WORLDS', ''),
                          'control': 'ros',
                          'sync': env('WEBDRIVESIM_SYNC', 'false'),
                          'web_port': env('WEBDRIVESIM_WEB_PORT', '8080'),
                          'rosbridge_port': env('WEBDRIVESIM_ROS_PORT', '9090')}.items())
    return LaunchDescription([sim])
