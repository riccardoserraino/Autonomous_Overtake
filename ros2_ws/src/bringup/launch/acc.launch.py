"""Module 1: adaptive cruise control + lane keeping (parameters: config/params.yaml)."""
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    params = [[FindPackageShare('bringup'), '/config/params.yaml'], {'use_sim_time': True}]
    return LaunchDescription([Node(package='acc_lane_keeping', executable='acc_node', parameters=params, output='screen')])
