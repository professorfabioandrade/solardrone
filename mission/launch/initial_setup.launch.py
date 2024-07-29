from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource, PythonLaunchDescriptionSource

import os

def generate_launch_description():
    
    mavros_launch_file = os.path.join(
        get_package_share_directory('mavros'), 'launch', 'apm.launch')

    mavros_launch = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(mavros_launch_file),
        launch_arguments={'fcu_url': 'udp://:14550@'}.items(),
    )

    airsim_launch_file = os.path.join(
        get_package_share_directory('airsim_ros_pkgs'), 'launch', 'airsim_node.launch.py')

    airsim_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(airsim_launch_file),
        launch_arguments={
            'output': 'screen',
            'host': '172.18.128.1',
        }.items(),
    )

    return LaunchDescription([
        mavros_launch,
        airsim_launch
    ])