from launch import LaunchDescription
from launch_ros.actions import Node

import os
import json

def get_drone_controller_params():
    config_json_file_path = '/home/ubuntu/solar_projet_settings.json'
    with open(config_json_file_path, 'r') as config:
        params = json.load(config)

    drone_controller_params = {}

    drone_params = params.get("drone_controller", {})
    drone_controller_params["takeoff_altitude"] = drone_params.get("takeoff_altitude")

    services = params.get("service_names", {})
    topics = params.get("topic_names", {})

    services_names = ["takeoff", "landing"]
    topics_names = ["waypoint", "gimbal"]

    for s in services_names:
        drone_controller_params[f'{s}_service_name'] = services.get(f'{s}')
    for t in topics_names:
        drone_controller_params[f'{t}_topic_name'] = topics.get(f'{t}')
    return drone_controller_params

def generate_launch_description():

    drone_controller_params = get_drone_controller_params()

    drone_controller_node = Node(
        package='drone_controller',
        executable='drone_controller_node',
        parameters=[drone_controller_params]
    )
    
    

    return LaunchDescription([
        drone_controller_node
    ])