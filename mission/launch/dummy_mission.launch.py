from launch import LaunchDescription
from launch_ros.actions import Node

import os
import json

def get_dummy_mission_params():
    config_json_file_path = '/home/ubuntu/solar_projet_settings.json'
    with open(config_json_file_path, 'r') as config:
        params = json.load(config)

    init_pos_params = params.get("drone_initial_position", {})
    mission_pos = params.get("drone_controller", {})

    dummy_mission_params = init_pos_params
    dummy_mission_params["flight_altitude"] = mission_pos.get("flight_altitude")
    dummy_mission_params["yaw"] = mission_pos.get("yaw")
    dummy_mission_params["use_waypoints_utm"] = mission_pos.get("use_waypoints_utm")

    unreal_environment_params = params.get("unreal_environment", {})    
    dummy_mission_params["waypoint_start_buffer"] = unreal_environment_params.get("waypoint_start_buffer")
    dummy_mission_params["waypoint_end_buffer_x"] = unreal_environment_params.get("waypoint_end_buffer_x")
    dummy_mission_params["waypoint_end_buffer_y"] = unreal_environment_params.get("waypoint_end_buffer_y")
    dummy_mission_params["waypoints"] = json.dumps(params.get("waypoints"))

    if dummy_mission_params["use_waypoints_utm"]:
        dummy_mission_params["waypoints"] = json.dumps(params.get("waypoints_utm"))
    else:
        dummy_mission_params["waypoints"] = json.dumps(params.get("waypoints"))
        
    services = params.get("service_names", {})    
    topics = params.get("topic_names", {})

    services_names = ["takeoff", "landing"]
    topics_names = ["waypoint", "camera_trigger", "local_position"]

    for s in services_names:
        dummy_mission_params[f'{s}_service_name'] = services.get(f'{s}')
    for t in topics_names:
        dummy_mission_params[f'{t}_topic_name'] = topics.get(f'{t}')
        
    return dummy_mission_params

def generate_launch_description():

    dummy_mission_params = get_dummy_mission_params()

    dummy_mission_node = Node(
        package='mission',
        executable='dummy_mission_node',
        parameters=[dummy_mission_params]
    )
    
    

    return LaunchDescription([
        dummy_mission_node
    ])