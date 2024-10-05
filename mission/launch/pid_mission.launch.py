from launch import LaunchDescription
from launch_ros.actions import Node

import os
import json

def get_pid_params():
    config_json_file_path = '/home/ubuntu/solar_projet_settings.json'
    with open(config_json_file_path, 'r') as config:
        params = json.load(config)

    init_pos_params = params.get("drone_initial_position", {})
    mission_pos = params.get("drone_controller", {})

    pid_params = init_pos_params
    pid_params["flight_altitude"] = mission_pos.get("flight_altitude")
    pid_params["yaw"] = mission_pos.get("yaw")
    pid_params["use_waypoints_utm"] = mission_pos.get("use_waypoints_utm")

    unreal_environment_params = params.get("unreal_environment", {})    
    pid_params["panel_tilt"] = unreal_environment_params.get("panel_tilt")
    pid_params["waypoint_start_buffer"] = unreal_environment_params.get("waypoint_start_buffer")
    pid_params["waypoint_end_buffer_x"] = unreal_environment_params.get("waypoint_end_buffer_x")
    pid_params["waypoint_end_buffer_y"] = unreal_environment_params.get("waypoint_end_buffer_y")
    pid_params["waypoints"] = json.dumps(params.get("waypoints"))

    if pid_params["use_waypoints_utm"]:
        pid_params["waypoints"] = json.dumps(params.get("waypoints_utm"))
    else:
        pid_params["waypoints"] = json.dumps(params.get("waypoints"))
        
    services = params.get("service_names", {})    
    topics = params.get("topic_names", {})

    services_names = ["takeoff", "landing"]
    topics_names = ["waypoint", "camera_trigger", "local_position", "georef_line", "compass_hdg", "gimbal"]

    for s in services_names:
        pid_params[f'{s}_service_name'] = services.get(f'{s}')
    for t in topics_names:
        pid_params[f'{t}_topic_name'] = topics.get(f'{t}')
        
    return pid_params

def generate_launch_description():

    pid_params = get_pid_params()

    pid_node = Node(
        package='mission',
        executable='pid_node',
        parameters=[pid_params]
    )
    
    return LaunchDescription([
        pid_node
    ])