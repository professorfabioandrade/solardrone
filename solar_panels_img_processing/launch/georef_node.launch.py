from launch import LaunchDescription
from launch_ros.actions import Node

import os
import json

def get_georef_params():
    config_json_file_path = '/home/ubuntu/solar_projet_settings.json'
    with open(config_json_file_path, 'r') as config:
        params = json.load(config)

    georef_params = {}

    unreal_environment_params = params.get("unreal_environment", {})
    camera_params = params.get("camera", {})

    georef_params["solar_panel_height"] = unreal_environment_params.get("solar_panel_height")
    georef_params["panel_tilt"] = unreal_environment_params.get("panel_tilt")
    georef_params["K_matrix"] = json.dumps(camera_params.get("K_matrix"))
 
    topics = params.get("topic_names", {})
    topics_names = ["local_position", "compass_hdg", "georef_line", "canny_line", "gimbal"]

    for t in topics_names:
        georef_params[f'{t}_topic_name'] = topics.get(f'{t}')
    return georef_params

def generate_launch_description():

    georef_params = get_georef_params()

    georef_node = Node(
        package='solar_panels_img_processing',
        executable='georef_node',
        parameters=[georef_params]
    )

    return LaunchDescription([
        georef_node
    ])