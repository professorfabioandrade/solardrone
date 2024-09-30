from launch import LaunchDescription
from launch_ros.actions import Node

import os
import json

def get_canny_params():
    config_json_file_path = '/home/ubuntu/solar_projet_settings.json'
    with open(config_json_file_path, 'r') as config:
        params = json.load(config)

    canny_params = {}
    
    camera_params = params.get("camera", {})
    canny_params["vertical_lines"] = camera_params.get("vertical_lines")

    topics = params.get("topic_names", {})
    topics_names = ["airsim_imgs", "camera_trigger", "canny_line"]

    for t in topics_names:
        canny_params[f'{t}_topic_name'] = topics.get(f'{t}')
    return canny_params

def generate_launch_description():

    canny_params = get_canny_params()

    canny_node = Node(
        package='solar_panels_img_processing',
        executable='canny_node',
        parameters=[canny_params]
    )
    
    

    return LaunchDescription([
        canny_node
    ])