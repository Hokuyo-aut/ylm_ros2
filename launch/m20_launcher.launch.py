#!/usr/bin/env python

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory

import os
import yaml
import sys

def generate_launch_description():

    ld = LaunchDescription([
        DeclareLaunchArgument('sensor_ip_', default_value="", description='IP of Lumotive sensor to connect to.'),
        DeclareLaunchArgument('sensor_port_', default_value="10940", description='Port of Lumotive sensor to connect to'),
    ])

    sensor_ip = ""
    sensor_port = 10940

    for arg in sys.argv:
        if arg.startswith("sensor_ip_:="):
            sensor_ip = arg.split(":=")[1]
        if arg.startswith("sensor_port_:="):
            sensor_port = int(arg.split(":=")[1])

    # load yaml file
    config_path = os.path.join(
        get_package_share_directory('ylm_ros2'),
        'config',
        'm20_configs.yaml'
    )

    yaml_configs = yaml.safe_load(open(config_path, 'r'))
    configs = yaml_configs['lumotive_ros_params']
    configs['sensor_ip'] = sensor_ip
    configs['sensor_port'] = sensor_port
    
    drv_node = Node(
        package='ylm_ros2',
        executable='lumotive_driver',
        output='screen',
        parameters = [configs]
        )

    ld.add_action(drv_node)
    return ld