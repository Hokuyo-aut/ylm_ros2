#!/usr/bin/env python

from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node

from ament_index_python.packages import get_package_share_directory

import os
import sys
import math
import copy
import shutil
import subprocess
from io import open

if sys.version_info[0] < 3:
    from pathlib2 import Path
else:
    from pathlib import Path

try:
    from lxml import etree
except ModuleNotFoundError:
    print('[Lumotive FoV configs generator] Call failed. Please follow the instructions in the M20 user manual on how to add the lxml dependency using rosdep or run view_M20_all.sh.')
    exit()
try:
    import ruamel.yaml
except ModuleNotFoundError:
    print('[Lumotive FoV configs generator] Call failed. Please follow the instructions in the M20 user manual on how to add the ruamel.yaml dependency using rosdep or run view_M20_all.sh.')
    exit()

from ruamel.yaml.comments import CommentedMap as ordereddict  # to avoid '!!omap' in yaml


class URDFFileCreator:
    """
        Class to generate XML URDF file

        config_dict must a dict in which each key is a sensor port (a, b, c or d) and each value is a list of the desired FoVs (0 to 7)
    """
    def __init__(self, config_dict, positions, visual_positions, fov_split_distance=0.0, robot_name="4_m20s", pkg_path=None):
        self.configs = config_dict
        self.positions = positions
        self.visual_origins = visual_positions
        self.fov_split_distance = fov_split_distance

        # XML structures base
        self.robot_root = etree.Element("robot")
        self.robot_root.set("name", robot_name)

    def generate_XML(self):
        self.generate_links()
        self.generate_joints()

    def generate_links(self):
        colors_name = {"a": "blue", "b": "red", "c": "green", "d": "yellow"}
        colors = {"a": "0 0 1 1", "b": "1 0 0 1", "c": "0 1 0 1", "d": "1 1 0 1"}
        link = etree.Element("link")
        link.set("name", "assembly_base")
        self.robot_root.append(link)

        for sensor_head in self.configs:
            for fov_num in self.configs[sensor_head]:
                link = etree.Element("link")
                link.set("name", 'm20_'+sensor_head+'_'+str(fov_num))
                visual = etree.Element("visual")

                origin = etree.Element("origin")
                xyz = self.visual_origins[sensor_head+"_vis"][0:3]
                xyz_str = ' '.join(map(str, xyz))
                rpy = self.visual_origins[sensor_head+"_vis"][3:6]
                rpy_str = ' '.join(map(str, rpy))
                origin.set("xyz", xyz_str)
                origin.set("rpy", rpy_str)

                geometry = etree.Element("geometry")
                mesh = etree.Element("mesh")
                mesh.set("filename", "package://lumotive_ros2/meshes/m20_lidar.STL")
                geometry.append(mesh)

                material = etree.Element("material")
                material.set("name", colors_name[sensor_head])
                color = etree.Element("color")
                color.set("rgba", colors[sensor_head])
                material.append(color)

                visual.append(origin)
                visual.append(geometry)
                visual.append(material)

                link.append(visual)

                self.robot_root.append(link)

    def generate_joints(self):
        current_yaw = 0.0
        for sensor_head in self.configs:
            for fov_num in self.configs[sensor_head]:
                joint = etree.Element("joint")
                joint.set("name", "m20_"+sensor_head+"_"+str(fov_num)+"_joint")
                joint.set("type", "fixed")
                parent = etree.Element("parent")
                parent.set("link", "assembly_base")
                child = etree.Element("child")
                child.set("link", 'm20_'+sensor_head+'_'+str(fov_num))
                origin = etree.Element("origin")

                xyz = self.positions[sensor_head+"_pos"][0:3]
                xyz[2] += fov_num*self.fov_split_distance
                xyz_str = ' '.join(map(str, xyz))
                rpy = self.positions[sensor_head+"_pos"][3:6]
                rpy_str = ' '.join(map(str, rpy))
                origin.set("xyz", xyz_str)
                origin.set("rpy", rpy_str)
                joint.append(parent)
                joint.append(child)
                joint.append(origin)
                self.robot_root.append(joint)

            current_yaw += math.pi / 2

    def write_to_file(self, filename):
        et = etree.ElementTree(self.robot_root)
        et.write(filename, pretty_print=True)


class ConfigFilesCreator:
    """
        Class to generate the config files

        config_dict must a dict in which each key is a sensor port (a, b, c or d) and each value is a list of the desired FoVs (0 to 7)
    """
    def __init__(self, config_dict, pkg_path=None):
        self.yaml = ruamel.yaml.YAML()
        self.yaml.preserve_quotes = True
        self.configs = config_dict
        self.pkg_path = pkg_path
        self.configs_base_content = self.yaml.load(open(os.path.join(self.pkg_path, 'config', 'm20_configs.yaml'), 'r')) # Use base file

    def dump_config_files(self, directory):

        if not os.path.exists(directory):
            os.makedirs(directory)
        else:
            shutil.rmtree(directory)
            os.makedirs(directory)

        for sensor_head in self.configs:
            for fov_num in self.configs[sensor_head]:
                current_config = self.configs_base_content
                current_config['device_frame_id'] = "m20_"+sensor_head+"_"+str(fov_num)
                self.yaml.dump(current_config, open(os.path.join(directory, 'm20_configs_'+sensor_head+'_'+str(fov_num)+'.yaml'), 'w+', encoding='utf8'))


class RVIzFileCreator:
    """
        Class to generate the rviz config file

        config_dict must a dict in which each key is a sensor port (a, b, c or d) and each value is a list of the desired FoVs (0 to 7)
    """
    def __init__(self, config_dict, pkg_path=None):
        self.yaml = ruamel.yaml.YAML()
        self.yaml.preserve_quotes = True
        self.configs = config_dict
        self.pkg_path = pkg_path
        self.rviz_base_content = self.yaml.load(open(os.path.join(self.pkg_path, 'rviz', 'multi_m20', 'multi_m20_base_config.rviz'), 'r'))
        self.link_base = ordereddict([('Alpha', 1), ('Show Axes', False), ('Show Trail', False), ('Value', True)])
        self.single_pointcloud2_display = ordereddict([('Alpha', 1), ('Autocompute Intensity Bounds', False), ('Autocompute Value Bounds', ordereddict([('Max Value', 10), ('Min Value', -10), ('Value', True)])), ('Axis', 'Z'), ('Channel Name', 'intensity'), ('Class', 'rviz_default_plugins/PointCloud2'), ('Color', '255; 255; 255'), ('Color Transformer', ''), ('Decay Time', 0), ('Enabled', True), ('Invert Rainbow', False), ('Max Color', '255; 255; 255'), ('Max Intensity', 2000), ('Min Color', '0; 0; 0'), ('Min Intensity', 0), ('Name', 'PointCloud2'), ('Position Transformer', ''), ('Selectable', True), ('Size (Pixels)', 2), ('Size (m)', 0.014000000432133675), ('Style', 'Points'), ('Topic', ordereddict([('Depth', 5), ('Durability Policy', 'Volatile'), ('History Policy', 'Keep Last'), ('Reliability Policy', 'Reliable'), ('Value', '/lumotive_ros/pointcloud_a')])), ('Use Fixed Frame', True), ('Use rainbow', True), ('Value', True)])

    def dump_rviz_config_file(self, directory):
        for sensor_head in self.configs:
            for fov_num in self.configs[sensor_head]:
                pt2_instance = self.single_pointcloud2_display
                pt2_instance['Name'] = "PointCloud_"+sensor_head+"_"+str(fov_num)
                pt2_instance['Topic']['Value'] = "/lumotive_ros/pointcloud_"+sensor_head+"_"+str(fov_num)
                self.rviz_base_content['Visualization Manager']['Displays'][1]['Links']["m20_"+sensor_head+"_"+str(fov_num)] = copy.deepcopy(self.link_base)
                self.rviz_base_content['Visualization Manager']['Displays'].append(copy.deepcopy(pt2_instance))

        self.yaml.dump(self.rviz_base_content, open(os.path.join(directory, 'multi_m20.rviz'), 'w+', encoding='utf8'))


class RvizTiledFileCreator:
    """Class to generate multiple tiled rviz config files

    config_dict must a dict in which each key is a sensor port (a, b, c or d)
    and each value is a list of the desired FoVs (0 to 7)
    """
    def __init__(self, config_dict, pkg_path=None):
        self.yaml = ruamel.yaml.YAML()
        self.yaml.preserve_quotes = True
        self.configs = config_dict
        self.pkg_path = pkg_path
        self.rviz_base_content = self.yaml.load(open(os.path.join(self.pkg_path, 'rviz', 'multi_m20', 'multi_m20_base_config.rviz'), 'r'))
        self.link_base = ordereddict([('Alpha', 1), ('Show Axes', False), ('Show Trail', False), ('Value', True)])
        self.single_pointcloud2_display = ordereddict([('Alpha', 1), ('Autocompute Intensity Bounds', False), ('Autocompute Value Bounds', ordereddict([('Max Value', 10), ('Min Value', -10), ('Value', True)])), ('Axis', 'Z'), ('Channel Name', 'intensity'), ('Class', 'rviz_default_plugins/PointCloud2'), ('Color', '255; 255; 255'), ('Color Transformer', ''), ('Decay Time', 0), ('Enabled', True), ('Invert Rainbow', False), ('Max Color', '255; 255; 255'), ('Max Intensity', 2000), ('Min Color', '0; 0; 0'), ('Min Intensity', 0), ('Name', 'PointCloud2'), ('Position Transformer', ''), ('Selectable', True), ('Size (Pixels)', 2), ('Size (m)', 0.014000000432133675), ('Style', 'Points'), ('Topic', ordereddict([('Depth', 5), ('Durability Policy', 'Volatile'), ('History Policy', 'Keep Last'), ('Reliability Policy', 'Reliable'), ('Value', '/lumotive_ros/pointcloud_a')])), ('Use Fixed Frame', True), ('Use rainbow', True), ('Value', True)])

    def dump_rviz_config_file(self, directory):
        for fov_num in range(4):
            starting_base_content = copy.deepcopy(self.rviz_base_content)
            pt2_instance = copy.deepcopy(self.single_pointcloud2_display)
            for sensor_head in self.configs:
                pt2_instance['Name'] = "PointCloud_"+sensor_head+"_"+str(fov_num)
                pt2_instance['Topic'] = "/lumotive_ros/pointcloud_"+sensor_head+"_"+str(fov_num)
                starting_base_content['Visualization Manager']['Displays'][1]['Links']["m20_"+sensor_head+"_"+str(fov_num)] = copy.deepcopy(self.link_base)
                starting_base_content['Visualization Manager']['Displays'].append(copy.deepcopy(pt2_instance))

            # Window size
            # width
            pw = subprocess.check_output("xrandr | awk -F'[ +]' '/primary/{print $4}' | cut -d x -f1",
                                         shell=True)
            width = int(pw)
            w = int(width / 2.25)
            ph = subprocess.check_output("xrandr | awk -F'[ +]' '/primary/{print $4}' | cut -d x -f2",
                                         shell=True)
            height = int(ph)
            h = int(height / 2.25)
            x = int((fov_num >> 1) * width // 2)
            y = int((fov_num & 0b1) * height // 2 + ((fov_num & 0b1) * height * .15))
            #print('fw fh w h x y', width, height, w, h, x, y)
            # Adjust window size and location based on which FOV num

            starting_base_content['Window Geometry']['Width'] = w
            starting_base_content['Window Geometry']['Height'] = h

            starting_base_content['Window Geometry']['X'] = x
            starting_base_content['Window Geometry']['Y'] = y
            starting_base_content['Window Geometry']['Hide Left Dock'] = True
            starting_base_content['Window Geometry']['Hide Right Dock'] = True
            starting_base_content['Window Geometry']['Displays']['collapsed'] = True
            starting_base_content['Window Geometry']['Views']['collapsed'] = True

            self.yaml.dump(starting_base_content, open(
                os.path.join(directory, 'multi_m20' + '_' + str(fov_num) + '.rviz'),
                'w+', encoding='utf8'))


def generate_launch_description():

    ld = LaunchDescription([
        DeclareLaunchArgument('sensor_ip_', default_value="", description='IP of Lumotive sensor to connect to.'),
        DeclareLaunchArgument('split_fov_', default_value="0", description='Use this to vertically split the FoV of each sensor head (specify a value in meters).'),
    ])

    sensor_ip = ""
    split_fov = 0
    for arg in sys.argv:
        if arg.startswith("sensor_ip_:="):
            sensor_ip = arg.split(":=")[1]
        if arg.startswith("split_fov_:="):
            split_fov = int(arg.split(":=")[1])

    # Find lumotive_ros path
    pkg_top = get_package_share_directory('ylm_ros2')

    base_port = 10940
    all_configuration = {'a': [0, 1, 2, 3, 4, 5, 6, 7],
                        'b': [0, 1, 2, 3, 4, 5, 6, 7],
                        'c': [0, 1, 2, 3, 4, 5, 6, 7],
                        'd': [0, 1, 2, 3, 4, 5, 6, 7]}
    ports = {'a': {0: base_port, 1: base_port+1, 2: base_port+2, 3: base_port+3, 4: base_port+4, 5: base_port+5, 6: base_port+6, 7: base_port+7},
            'b': {0: base_port+8, 1: base_port+9, 2: base_port+10, 3: base_port+11, 4: base_port+12, 5: base_port+13, 6: base_port+14, 7: base_port+15},
            'c': {0: base_port+16, 1: base_port+17, 2: base_port+18, 3: base_port+19, 4: base_port+20, 5: base_port+21, 6: base_port+22, 7: base_port+23},
            'd': {0: base_port+24, 1: base_port+25, 2: base_port+26, 3: base_port+27, 4: base_port+28, 5: base_port+29, 6: base_port+30, 7: base_port+31}}

    # Sensor head keys
    sensor_head_keys = list(all_configuration.keys())
    sensor_positions_keys = [s + '_pos' for s in sensor_head_keys]
    sensor_visual_keys = [s + '_vis' for s in sensor_head_keys]

    yaml = ruamel.yaml.YAML()
    full_configuration = dict(yaml.load(open(os.path.join(pkg_top, 'launch', 'm20_swdl_configs.yaml'), 'r')))
    configuration = {key: full_configuration[key] for key in sensor_head_keys if key in full_configuration.keys()}
    positions = {key: full_configuration[key] for key in sensor_positions_keys if key in full_configuration.keys()}
    visual_positions = {key: full_configuration[key] for key in sensor_visual_keys if key in full_configuration.keys()}

    ################################################################################################
    # Check the input data
    for sensor_head in configuration.keys():
        if sensor_head not in all_configuration.keys():
            raise Exception("Sensor head '" + sensor_head + "' is not valid. Valid sensor heads are: " + str(list(all_configuration.keys())))
        for fov_num in configuration[sensor_head]:
            if fov_num not in all_configuration[sensor_head]:
                raise Exception("Sensor head " + sensor_head + " has no FoV " + str(fov_num) + ". Valid FoVs for sensor head " + sensor_head + " are: " + str(all_configuration[sensor_head]))
        if sensor_head + "_pos" not in positions.keys():
            raise Exception("Sensor head " + sensor_head + " has no position specified (no " + sensor_head + "_pos key)")
        if sensor_head + "_vis" not in visual_positions.keys():
            raise Exception("Sensor head " + sensor_head + " has no visual origin specified (no " + sensor_head + "_vis key)")
    ################################################################################################

    urdf_path = os.path.join(pkg_top, 'urdf', 'multi_m20.urdf')
    urdf_creator = URDFFileCreator(configuration, positions, visual_positions, fov_split_distance=split_fov, pkg_path=pkg_top)
    urdf_creator.generate_XML()
    Path(os.path.join(pkg_top, 'urdf')).mkdir(parents=False, exist_ok=True)
    urdf_creator.write_to_file(urdf_path)

    # rviz_creator = RVIzFileCreator(configuration, pkg_path=pkg_top)
    rviz_creator = RvizTiledFileCreator(configuration, pkg_path=pkg_top)
    Path(os.path.join(pkg_top, 'rviz', 'multi_m20')).mkdir(parents=False, exist_ok=True)
    rviz_creator.dump_rviz_config_file(os.path.join(pkg_top, 'rviz', 'multi_m20'))

    # load yaml file
    config_path = os.path.join(pkg_top, 'config', 'm20_configs.yaml')
    yaml_configs = dict(yaml.load(open(config_path, 'r')))
    configs = yaml_configs['lumotive_ros_params']
    configs['sensor_ip'] = sensor_ip

    # Generate launch nodes here
    for sensor_head in configuration.keys():
        for fov in configuration[sensor_head]:
            configs['device_frame_id'] = 'm20_'+sensor_head+'_'+str(fov)
            configs['sensor_port'] = ports[sensor_head][fov]
            ld.add_action(Node(package='ylm_ros2',
                            executable='lumotive_driver',
                            name='lumotive_driver_'+sensor_head+'_'+str(fov),
                            remappings=[('/lumotive_ros/pointcloud', '/lumotive_ros/pointcloud_'+str(sensor_head)+'_'+str(fov))],
                            output='screen',
                            parameters=[configs]))

    # load URDF and rviz
    urdf = open(urdf_path).read()
    robot_state_publisher_node = Node(package='robot_state_publisher',
                                    executable='robot_state_publisher',
                                    name='robot_state_publisher',
                                    output='screen',
                                    parameters=[{'robot_description': urdf}],
                                    arguments=[urdf_path],
                                    )
    joint_state_publisher_node = Node(package='joint_state_publisher',
                                    executable='joint_state_publisher',
                                    output='screen',
                                    arguments=[urdf_path],
                                    )

    rviz_node_list = []
    for fov in reversed(range(4)):
        rviz_node_list.append(Node(package='rviz2', executable='rviz2', name='rviz2', output='screen', arguments=['-d', os.path.join(pkg_top, 'rviz', 'multi_m20', 'multi_m20_' + str(fov) + '.rviz')]))

    ld.add_action(robot_state_publisher_node)
    ld.add_action(joint_state_publisher_node)
    for fov in reversed(range(4)):
        ld.add_action(rviz_node_list[fov])
    
    return ld