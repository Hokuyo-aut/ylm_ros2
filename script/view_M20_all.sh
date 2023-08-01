#!/bin/bash

#Present on ubuntu installs contains OS info
# source it cause it is just a bunch of variable!
. /etc/os-release

# Source ROS
source /opt/ros/foxy/setup.bash

# Make sure joint_state_publisher is installed
source apt install ros-foxy-joint-state-publisher 

# Build the workspace
colcon build --base-paths ~/ros2_ws

# Source the workspace
source ~/ros2_ws/install/local_setup.bash

# Launch
ros2 launch ylm_ros2 m20_swdl_launcher.launch.py sensor_ip_:=192.168.0.10