YLM ROS2 driver
====================================

This repository holds the ROS2 driver for some of the YLM devices.


ROS2 Workspace Setup
-----------------------
In order to properly build the Lumotive ROS2 driver the colcon (build tool for ROS2 drivers) workspace must be properly set up.
To source the workspace **on Linux** with **Foxy**:
```bash
source /opt/ros/foxy/local_setup.bash
mkdir -p ~/ros2_ws/src && cd ~/ros2_ws
colcon build
```

If you want to see the build verbose, use:
```bash
colcon build --event-handlers console_direct+
```

To source the workspace **on Windows** with **Foxy** (assuming the installation folder is C:\dev\ros2_foxy\):
```batch
call C:\dev\ros2_foxy\local_setup.bat
mkdir C:\ros2_ws\src
cd C:\ros2_ws
colcon build --event-handlers console_cohesion+ --merge-install
```

If you don't have ROS2 installed this command will fail.
Once the workspace is setup you can proceed to the Installation section.

Installation
-------------
Clone this package into the *src* folder of your ROS2 workspace using:
```
git clone https://github.com/Hokuyo-aut/ylm_ros2
```

Make sure that Boost is installed on the machine.

Build the package in the workspace using (don't forget to source your workspace afterward):

**Linux**:
```
colcon build --event-handlers console_direct+
```

**Windows** (must be done from a 'x64 Native Tools Command Prompt for VS' command prompt):
```
colcon build --event-handlers console_cohesion+ --merge-install
```

Usage
------
Linux: in a separate terminal (not the one you used to build), source your workspace:
```bash
cd <top of your ROS2 workspace>
source install/setup.bash
```

Windows: in a separate non-VS terminal, source your workspace:
```batch
cd <top of your ROS2 workspace>
call install\setup.bat
```

To launch the node with a livestream from the M20 lidar, use:
```
ros2 launch ylm_ros2 m20_launcher.launch.py sensor_ip_:=192.168.0.10
```

Tip for showing help related to input args:
```
ros2 launch ylm_ros2 m20_launcher.launch.py --show-args
```

Config file presentation
-------------------------

* device_frame_id: TF2 frame to use to publish steering angles;
* nb_packets_in_array_msg: Override for the number of packets per array (0 = automatically computed by driver based on FoV size);
* organized_cloud: This driver can publish unorganized (dense) PointCloud2 messages or organized (image-like) PointCloud2 messages. Use this flag to switch between both;
* color_range_max: Color palette range in meters when converting point cloud to depth map (organized_cloud must be True);
* colormap_name: Choose between various color palettes for depth maps;
* color_by_reflectivity: Color points by reflectivity.
* range_min: minimum range of a valid detection;
* range_max: maximum range of a valid detection;

* min_brightness: minimum value for normalized reflectivity (value in range range [0., 1.0[);
* max_brightness: maximum value that the unnormalized reflectivity can take. If this is too high, everything will appear dark;


Supported devices
------------------

The systems currently supported by this driver are listed below:

- HM25

Compatibility
----------------

This driver is known to work with

- ROS2 Foxy on both Ubuntu 20.04
- ROS2 Humble on both Ubuntu 22.04
