# Event Camera Demo
## Install
1. Install Robostack - https://github.com/RoboStack/ros-noetic
2. Install various dependancies
```
# needed for https://github.com/uzh-rpg/rpg_dvs_ros
mamba install libcaer -c tobiasrobotics
mamba install ros-noetic-camera-info-manager
mamba install ros-noetic-image-view
# c++ compile
mamba install compilers make cmake libopencv boost-cpp
# used to display 3D event plot 
mamba install pyqtgraph
```
3. Clone Repository
```
git clone https://github.com/Harl-Towne/event-camera-vpr-demonstrator.git
cd event-camera-vpr-demonstrator
git submodule init
git submodule update
```
## Usage
### Launch Files
```
# Launch appropriate nodes with ros drive for use with camera
roslaunch main_ui camera_launch.launch 

# Launch appropriate nodes without ros drive for use with bag files (bag file sold seperatly)
roslaunch main_ui bag_launch.launch
```
### Manual Launching
Run in seperate tabs
```
## node converts dvs_msgs.EventArray to demos_msgs.EventPacket
rosrun event_format_converter node
```
```
## node for filterting events, no filter capable of real time yet
rosrun filter null.py		# no filtering
# or
rosrun filter filter.py		# with tonic filter (currently too slow for real time)
```
```
## node for the demo program itself
rosrun main_ui main.py
```
<!--
## Troubleshooting
### C++ compile errors
Make sure `sysroot_linux-64` is version 2.17 (or above maybe).
```
# run in robostack enviroment
conda list | grep sys
```
Should show:
```
sysroot_linux-64          2.17                h4a8ded7_13    conda-forg
```
If not run:
```
mamba uninstall sysroot_linux-64   # maybe not needed
mamba install compilers sysroot_linux-64=2.17
```
-->
## Documentation
### Node Structure

![Node structure.](node_graph.JPG)
The `format_converter` node converts `dvs_msgs.EventArray` ROS mesages to `demo_msgs.EventPacket`. This is necesary as ROS Python isn't capable of reading `EventArray` messages fasted enough to run in real time (at least not as far as I know). This node is in `event_format_converter/src/converter_node.cpp`.

The filter node filters events between the `format_converter` and `event_demo_ui` nodes. `null_filter` is a pass through filter and `tonic_filter` is intended to remove noise but currently doesn't run in real-time. The files for these nodes are in `filter/src/`.

The `event_demo_ui` node is part of the main ui of the demo itself. 