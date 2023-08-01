#!/bin/bash

#Present on ubuntu installs contains OS info
# source it cause it is just a bunch of variable!
. /etc/os-release

# Source ROS
if [ "$VERSION_ID" = "20.04" ]; then
 source /opt/ros/noetic/setup.bash
elif [ "$VERSION_ID" = "18.04" ]; then
 source /opt/ros/melodic/setup.bash
fi

# Check if lumotive_ros2 is in the workspace and delete it if that's the case to avoid catkin trying to build it
# Unfortunately, catkin does not like having ROS and ROS2 packages in the same workspace
rm -rf ~/catkin_ws/src/lumotive_stream_receiver/lumotive_ros2

# Build the workspace
catkin_make -C ~/catkin_ws

# Source the workspace
source ~/catkin_ws/devel/setup.bash

# Update ros dependencies
rosdep install --from-paths ~/catkin_ws/src/lumotive_stream_receiver/lumotive_ros --ignore-src -r -y

# Run generation script
if [ "$VERSION_ID" = "20.04" ]; then
 python3 ~/catkin_ws/src/lumotive_stream_receiver/lumotive_ros/scripts/multi_m20_files_generator.py ~/catkin_ws/src/lumotive_stream_receiver/lumotive_ros/scripts/multi_m20_gen_configs.yaml
elif [ "$VERSION_ID" = "18.04" ]; then
 python ~/catkin_ws/src/lumotive_stream_receiver/lumotive_ros/scripts/multi_m20_files_generator.py ~/catkin_ws/src/lumotive_stream_receiver/lumotive_ros/scripts/multi_m20_gen_configs.yaml
fi

roslaunch lumotive_ros multi_m20_node_launcher.launch sensor_ip_:=192.168.0.10
