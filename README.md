YLM ROS driver
======================

This repository holds the ROS driver for some of the YLM devices.

Installation
-------------

Clone this package into the *src* folder of your catkin workspace using:
```
git clone https://github.com/Hokuyo-aut/ylm_ros
```

To build this package, we strongly recommend that you use [catkin_tools](https://catkin-tools.readthedocs.io/en/latest/index.html). You can install it on Ubuntu by following the instructions provided [here](https://catkin-tools.readthedocs.io/en/latest/installing.html#installing-on-ubuntu-with-apt-get).

Build the package in the workspace. With catkin_tools and from *your ROS workspace root*:

```
source /opt/ros/<your_ROS_version>/setup.bash
catkin init
catkin build
source devel/setup.bash
```

With catkin_make (assuming you've already created a [catkin workspace](http://wiki.ros.org/catkin/Tutorials/create_a_workspace)):
```
source /opt/ros/<your_ROS_version>/setup.bash
catkin_make
source devel/setup.bash
```

Usage
------

Make sure that you are using this driver with one of the supported device. Don't forget to source your terminal. To launch the node with a livestream from the M20 lidar, use:
```
roslaunch ylm_ros m20_node_launcher.launch sensor_ip_:=192.168.0.10
```

Tip for showing help related to input args:
```
roslaunch ylm_ros m20_node_launcher.launch --ros-args
```

Tests
------
You can build/run tests with
```
catkin_make -Dtest=on run_tests
```
or with catkin_tools use:
```
catkin build -Dtest=on
catkin test ylm_ros
```

**Warning**: If you built using the test option you need to rebuild without the test option (follow the instructions in the **Installation** section above) to use the driver normally.

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

- ROS Noetic
- ROS Melodic


