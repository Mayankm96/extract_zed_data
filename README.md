# extract_zed_data

## Overview

This is a ROS package for saving the data published using the [zed-ros-wrapper](https://github.com/stereolabs/zed-ros-wrapper). The package is meant for creating real world datasets using the Zed Camera. It saves the stereo images and depth images along with the respecitive camera parameters into a directory specified by the user.

The `extract_zed_data` package has been tested under [ROS](http://www.ros.org) Kinetic and Ubuntu 16.04. The source code is released under a [BSD 3-Clause license](extract_zed_data/LICENSE).

**Author: Mayank Mittal  
Maintainer: Mayank Mittal, mayankm.iitk@gmail.com**

## Installation

### Building from Source

#### Dependencies

- [Robot Operating System (ROS)](http://wiki.ros.org) (middleware for robotics),
- Following ROS Packages: [cv_bridge](http://wiki.ros.org/cv_bridge), [image_transport](http://wiki.ros.org/image_transport), [message_filters](http://wiki.ros.org/message_filters), [camera_calibration_parsers](http://wiki.ros.org/camera_calibration_parsers)

#### Building

To build from source, clone the latest version from this repository into your catkin workspace and compile the package using
```
cd catkin_ws/src
git clone https://github.com/Mayankm96/extract_zed_data.git
cd ../
catkin_make
```

## Usage

To save the data directly by reading the ROS bag run:
```
roslaunch extract_zed_data extract_rosbag_zed.launch
```

To save the data published from the zed-ros-wrapper run:
```
roslaunch extract_zed_data extract_topic_zed.launch
```

## Nodes

### extract_rosbag_zed

Reads all the messages present in the specified ROS bag and saves them into a folder.

### extract_topic_zed

Reads the messages being published using the zed-ros-wrapper and saves them into a folder.

#### Subscribed Topics

* **`/zed/left/image_rect_color/compressed`** ([sensor_msgs/CompressedImage])

	The left camera images in compressed form.

* **`/zed/right/image_rect_color/compressed`** ([sensor_msgs/CompressedImage])

	The right camera images in compressed form.

* **`/zed/depth/depth_registered`** ([sensor_msgs/Image])

	The depth camera images which can be in `32FC1` or `16UC1` encodings.


* **`/zed/left/camera_info`** ([sensor_msgs/CameraInfo])

	The left camera paramters.

* **`/zed/right/camera_info`** ([sensor_msgs/CameraInfo])

  The right camera paramters.

* **`/zed/depth/camera_info`** ([sensor_msgs/CameraInfo])

  The depth camera paramters.


## Bugs & Feature Requests

Please report bugs and request features using the [Issue Tracker](https://github.com/Mayankm96/extract_zed_data/issues).
