# Online Learning for Human Detection in 3D Point Clouds

[![Build Status](https://travis-ci.org/yzrobot/online_learning.svg?branch=master)](https://travis-ci.org/yzrobot/online_learning)
[![Codacy Badge](https://app.codacy.com/project/badge/Grade/63b189ca851b4a30903e19fef1a10d36)](https://www.codacy.com/gh/yzrobot/online_learning/dashboard?utm_source=github.com&amp;utm_medium=referral&amp;utm_content=yzrobot/online_learning&amp;utm_campaign=Badge_Grade)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)

# Build

If you don't have ROS Melodic yet, please follow the [official tutorial](http://wiki.ros.org/melodic/Installation/Ubuntu) to install it and create your [catkin_ws](http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment).

Then,

```bash
$ sudo apt-get install ros-melodic-velodyne-pointcloud ros-melodic-people-msgs ros-melodic-leg-detector libsvm-dev libsvm-tools
$ cd ~/catkin_ws/src/
$ git clone -b multi-sensor-ros-melodic https://github.com/yzrobot/online_learning.git
$ cd ~/catkin_ws
$ catkin_make
```

# Datasets

[L-CAS Multisensor People Dataset](https://lcas.lincoln.ac.uk/wp/research/data-sets-software/l-cas-multisensor-people-dataset/)

# Run

```bash
$ roslaunch online_learning object3d_detector_ol_ms.launch bag:=/path_to_your/LCAS_MS_20160523_1200_1218.bag
```
