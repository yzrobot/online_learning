# Online Learning for Human Detection in 3D Point Clouds

[![License: GPL v3](https://img.shields.io/badge/License-GPLv3-blue.svg)](https://www.gnu.org/licenses/gpl-3.0)

## This branch is in active development and we will let you know when it is ready.

## Build

If you don't have ROS Melodic yet, please follow the [official tutorial](http://wiki.ros.org/melodic/Installation/Ubuntu) to install it and create your [catkin_ws](http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment).

Then,

```bash
$ sudo apt-get install ros-melodic-velodyne-pointcloud ros-melodic-people-msgs ros-melodic-leg-detector libsvm-dev libsvm-tools
$ cd ~/catkin_ws/src/
$ git clone -b multi-sensor-ros-melodic https://github.com/yzrobot/online_learning.git
$ cd ~/catkin_ws
$ catkin_make
```

## Datasets

[L-CAS Multisensor People Dataset](https://lcas.lincoln.ac.uk/wp/research/data-sets-software/l-cas-multisensor-people-dataset/)

## Run

```bash
$ roslaunch online_learning object3d_detector_ol_ms.launch bag:=/path_to_your/LCAS_MS_20160523_1200_1218.bag
```
