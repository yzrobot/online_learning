# Multisensor Online Transfer Learning for 3D LiDAR-based Human Detection with a Mobile Robot #

[![Build Status](https://travis-ci.org/yzrobot/online_learning.svg?branch=master)](https://travis-ci.org/yzrobot/online_learning)
[![Codacy Badge](https://app.codacy.com/project/badge/Grade/63b189ca851b4a30903e19fef1a10d36)](https://www.codacy.com/gh/yzrobot/online_learning/dashboard?utm_source=github.com&amp;utm_medium=referral&amp;utm_content=yzrobot/online_learning&amp;utm_campaign=Badge_Grade)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)

This is a ROS-based online transfer learning framework for human classification in 3D LiDAR scans, taking advantage of robust multi-target tracking to avoid the need for data annotation by a human expert.

## How to build ##
```
$ cd ~/catkin_ws/src/
$ git clone -b multisensor https://github.com/yzrobot/online_learning.git
$ cd ~/catkin_ws
$ catkin_make
```

## Dataset ##
[https://lcas.lincoln.ac.uk/wp/research/data-sets-software/l-cas-multisensor-people-dataset/](https://lcas.lincoln.ac.uk/wp/research/data-sets-software/l-cas-multisensor-people-dataset/)

## Citation ##
If you are considering using this code, please reference the following:
```
@inproceedings{yz18iros,
   title={Multisensor Online Transfer Learning for 3D LiDAR-based Human Detection with a Mobile Robot},
   author={Zhi Yan and Li Sun and Tom Duckett and Nicola Bellotto},
   booktitle = {In Proceedings of the 2018 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)},
   address = {Madrid, Spain},
   month = {October},
   year = {2018}
}
```
