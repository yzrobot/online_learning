# Online learning for human classification in 3D LiDAR-based tracking #

[![Build Status](https://travis-ci.org/yzrobot/online_learning.svg?branch=master)](https://travis-ci.org/yzrobot/online_learning)
[![Codacy Badge](https://api.codacy.com/project/badge/Grade/85d6393df92749238fb740e173be5bfa)](https://www.codacy.com/app/yzrobot/online_learning?utm_source=github.com&amp;utm_medium=referral&amp;utm_content=yzrobot/online_learning&amp;utm_campaign=Badge_Grade)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)

This is a ROS-based online learning framework for human classification in 3D LiDAR scans, taking advantage of robust multi-target tracking to avoid the need for data annotation by a human expert.
Please watch the videos below for more details.

[![YouTube Video 1](https://img.youtube.com/vi/bjztHV9rC-0/0.jpg)](https://www.youtube.com/watch?v=bjztHV9rC-0)
[![YouTube Video 2](https://img.youtube.com/vi/rmPn7mWssto/0.jpg)](https://www.youtube.com/watch?v=rmPn7mWssto)

## How to build ##
```
$ cd ~/catkin_ws/src/
$ git clone https://github.com/yzrobot/online_learning.git
$ cd ~/catkin_ws
$ catkin_make
```

## Citation ##
If you are considering using this code, please reference the following:
```
@inproceedings{yz17iros,
   author = {Zhi Yan and Tom Duckett and Nicola Bellotto},
   title = {Online learning for human classification in {3D LiDAR-based} tracking},
   booktitle = {In Proceedings of the 2017 IEEE/RSJ International Conference on Intelligent Robots and Systems (IROS)},
   pages = {864--871},
   address = {Vancouver, Canada},
   month = {September},
   year = {2017}
}
```
