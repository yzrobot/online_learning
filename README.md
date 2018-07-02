# Multisensor Online Transfer Learning for 3D LiDAR-based Human Detection with a Mobile Robot #

[![Build Status](https://travis-ci.org/yzrobot/online_learning.svg?branch=master)](https://travis-ci.org/yzrobot/online_learning)
[![Codacy Badge](https://api.codacy.com/project/badge/Grade/85d6393df92749238fb740e173be5bfa)](https://www.codacy.com/app/yzrobot/online_learning?utm_source=github.com&amp;utm_medium=referral&amp;utm_content=yzrobot/online_learning&amp;utm_campaign=Badge_Grade)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)

This is a ROS-based online transfer learning framework for human classification in 3D LiDAR scans, taking advantage of robust multi-target tracking to avoid the need for data annotation by a human expert.

## How to build ##
```
$ cd ~/catkin_ws/src/
$ git clone -b multisensor https://github.com/yzrobot/online_learning.git
$ cd ~/catkin_ws
$ catkin_make
```

## Citation ##
If you are considering using this code, please reference the following:
```
@article{yan2018multisensor,
  title={Multisensor Online Transfer Learning for 3D LiDAR-based Human Detection with a Mobile Robot},
  author={Yan, Zhi and Sun, Li and Duckett, Tom and Bellotto, Nicola},
  journal={arXiv preprint arXiv:1801.04137},
  year={2018}
}
```
