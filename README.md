# Fast-Planner 

This is the revised version on Ubuntu 20.04/ROS noetic.

Independent planning module, code revised from fast planner: https://github.com/HKUST-Aerial-Robotics/Fast-Planner

You need to install nlopt in your workspace:  https://github.com/ethz-asl/nlopt


Clone the repo:

```console
$ git clone https://github.com/yuwei-wu/Fast-Planner.git

```
To simply set up mapping and simulator in the Fast-Planner

```console
$ cd Fast-Planner
$ git clone https://github.com/ethz-asl/nlopt.git
$ wstool init && wstool merge fast.rosinstall && wstool update
```


