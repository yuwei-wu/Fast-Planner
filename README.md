# Fast-Planner 

This is the revised version on Ubuntu 20.04/ROS noetic, which can work with the controller here: https://github.com/KumarRobotics/kr_mav_control

Independent planning module, code revised from fast planner: https://github.com/HKUST-Aerial-Robotics/Fast-Planner


Clone the repo and complie it:

```console
$ git clone -b exp_tech git@github.com:yuwei-wu/Fast-Planner.git
$ wstool init && wstool merge Fast-Planner/fast.rosinstall && wstool update
$ cd ..
$ catkin build
```

To run the code:

you can directly run the shell code


