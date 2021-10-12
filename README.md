# Fast-Planner 

This is the revised version on Ubuntu 20.04/ROS noetic, which can work with the controller here: https://github.com/KumarRobotics/kr_mav_control

Independent planning module, code revised from fast planner: https://github.com/HKUST-Aerial-Robotics/Fast-Planner


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

To run the code:

terminal 1:

```
roslaunch plan_manage  rviz.launch

```
terminal 2:

```
 roslaunch plan_manage  fast.launch sim:=true vicon:=false mav_name:=quadrotor random_map:=true
```

terminal 3:

```
rosrun rqt_mav_manager rqt_mav_manager

```
