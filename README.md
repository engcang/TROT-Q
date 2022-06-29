# TROT-Q
#### ASCC2022 - `Eungchang Mason Lee, Jinwoo Jeon, and Hyun Myung "TROT-Q: TRaversability and Obstacle aware Target tracking system for Quadruped robots"`
+ Tracking detected target (`YOLOv4-tiny-3l` network is used)
+ `FAST-LIO2` SLAM is used to estimate the states
+ Traversability and obstacle collision are checked
+ Minimum Jerk Trajectory is tracked
+ High-level velocity MPC control
+ Low-level PID control
#### Video - https://youtu.be/hEQSOyfWevY

<br>

### Requirements
+ `OpenCV` version >= 4.4.0
+ `cv_bridge`
+ ROS and Gazebo
    + refer [here](http://wiki.ros.org/ROS/Installation)
    + `$ sudo apt install ros-<distro>-desktop-full`
+ Dependencies
~~~shell
$ sudo apt install ros-melodic-gazebo-plugins ros-melodic-ros-control ros-melodic-ros-controllers

$ wget -O ubuntu.sh https://raw.githubusercontent.com/PX4/PX4-Autopilot/master/Tools/setup/ubuntu.sh
$ source ubuntu.sh
$ sudo apt upgrade libignition-math4
~~~
+ This Repo
~~~shell
$ cd ~/your_workspace/src

$ git clone --recursive https://github.com/engcang/ascc2022

$ cd ascc2022/

$ echo "export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:$(pwd)/gazebo_maps/height_maze" >> ~/.bashrc
$ . ~/.bashrc

$ cd ..
$ catkin build -DCMAKE_BUILD_TYPE=Release
~~~

<br>

### How to run
+ Launch Gazebo world
	+ Change `world` in the launch file, if you want.
    + Check `dev` of `joystick` to manually control `Jackal` in the launch file.
~~~shell
$ roslaunch trot-q gazebo.launch
~~~
+ Run `TROT-Q`
~~~shell
$ roslaunch trot-q main.launch
~~~

<br>

### Components
+ `ANYMAL-B` Gazebo model from [here](https://github.com/ANYbotics/anymal_b_simple_description)
    + Many unnecessary files are trimmed
+ `Champ` quadruped robot controller from [here](https://github.com/chvmp/champ)
+ Gazebo map, made by myself from [here](https://github.com/engcang/gazebo_maps)
+ Gazebo Jackal Gazebo model from [here](https://github.com/jackal)
    + Many unnecessary files are trimmed
+ Gazebo Ouster - referred [here1](https://www.wilselby.com/2019/05/simulating-an-ouster-os-1-lidar-sensor-in-ros-gazebo-and-rviz/) and [here2](https://github.com/wilselby/ouster_example)
    + Many unnecessary files are trimmed
    + GPU-ray is enabled as [here](https://engcang.github.io/Ouster-Gazebo-Plugin-boosting-up-with-GPU-ray/)

### SLAM
+ [FAST-LIO2](https://github.com/hku-mars/FAST_LIO)

### Object Detection
+ ROS-YOLO using `OpenCV` code: from [here (myself)](https://github.com/engcang/ros-yolo-sort/blob/master/YOLO_and_ROS_ver/ros_opencv_dnn.py)
