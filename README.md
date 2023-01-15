# IMU - FLOAM 
## Fast LOAM (Lidar Odometry And Mapping)

This work is an optimized version of FLOAM which uses an IMU to aid odometry estimation


**Modifier:** [Daniel Adolfsson](https://www.linkedin.com/in/daniel-adolfsson-7613417a/),

### List of modifications
- IMU is used to deskew point cloud in laserProcessingNode.cpp
    - Assumpitions: high frequency, no linear motion (not valid - please fix)
- IMU is used to predict movement of sensor
    - This is quickly implemented directly in laserProcessingNode.cpp - a side effect is that estimated orientation will appear fixed.

### To do
- Compesate for linear velocity - to gain up to 0.1m less noise for motion at 1m/s
- Improve map represetation in scan matcher
- Cauchy loss function in solver.



## 1. Prerequisites

### 1.0 IMU
IMU. It is assumed that the IMU to lidar extrinsic parameters are eulerRPY=[0, 0, 180] deg
 
### 1.1 **Ubuntu** and **ROS**
Ubuntu 64-bit 18.04.

ROS Melodic. [ROS Installation](http://wiki.ros.org/ROS/Installation)

### 1.2. **Ceres Solver**
Follow [Ceres Installation](http://ceres-solver.org/installation.html).

### 1.3. **PCL**
Follow [PCL Installation](http://www.pointclouds.org/downloads/linux.html).

### 1.4. **Trajectory visualization**
For visualization purpose, this package uses hector trajectory sever, you may install the package by 
```
sudo apt-get install ros-melodic-hector-trajectory-server
```
Alternatively, you may remove the hector trajectory server node if trajectory visualization is not needed

## 2. Build 
### 2.1 Clone repository:
```
    cd ~/catkin_ws/src
    git clone https://github.com/dan11003/floam
    cd ..
    catkin_make
    source ~/catkin_ws/devel/setup.bash
```


### 3 Launch ROS
```
    roslaunch floam structor.launch
```
if you would like to create the map at the same time, you can run (more cpu cost)
```
    roslaunch floam floam_mapping.launch
```
If the mapping process is slow, you may wish to change the rosbag speed by replacing "--clock -r 0.5" with "--clock -r 0.2" in your launch file, or you can change the map publish frequency manually (default is 10 Hz)




