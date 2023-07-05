# GPRL
It is a ROS package that implements a GPRL: GPS-denied Multi-Robot Relative Localization Using Hierarchical Gaussian Process (GP) Inferencing on Wireless Signals. It uses Gaussian process regression with hierarchical inferencing for access point position prediction and vector transformation for relative localization. This package contains robotarium and ROS package which has 3 ROS nodes:

  - Access Point Prediction Node.
  - Position Prediction Node.
  - Rendezvous Node
 
## Overview
The system first trains the GP for a few iterations and then keeps optimizing GP during the navigation of the robot. The robot running the GP predicts the access point position and then shares it with other robots for relative localization. An overview of the proposed approach can be found in the Figure below:

![Overview](/images/gprl_overview.png)
## Architecture
The overall functionality of GPRL with respect to single robot and information sharing can be seen in the architecture figure below:

![Overview](/images/gprl_architecture.png)

## Installation Requirements
* C++ requirements.   
([pybind11](https://github.com/pybind/pybind11) is also required, but it's built into this repository, you don't need to install)
* python 3.6+
* [Robotarium Installation](https://pypi.org/project/robotarium-python-simulator/) is required for robotarium simulation
* Robots setup with mapping and navigation stack proper TF-Tree with the multi-robotic configuration; topics should be /tb2_0/odom, /tb2_1/odom etc.


## 2. Installation
Download the package and place it inside the ```/src``` folder in catkin workspace. And then compile using ```catkin_make```.

## 3. Simulation:
Run the robotarium script for GPRL as:
``` python3 gprl_robotarium.py ```

## 4. Compile package
Run the GPRL package after installation on a robot and source bash and ~/catkin_explore/devel/setup.sh file:
```
$ mkdir -p catkin_explore/src
$ cd catkin_explore/src
$ git clone https://github.com/herolab-uga/gp-multi-robot-localization.git
$ cd ~/catkin_explore/
$ catkin_make
```

## 5. Real Robot Experimentation
Setup bring-up of robots and make sure they are publishing all the topic in the correct frames, the launch gp-loc file as:
``` roslaunch gprl gprl.launch robot_name:=<tb2_0> robot_number:=<0>  ```
> robot_name can be your robots name and robot_number starts from 0 to n.

### 5.1. Robots Network
For the cooperative robotic configuration, the package doesn't require special network configuration, it simply works by having a single ROS master (can be one of the robots). So on the other robots, the ```ROS_MASTER_URI``` parameter should be pointing at the master's address. 
For more information on setting up ROS on multiple machines, follow [this](http://wiki.ros.org/ROS/NetworkSetup) link.

### 5.2. Robot's frame names in ```tf```
All robot frames should be prefixed by their name. The naming of robots starts from "/tb2_0", "/tb2_1", ... and so on.

### 5.3. Robot's node and topic names
All the nodes and topics running on a robot must also be prefixed by its name. For tb2_0, node names should look like:  ```/tb3_2/ap_pos```.

And topic names should be like: ```/tb3_0/odom```,  ```/tb3_0/ap_pos```,  ```/tb3_0/tb2_1_rel_pos```, ..etc.

## 6. Rendezvous Experiment
To perform rendezvous run the rendezvous node as:
``` ROS_NAMESPACE=<tb2_0> rosrun gprl rendezvous.py _robot_name:=<tb2_0> _robot_count:=<n> _robot_number:=<0> ```
> Set all the parameters and namespace accordingly.


## Core contributors

* **Ehsan Latif** - PhD Candidate

* **Dr. Ramviyas Parasuraman** - Lab Director


## Heterogeneous Robotics (HeRoLab)

**Heterogeneous Robotics Lab (HeRoLab), School of Computing, University of Georgia.** 

For further information, contact Ehsan Latif ehsan.latif@uga.edu or Dr. Ramviyas Parasuraman ramviyas@uga.edu

https://hero.uga.edu/

<p align="center">
<img src="http://hero.uga.edu/wp-content/uploads/2021/04/herolab_newlogo_whitebg.png" width="300">
</p>
