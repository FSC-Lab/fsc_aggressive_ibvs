# Perception-Aware Image-Based Visual Servoing of Aggressive Quadrotor UAVs

## I. Overview

This IBVS control algorithm is designed for aggressive quadrotors to ensure target visibility during agile flight.

**Authors**: Chao Qin, QIuyu Yu, Helso Go, and Hugh H.-T. Liu from [FSC lab](https://www.flight.utias.utoronto.ca/fsc/)

**Paper**: [Perception-Aware Image-Based Visual Servoing of Aggressive Quadrotor UAVs](https://ieeexplore.ieee.org/abstract/document/10140151), Qin, Chao; Yu, Qiuyu; Go, Shing Hei Helson; Liu, Hugh H. -T., in IEEE/ASME Transactions on Mechatronics, 2023

**Video Links**: [youtube](https://www.youtube.com/watch?v=X2-SMGD99oA)

![fig2](https://github.com/FSC-Lab/fsc_aggressive_ibvs/blob/main/docs/fig_exp.png)



## II. Installation

### Get ROS

This framework is based on the Robot Operating System ([ROS](http://www.ros.org/)) and you therefore first need to install it (Desktop-Full Install) by following the steps described in the [ROS Installation](http://wiki.ros.org/ROS/Installation).

### Get catkin tools

Get catkin tools with the following commands:

```
sudo apt-get install python-pip
sudo pip install catkin-tools
```

### Create a catkin workspace

Create a catkin workspace with the following commands by replacing `<ROS VERSION>` with the actual version of ROS you installed:

```
cd
mkdir -p catkin_ws/src
cd catkin_ws
catkin config --init --mkdirs --extend /opt/ros/$ROS_DISTRO --merge-devel --cmake-args -DCMAKE_BUILD_TYPE=Release
cd src
```

### Get catkin_simple

```
git clone https://github.com/catkin/catkin_simple
```


### Get eigen_catkin

```
git clone https://github.com/ethz-asl/eigen_catkin
```
### Get mav_comm

```
git clone https://github.com/ethz-asl/mav_comm
```

### Get rotors_simulator

```
git clone https://github.com/ethz-asl/rotors_simulator
```

### Clone this repository:

```
git clone https://github.com/FSC-Lab/fsc_aggressive_ibvs
```

### Configure RotorS

```
mv src/fsc_quadrotor_control/mav_fpv_sensor.gazebo src/rotors_simulator/rotors_description/urdf
```

We modify the weight of the simulated camera to zero for better visualization. At the ```<!-- VI-Sensor Macro -->``` in```src/rotors_simulator/rotors_description/urdf/component_snippets.xacro```

change ```<mass value="0.13" />``` to ```<mass value="0.0" />```



### Compile the source code

```
cd ..
catkin build
```



## III. Target Tracking Simulation

```
source devel/setup.bash
roslaunch fsc_autopilot run_autopilot_rotors_planning.launch 
```

Click feature track to switch the control from position control to our IBVS


![fig2](https://github.com/FSC-Lab/fsc_aggressive_ibvs/blob/main/docs/fig_rqt_interface_02.png)

Specify the desired distance 
![fig3](https://github.com/FSC-Lab/fsc_aggressive_ibvs/blob/main/docs/fig_rqt_interface_03.png)

And you will see the resulting flight trajectory in rviz.

<p align="center">
    <img src="https://github.com/FSC-Lab/fsc_aggressive_ibvs/blob/main/docs/video_tracking.gif" width="500"/>
</p>


