# Multi turtlebot spinning demonstration
[![Build Status](https://travis-ci.org/uf-reef-avl/multi_spin_demo.svg?branch=melodic-devel)](https://travis-ci.org/uf-reef-avl/multi_spin_demo)
[![Main Actions Status](https://github.com/uf-reef-avl/multi_spin_demo/workflows/main/badge.svg)](https://github.com/uf-reef-avl/multi_spin_demo/actions)


This package is a demonstration of the action client-server architecture in ros. The main idea is to launch several turtlebots at the same time and make them spin to one direction. Every random amount of seconds, one of them will change its spinning direction and ask the other ones to do the same. 



**Table of Contents**
---------------------

1. [Installation](#Installation)

2. [Dependencies](#Dependencies)

3. [Usage](#Usage)



<a name="Installation"/>
## Installation

To install it, clone the multi_demo_spin remote repository into a catkin workspace on your computer and on all the turtlebot's nuc

        git clone http://192.168.1.101/AVL-Summer-18/multi_spin_demo

<a name="Dependencies"/>
## Dependencies

This package is a **ros-kinetic** package and depends mainly upon the [kobuki packages](http://wiki.ros.org/kobuki/Tutorials/Installation) in order to control the turtlebot's actions.
 


<a name="Usage"/>
## Usage
Real environment:

First of all ensure that the nucs have the user computer set as master. To do so, follow the [network configuration](http://wiki.ros.org/turtlebot/Tutorials/indigo/Network%20Configuration) tutorial. Compile the package on the user computer and on the nucs by using the ```catkin_make``` command into the catkin_workspace of each devices.

If it's not already done, connect the different nucs to the turtlebot and ssh into them from the master computer. Then source the catkin workspace of every devices with the ```source devel/setup.bash``` command.

On the master computer, modify the yaml file **turtlebot_list.yaml** to match the demonstration condition (number and names of the turtlebot, if you will use gazebo or a real tutlebot environment ...). 

Then launch the master node on the user computer with the command:

        roslaunch multi_spin_demo rosmaster_server.launch
        
Via the ssh connection, launch on every nuc the file **turtlebot_server.launch** with one of the turtlebot name specified previously in the yaml file **turtlebot_list.yaml**. For an example, if 2 nucs are set up and the names "pturtle", "mturtle" are written in the yaml file then send commands:

On the first nuc via ssh:

        roslaunch multi_spin_demo turtlebot_server.launch robot_name:=pturtle

On the second nuc via ssh:
        
        roslaunch multi_spin_demo turtlebot_server.launch robot_name:=mturtle
        
        
It's important to launch all the turtlebot/nuc specified in the yaml file and to connect the nuc to the turtlebot otherwise the demonstration won't start.

Simulated environment:

If you want to use a gazebo environment, you will have to launch an empty world in gazebo first. So launch it with the command:

        roslaunch gazebo_ros empty_world.launch

Then make sure in the **turtlebot_list.yaml** file that the launchMode of the turtle is set to ***gazebo*** and that the rigidbody of the turtles are specified also in the **gazebo_demo.launch** file , then launch the following command:

        roslaunch multi_spin_demo gazebo_demo.launch
        
Make sure that the **gazebo_ros** library is already installed on your computer.
