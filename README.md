demo_pioneer
============

A set of demos using a Pioneer robot and based on ViSP. This package is documented on [ROS wiki] [demo_pioneer-wiki].

# Installation

## Get the source

	$ cd ~/catkin_ws/src

Get rosaria stack that allows to control a real Pioneer robot

	$ git clone https://github.com/amor-ros-pkg/rosaria.git (master branch)

Get demo_pioneer stack that does the visual servoing

	$ git clone https://github.com/lagadic/demo_pioneer.git (master branch)

## Update the dependencies to control the real Pionner

	$ source ~/catkin_ws/devel/setup.bash
	$ rosdep install rosaria
	$ rosdep install demo_pioneer

## Build the catkin packages from source

	$ cd ~/catkin_ws
	$ catkin_make -DCMAKE_BUILD_TYPE=Release --pkg rosaria
	$ catkin_make -DCMAKE_BUILD_TYPE=Release --pkg demo_pioneer

# Usage

## Test pioneer tele operation from gamepad

Check if you have the read/write rights in /dev/ttyUSB0. If not

	$ sudo chmod a+rw /dev/ttyUSB0

or even better edit '/etc/udev/rules.d/51-local.rules' and add the following line:

	$ KERNEL=="ttyUSB*", MODE="0666"

First check if the Pioneer can be tele operated

	$ source ~/catkin_ws/devel/setup.bash
	$ roslaunch demo_pioneer pioneer-teleop.launch

## Start the visual servoing demo on the Pioneer

Then start the demo with a real Pioneer robot using:

	$ source ~/catkin_ws/devel/setup.bash
	$ roslaunch demo_pioneer demo-visual-servo-pioneer.launch

# Usage only in the lab where the Pioneer is equipped with a Biclops PT head
 
## Prerequisities

Get visp_ros stack that does the control of the Biclops head using ViSP vpRobotBiclops class, a wrapper over Biclops low level controller that is not open source 

	$ cd ~/catkin_ws/src
	$ git clone https://github.com/lagadic/visp_ros.git (master branch)
	$ cd ~/catkin_ws
	$ catkin_make -DCMAKE_BUILD_TYPE=Release --pkg visp_ros

## Start the visual servoing demo on the Pioneer equipped with a Biclops PT head

Then start the demo with a real Pioneer robot using:

	$ source ~/catkin_ws/devel/setup.bash
	$ roslaunch visp_ros biclops.launch
	$ roslaunch demo_pioneer demo-visual-servo-pioneer-pan.launch


[demo_pioneer-wiki]: http://wiki.ros.org/demo_pioneer
