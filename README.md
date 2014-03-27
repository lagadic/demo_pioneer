demo_pioneer
============

A set of demos using a Pioneer robot and based on ViSP


## Get the source

	$ cd ~/catkin_ws/src

Get rosaria stack that allows to control a real Pioneer robot

	$ git clone https://github.com/amor-ros-pkg/rosaria.git (master branch)

Get vision_visp stack that does the image processing

	$ git clone https://github.com/lagadic/vision_visp.git (master branch)

Get demo_pioneer stack that does the visual servoing

	$ git clone https://github.com/lagadic/demo_pioneer.git (master branch)

## Update the dependencies to control the real Pionner

	$ source ~/catkin_ws/devel/setup.bash
	$ rosdep install rosaria

## Update the dependencies to use a Logitec wireless gamepad F710

	$ sudo apt-get install ros-hydro-joy

## Build the catkin packages from source

	$ cd ~/catkin_ws
	$ catkin_make -DCMAKE_BUILD_TYPE=Release --pkg rosaria
	$ catkin_make -DCMAKE_BUILD_TYPE=Release --pkg visp_auto_tracker
	$ catkin_make -DCMAKE_BUILD_TYPE=Release --pkg demo_pioneer

## How to run

Check if you have the read/write rights in /dev/ttyUSB0. If not

	$ sudo chmod a+rw /dev/ttUSB0

or even better edit '/etc/udev/rules.d/51-local.rules' and add the following line:

	$ KERNEL=="ttyUSB*",          MODE="0666"

First check if the Pioneer can be tele operated

	$ source ~/catkin_ws/devel/setup.bash
	$ roslaunch demo_pioneer pioneer-teleop.launch


Then start the demo with a real Pioneer robot using:

	$ source ~/catkin_ws/devel/setup.bash
	$ roslaunch demo_pioneer demo-visual-servo-pioneer.launch

