# A-Star Algorithm on Turtlebot
[![Build Status](https://travis-ci.org/urastogi885/a-star-turtlebot.svg?branch=master)](https://travis-ci.org/urastogi885/a-star-turtlebot)
[![License](https://img.shields.io/badge/License-BSD%203--Clause-blue.svg)](https://github.com/urastogi885/a-star-turtlebot/blob/master/LICENSE)

## Overview
In this project, a-star algorithm is simulated on a differential drive (non-holonomic) mobile robot in a defined static 
world. We are using Turtlebot in ROS-Gazebo for the simulation.

- Phase-1: Installation of ROS and V-REP
- Phase-2: Implementation of A* on a non-holonomic robot
- Phase-3: Implementation of A* on a differential-drive robot
- Phase-4: Implementation of A* on Turtlebot using ROs

Phases 1 and 2 have been implemented on another [repository](https://github.com/urastogi885/a-star-robot)

## Authors

- [Umang Rastogi](https://github.com/urastogi885)
- [Naman Gupta](https://github.com/namangupta98)

## For TAs

- The following repositories contain implementation of both Phase-3 and Phase-4 of Project-3.
- You can access the submission version from the [*release section*](https://github.com/urastogi885/a-star-turtlebot/releases)
- For the 2 videos, the following start and goal points were used repsectively:
	- First video: Start-(-4,-3,0) Goal-(0,-3)
	- Second video: Start-(-4,-4.5,0) Goal-(4.25,2.75)
- There are 2 launch files, namely, one for each video mentioned above. They have been indexed accordingly.

## Dependencies

- Ubuntu 16.04/18.04
- ROS Kinetic
- Gazebo
- Turtlebot3 Packages
- Python Packages: Numpy, OpenCV-Python, Math, Queue

## Install Dependencies

- Install Python3, Python3-tk, and the necessary libraries: (if not already installed)

```
sudo apt install python3 python3-tk
sudo apt install python3-pip
pip3 install numpy opencv-python
```

- Check if your system successfully installed all the dependencies
- Open terminal using Ctrl+Alt+T and enter python3.
- The terminal should now present a new area represented by >>> to enter python commands
- Now use the following commands to check libraries: (Exit python window using Ctrl+Z if an error pops up while running 
the below commands)

```
import tkinter
import numpy
import cv2
import math
import queue
```

- If you want to work on Ubuntu 16.04, you will need to install [ROS Kinetic](http://wiki.ros.org/kinetic/Installation/Ubuntu)
by following the instructions given on referenced web-page.
- If you want to work on Ubuntu 18.04, you will need to install [ROS Melodic](http://wiki.ros.org/melodic/Installation/Ubuntu)
by following the instructions given on referenced web-page.
- We recommend installing the full-desktop version of ROS because it automatically installs the latest compatible version of
Gazebo on your system.
- If you wish to install Gazebo separately, then follow the instruction on the [Gazebo install page](http://gazebosim.org/tutorials?tut=install_ubuntu&cat=install).
- Install Turtlebot-3 package and its dependencies using this [link](https://programmer.help/blogs/ubuntu-18.04-lts-melodic-ros-configuration-turtlebot-3-running-gazebo-simulation.html).
 
## Run Instructions

- Clone the repository in your ROS workspace. Type
```
cd ~/<ROS_Workspace>/src
git clone https://github.com/urastogi885/a-star-turtlebot
```

- If you have a compressed version of the project, extract it, go into project directory, open the terminal, and type:

```
python3 a_star_turtlebot.py start_x,start_y,start_orientation goal_x,goal_y rpm1,rpm2,clearance animation
python3 a_star_turtlebot.py -4,-4.5,0 4.25,2.75 30,25,5 0
```

- This would generate a text file that will be used to run the Turtlebot in Gazebo.
- Note the following to try the code with other inputs:
    - The origin of the map is taken at the center of the map s make sure to take into consideration in which 
    quadrant your points lie.
    - Provide start and goal coordinates in meters while start orientation is to be provided in degrees.
    - In addition to that, clearance is taken in centimeters.
    - Refer documentation of [*a_star_turtlebot.py*](https://github.com/urastogi885/a-star-turtlebot/blob/master/a_star_turtlebot.py) to
    understand further about input arguments.
- Launch the world, spawn turtlebot, navigate it to the desired goal point. Type:

```
cd ~/<ROS_Workspace>
source devel/setup.bash
catkin_make or catkin build (For ROS Melodic, you might have to use catkin build instead of catkin_make)
export TURTLEBOT3_MODEL=burger
roslaunch a-star-turtlebot launcher_1.launch
roslaunch a-star-turtlebot launcher_2.launch
```
