# A-Star Algorithm on Turtlebot
In this project, a-star algorithm is simulated on a differential drive (non-holonomic) mobile robot in a defined static world. We are using Turtlebot in ROS-Gazebo for the simulation.

## Authors
- [Umang Rastogi](https://github.com/urastogi885)
- [Naman Gupta](https://github.com/namangupta98)

## Dependencies
- Ubuntu 16.04/18.04
- ROS Kinetic
- Gazebo
- Turtlebot3 Packages

## Install Dependencies
- If you want to work on Ubuntu 16.04, you will need to install [ROS Kinetic](http://wiki.ros.org/kinetic/Installation/Ubuntu)
by following the instructions given on referenced web-page.
- If you want to work on Ubuntu 18.04, you will need to install [ROS Melodic](http://wiki.ros.org/melodic/Installation/Ubuntu)
by following the instructions given on referenced web-page.
- We recommend installing the full-desktop version of ROS because it automatically installs the latest compatible version of
Gazebo on your system.
- If you wish to install Gazebo separately, then follow the instruction on the [Gazebo install page](http://gazebosim.org/tutorials?tut=install_ubuntu&cat=install).
- Install Turtlebot-3 package and its dependencies using this [link](https://programmer.help/blogs/ubuntu-18.04-lts-melodic-ros-configuration-turtlebot-3-running-gazebo-simulation.html).
 
## Run Instructions

Clone the repository in your ROS workspace. Type
```
cd ~/<ROS_Workspace>/src
git clone https://github.com/urastogi885/a-star-turtlebot
```
Launch the world. Type
```
cd ~/<ROS_Workspace>
source devel/setup.bash
catkin_make
export TURTLEBOT3_MODEL=burger
roslaunch a-star-turtlebot launcher.launch
```

This will open the world in Gazebo.
