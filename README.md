# A-Star Algorithm on Turtlebot

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

## Dependencies

- Python3
- Python3-tk
- Python3 Libraries: Numpy, OpenCV-Python, Math, Queue

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

## Run Instructions

- Using the terminal, clone this repository and go into the project directory, and run the main program:

```
git clone https://github.com/urastogi885/a-star-turtlebot
cd a-star-robot
```

- If you have a compressed version of the project, extract it, go into project directory, open the terminal, and type:

```
python3 a_star_turtlebot.py start_x,start_y,start_orientation goal_x,goal_y rpm1,rpm2,clearance
python3 a_star_turtlebot.py -4.75,-4.75,0 4.75,4.75 30,25,5
```

- Note the following to try the code with other inputs:
    - The origin of the map is taken at the center of the map s make sure to take into consideration in which 
    quadrant your points lie.
    - Provide start and goal coordinates in meters while start orientation is to be provided in degrees.
    - In addition to that, clearance is taken in centimeters.
    - Refer documentation of [*a_star_turtlebot.py*](https://github.com/urastogi885/a-star-robot/blob/master/a_star_robot.py) to
    understand further about input arguments.
