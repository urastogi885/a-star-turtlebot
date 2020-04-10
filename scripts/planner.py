#! /usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
import os


# function to run turtlebot3
def runner():
    pass


if __name__ == '__main__':

    # initialize node
    rospy.init_node('planner')

    # publisher object publishing to command velocity
    pub = rospy.Publisher('/cmd_vel', Twist, queue_size=1)

    # read text file
    with open(os.getcwd() + '/src/a-star-turtlebot/commander.txt', 'r') as command:
        orders = command.readlines()
        print(orders)

    # calling runner
    runner()

    # infinte loop until goal is reached
    rospy.spin()
