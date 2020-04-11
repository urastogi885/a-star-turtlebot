#! /usr/bin/env python

import rospy
from geometry_msgs.msg import Twist
import rospkg


# function to convert file into array
def converter(file):
    for i in range(len(file)):
        file[i] = file[i].split(',')

    return file


# function to run turtlebot3
def runner(order):

    # declaring object for class Twist
    msg = Twist()

    for i in range(len(order)):
        msg.linear.x = float(order[i][0])
        msg.angular.z = float(order[i][1])
        print(msg)
        pub.publish(msg)


if __name__ == '__main__':

    global pub

    # publisher object publishing to command velocity
    pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)

    # initialize node
    rospy.init_node('planner', anonymous=True)

    # read text file
    ros_root = rospkg.RosPack()
    with open(ros_root.get_path('a-star-turtlebot') + '/commander.txt', 'r') as command:
        orders = command.readlines()

    rate = rospy.Rate(1)
    # convert into array
    orders = converter(orders)
    while True:
        # calling runner
        runner(orders)
        rate.sleep()

    # runnning infinte times
    rospy.spin()