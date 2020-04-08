# Import necessary standard libraries
import ast
from sys import argv
from time import time
# Import necessary custom-built classes and methods
from utils.obstacle_space import Map
from utils.constants import scaling_factor, step_params
from utils.explorer import Explorer, check_node_validity

"""
Add various parameters as input arguments from user
:param start_node_data: a tuple of 3 values: start coordinates and orientation (x, y, theta)
:param goal_node_data: a tuple of 2 values: goal coordinates (x, y)
:param robot_params: a tuple of 2 values: RPM of left and right wheel respectively
                     The RPMs make the robot differential-drive
:param animation: 1 to generate video otherwise use 0
"""
script, start_node_data, goal_node_data, robot_params, animation = argv

if __name__ == '__main__':
    # Convert input arguments into their required data types and scale them according to the size of the map
    robot_params = tuple(ast.literal_eval(robot_params))
    start_node_data = tuple(ast.literal_eval(start_node_data))
    start_node_data = (int(scaling_factor * start_node_data[1]), int(scaling_factor * start_node_data[0]),
                       start_node_data[2] // step_params[1])
    goal_node_data = tuple(ast.literal_eval(goal_node_data))
    goal_node_data = (int(scaling_factor * goal_node_data[1]), int(scaling_factor * goal_node_data[0]),
                      goal_node_data[2] // step_params[1])
    # Initialize the map class
    obstacle_map = Map()
    check_image = obstacle_map.check_img
    # Check validity of start and goal nodes
    if not (check_node_validity(check_image, start_node_data[1], obstacle_map.height - start_node_data[0])
            and check_node_validity(check_image, goal_node_data[1], obstacle_map.height - goal_node_data[0])):
        print('One of the points lie in obstacle space!!\nPlease try again')
        quit()
    # Initialize the explorer class to find the goal node
    # Initialize explorer only after checking start and goal points
    explorer = Explorer(start_node_data, goal_node_data, robot_params)
    # Get start time for exploration
    start_time = time()
    # Start exploration
    explorer.explore(check_image)
    # Show time for exploration
    print('Exploration Time:', time() - start_time)
    if int(animation):
        # Get start time for animation
        start_time = time()
        # Display animation of map exploration to find goal
        explorer.show_exploration(obstacle_map.map_img)
        # Show time for animation
        print('Animation Time:', time() - start_time)
