# Import necessary standard libraries
import ast
from sys import argv
from time import time
# Import necessary custom-built classes and methods
from utils.obstacle_space import Map
from utils.constants import scaling_factor, angular_step
from utils.explorer import Explorer, check_node_validity

"""
Add various parameters as input arguments from user
:param start_node_data: a tuple of 3 values: start coordinates and orientation (x, y, theta)
:param goal_node_data: a tuple of 2 values: goal coordinates (x, y)
:param robot_params: a tuple of 3 values: 2 possible values for RPM and a clearance value for the robot
                     The RPMs make the robot differential-drive
"""
script, start_node_data, goal_node_data, robot_params, animation = argv

if __name__ == '__main__':
    # Convert input arguments into tuples
    robot_params = tuple(ast.literal_eval(robot_params))
    start_node_data = tuple(ast.literal_eval(start_node_data))
    goal_node_data = tuple(ast.literal_eval(goal_node_data))
    # Initialize the map class and get map image to check for obstacles
    obstacle_map = Map(robot_params[2])
    check_image = obstacle_map.check_img
    # Convert start and goal nodes given by user into coordinates from map frame
    start_node_data = obstacle_map.get_position_in_map((start_node_data[0], start_node_data[1]), start_node_data[2])
    goal_node_data = obstacle_map.get_position_in_map(goal_node_data)
    # Check validity of start and goal nodes
    if not (check_node_validity(check_image, start_node_data[1], obstacle_map.height - start_node_data[0])
            and check_node_validity(check_image, goal_node_data[1], obstacle_map.height - goal_node_data[0])):
        print('One of the points lie in obstacle space!!\nPlease try again')
        quit()
    # Initialize the explorer class to find the goal node
    # Initialize explorer only after checking start and goal points
    explorer = Explorer(start_node_data, goal_node_data, (robot_params[0], robot_params[1]),
                        (obstacle_map.map_img, check_image), int(animation))
    # Start exploration
    start_time = time()
    explorer.explore()
    print('Exploration time:', time() - start_time)
    start_time = time()
    explorer.generate_path()
    print('Path Generation Time:', time() - start_time)
