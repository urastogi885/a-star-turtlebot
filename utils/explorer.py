# Import necessary standard libraries
import cv2
import numpy as np
from math import sqrt
from queue import PriorityQueue
# Import custom-built methods
from utils import constants


def check_node_validity(check_img, x, y):
    """
    Method to check whether point lies within any obstacle
    :param check_img: 2-d array with information of the map
    :param x: x-coordinate of the current node
    :param y: y-coordinate of the current node
    :return: false if point lies within any obstacle
    """
    # Check condition for out of bounds
    if x <= 0 or x >= constants.map_size[1] or y <= 0 or y >= constants.map_size[0]:
        return False
    # Check condition to verify if point lies within obstacle
    elif check_img[y, x].all() == 0:
        return False

    return True


class Explorer:
    def __init__(self, start_node, goal_node, robot_rpm, map_img):
        """
        Initialize the explorer with a start node and final goal node
        :param start_node: a tuple of start coordinates and orientation provided by the user
        :param goal_node: a tuple of goal coordinates and orientation provided by the user
        :param robot_rpm: a tuple of 2 RPMs for 2 wheels
        :param map_img: a tuple of 2-d arrays with information of the map
        """
        # Store start and goal nodes
        self.start_node = start_node
        self.goal_node = goal_node
        # Store RPMs of robot
        self.robot_rpm = robot_rpm
        self.map_img = map_img[0]
        self.check_img = map_img[1]
        # Store angular step size and translation step size
        self.step_theta = constants.angular_step
        # Store map dimensions
        self.map_size = constants.map_size[0], constants.map_size[1], (constants.total_angle // self.step_theta)
        # Define an empty list to store all generated nodes and path nodes
        # self.generated_nodes = []
        self.path_nodes = []
        # Define 3-D arrays to store information about generated nodes and parent nodes
        self.parent = np.full(fill_value=constants.no_parent, shape=self.map_size)
        # Define a 3-D array to store base cost of each node
        self.base_cost = np.full(fill_value=constants.no_parent, shape=self.map_size)
        # Define grey
        self.grey = [128, 128, 128]
        # Define video-writer of open-cv to record the exploration and final path
        video_format = cv2.VideoWriter_fourcc('M', 'P', '4', 'V')
        self.video_output = cv2.VideoWriter('video_output_1.mp4', video_format, 600.0,
                                            (self.map_size[1], self.map_size[0]))

    def get_heuristic_score(self, node):
        """
        Implement heuristic function for a-star by calculating euclidean distance
        Heuristic is nothing but cost to goal
        :param: node: tuple containing coordinates and orientation of the current node
        :return: distance between the goal node and current node
        """
        # Evaluate euclidean distance between goal node and current node and return it
        return sqrt((self.goal_node[0] - node[0])**2 + (self.goal_node[1] - node[1])**2)

    def get_final_weight(self, node):
        """
        Get final weight for a-star
        :param node: tuple containing coordinates and orientation of the current node
        :return: final weight for according to method
        """
        # Add cost-to-goal and cost-to-come to get final cost and return it
        return self.get_heuristic_score(node) + self.base_cost[node[0]][node[1]][node[2]]

    def get_child_base_cost(self, parent_node, child_node):
        """
        Get base cost of child node
        :param parent_node: a tuple of parent node coordinates and orientation
        :param child_node: a tuple of child node coordinates and orientation
        :return: parent cost + euclidean distance between parent node and child node
        """
        # Self-data contains coordinates of the parent node as a tuple
        return (self.base_cost[parent_node[0]][parent_node[1]][parent_node[2]] +
                sqrt((child_node[0] - parent_node[0]) ** 2 + (child_node[1] - parent_node[1]) ** 2))

    def action_space(self, action):
        """
        Define action space
        :param action: Varies from 0-7 to call one of the 8 defined actions
        :return: a tuple of left and right wheel RPM
        """
        if action == 0:
            return 0, self.robot_rpm[0]
        elif action == 1:
            return 0, self.robot_rpm[1]
        elif action == 2:
            return self.robot_rpm[0], 0
        elif action == 3:
            return self.robot_rpm[1], 0
        elif action == 4:
            return self.robot_rpm[0], self.robot_rpm[0]
        elif action == 5:
            return self.robot_rpm[1], self.robot_rpm[1]
        elif action == 6:
            return self.robot_rpm[0], self.robot_rpm[1]

        return self.robot_rpm[1], self.robot_rpm[0]

    def take_action(self, parent_node, action):
        """
        Call various actions based on an integer and get new child coordinates and orientation
        Applying differential drive formulae to get coordinates and orientation
        :param parent_node: a tuple of parent node coordinates and orientation
        :param action: Varies from 0-n to call one of the 8 defined actions
        :return: new coordinates and orientation of the node after the desired action
        """
        # Get parent coordinates and orientation in radians
        y, x = parent_node[0], parent_node[1]
        theta = np.pi * parent_node[2] * self.step_theta / 180
        # Get action to be performed on parent to generate child
        rpm = self.action_space(action)
        # Convert rpm into left and right wheel velocities respectively
        lw_velocity = rpm[0] * (constants.robot_radius + (0.5 * constants.wheel_distance)) * (2 * np.pi / 60)
        rw_velocity = rpm[1] * (constants.robot_radius - (0.5 * constants.wheel_distance)) * (2 * np.pi / 60)
        child_node = self.show_exploration(parent_node, x, y, theta, lw_velocity, rw_velocity)
        if child_node is not None:
            x, y, theta = child_node
            # Get orientation of child node in degrees
            if theta >= 2 * np.pi:
                n = int(theta / (2 * np.pi))
                theta = theta - n * 2 * np.pi
            elif theta < 0:
                theta = abs(theta)
                if theta > 2 * np.pi:
                    n = int(theta / (2 * np.pi))
                    theta = theta - n * 2 * np.pi
            theta = theta + (180 * theta / np.pi)
            if theta > constants.total_angle:
                theta = theta - constants.total_angle
            # Return the coordinates and orientation of the child node
            return y, x, int(theta / self.step_theta)
        return None

    def explore(self):
        """
        Method to explore the map to find the goal
        :return: nothing
        """
        # Initialize priority queue
        node_queue = PriorityQueue()
        # Add cost-to-come of start node in the array
        # Start node has a cost-to-come of 0
        self.base_cost[self.start_node[0]][self.start_node[1]][self.start_node[2]] = 0
        # self.generated_nodes.append(self.start_node)
        self.parent[self.start_node[0]][self.start_node[1]][self.start_node[2]] = constants.start_parent
        # Add start node to priority queue
        node_queue.put((self.get_final_weight(self.start_node), self.start_node))
        # Start exploring
        while not node_queue.empty():
            # Get node with minimum total cost
            current_node = node_queue.get()
            # Add node to generated nodes array
            # Check for goal node
            if self.get_heuristic_score(current_node[1]) <= constants.goal_thresh or current_node[1] == self.goal_node:
                self.path_nodes.append(current_node[1])
                break
            # Generate child nodes from current node
            for i in range(constants.max_actions):
                child_node = self.take_action(current_node[1], i)
                if child_node is not None:
                    # Get coordinates of child node
                    y, x, theta = child_node
                    # Check whether child node is not within obstacle space and has not been previously generated
                    if (check_node_validity(self.check_img, x, self.map_size[0] - y) and
                            self.parent[y][x][theta] == constants.no_parent):
                        # Update cost-to-come of child node
                        self.base_cost[y][x][theta] = self.get_child_base_cost(current_node[1], (y, x, theta))
                        # Add child node to priority queue
                        node_queue.put((self.get_final_weight((y, x, theta)), (y, x, theta)))
                        # Update parent of the child node
                        self.parent[y][x][theta] = np.ravel_multi_index([current_node[1][0], current_node[1][1],
                                                                         current_node[1][2]], dims=self.map_size)

    def generate_path(self):
        """
        Generate path using back-tracking
        :return: a list containing path nodes
        """
        if not len(self.path_nodes):
            print('No path found')
            return False
        last_node = self.path_nodes[0]
        # Iterate until we reach the initial node
        while self.parent[last_node[0]][last_node[1]][last_node[2]] != constants.start_parent:
            # Search for parent node in the list of closed nodes
            last_node = np.unravel_index(self.parent[last_node[0]][last_node[1]][last_node[2]], dims=self.map_size)
            self.path_nodes.append(last_node)

        return True

    def show_exploration(self, parent_node, x, y, theta, l_vel, r_vel):
        """
        Show animation of map exploration and path from start to goal
        :return: nothing
        """
        # An empty list to store intermediate nodes
        inter_nodes = []
        valid_path = True
        prev_x, prev_y = parent_node[1], parent_node[0]
        t = 0
        while t < constants.total_time:
            t += constants.time_step
            # Get new  coordinates and orientation using time step
            x += (0.5 * constants.robot_radius * (l_vel + r_vel) * np.cos(theta) * constants.time_step)
            y += (0.5 * constants.robot_radius * (l_vel + r_vel) * np.sin(theta) * constants.time_step)
            theta += ((constants.robot_radius / constants.wheel_distance) * (r_vel - l_vel) *
                      constants.time_step)
            if check_node_validity(self.check_img, int(x), int(y)):
                # Plot curve on map
                inter_nodes.append(((int(prev_y), int(prev_x)), (int(y), int(x))))
                prev_x, prev_y = x, y
            else:
                valid_path = False
                break
        if valid_path:
            for prev_node, current_node in inter_nodes:
                cv2.arrowedLine(self.map_img, (prev_node[1], self.map_size[0] - prev_node[0]),
                                (current_node[1], self.map_size[0] - current_node[0]), self.grey)
                self.video_output.write(self.map_img)
            return int(x), int(y), theta
        return None
