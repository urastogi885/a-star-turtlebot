# Import necessary standard libraries
import cv2
import numpy as np
from math import sqrt
from queue import PriorityQueue
# Import custom-built methods
from utils import constants
from utils.node import Node


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


def get_euclidean_distance(first_node, second_node):
    return sqrt((second_node[0] - first_node[0]) ** 2 + (second_node[1] - first_node[1]) ** 2)


def get_base_cost(parent_node, child_node):
    """
    Get base cost of child node
    :param parent_node: a tuple of parent node coordinates and orientation
    :param child_node: a tuple of child node coordinates and orientation
    :return: parent cost + euclidean distance between parent node and child node
    """
    # Self-data contains coordinates of the parent node as a tuple
    return parent_node.base_weight + get_euclidean_distance(parent_node.get_data(), child_node)


class Explorer:
    def __init__(self, start_node, goal_node, robot_rpm, map_img):
        """
        Initialize the explorer with a start node and final goal node
        :param start_node: a list of start coordinates and orientation provided by the user
        :param goal_node: a list of goal coordinates and orientation provided by the user
        :param robot_rpm: a tuple of 2 RPMs for 2 wheels
        :param map_img: a tuple of 2-d arrays with information of the map
        """
        # Store start and goal nodes
        self.start_node = Node(start_node, None, 0, 0, None)
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
        self.closed_nodes = []
        self.path_nodes = []
        # Define 3-D arrays to store information about generated nodes and parent nodes
        self.parent = np.full(fill_value=constants.no_parent, shape=self.map_size)
        # Define grey
        self.grey = [128, 128, 128]
        # Define video-writer of open-cv to record the exploration and final path
        video_format = cv2.VideoWriter_fourcc('M', 'P', '4', 'V')
        self.video_output = cv2.VideoWriter('video_output.mp4', video_format, 600.0,
                                            (self.map_size[1], self.map_size[0]))

    def get_heuristic_score(self, node):
        """
        Implement heuristic function for a-star by calculating euclidean distance
        Heuristic is nothing but cost to goal
        :param: node: tuple containing coordinates and orientation of the current node
        :return: distance between the goal node and current node
        """
        # Evaluate euclidean distance between goal node and current node and return it
        return get_euclidean_distance(self.goal_node, node)

    def get_final_weight(self, node, base_cost):
        """
        Get final weight for a-star
        :param node: tuple containing coordinates and orientation of the current node
        :param base_cost: base cost of the current node
        :return: final weight for according to method
        """
        # Add cost-to-goal and cost-to-come to get final cost and return it
        return self.get_heuristic_score(node) + base_cost

    def get_orientation_index(self, theta):
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
        return int(theta / self.step_theta)

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
        parent_node_data = parent_node.get_data()
        y, x = parent_node_data[0], parent_node_data[1]
        theta = np.pi * parent_node_data[2] * self.step_theta / 180
        # Get action to be performed on parent to generate child
        rpm = self.action_space(action)
        # Convert rpm into left and right wheel velocities respectively
        lw_velocity = rpm[0] * (constants.robot_radius + (0.5 * constants.wheel_distance)) * (2 * np.pi / 60)
        rw_velocity = rpm[1] * (constants.robot_radius - (0.5 * constants.wheel_distance)) * (2 * np.pi / 60)
        return self.show_exploration(parent_node, x, y, theta, lw_velocity, rw_velocity)

    def explore(self):
        """
        Method to explore the map to find the goal
        :return: nothing
        """
        # Initialize priority queue
        node_queue = PriorityQueue()
        start_node = self.start_node.get_data()
        self.parent[int(start_node[0])][int(start_node[1])][int(start_node[2])] = constants.start_parent
        # Add start node to priority queue
        node_queue.put(self.start_node)
        # Start exploring
        while not node_queue.empty():
            # Get node with minimum total cost
            current_node = node_queue.get()
            self.closed_nodes.append(current_node)
            # Add node to generated nodes array
            # Check for goal node
            if (self.get_heuristic_score(current_node.get_data()) <= constants.goal_thresh or
                    current_node.get_data() == self.goal_node):
                self.path_nodes.append(current_node)
                break
            # Generate child nodes from current node
            for i in range(constants.max_actions):
                child_node = self.take_action(current_node, i)
                if child_node is not None:
                    # Add child node to priority queue
                    node_queue.put(child_node)

    def show_exploration(self, parent_node, x, y, theta, l_vel, r_vel):
        """
        Show animation of map exploration and path from start to goal
        :return: nothing
        """
        # An empty list to store intermediate nodes
        inter_nodes = []
        valid_path = True
        parent_node_data = parent_node.get_data()
        prev_x, prev_y, prev_theta = parent_node_data[1], parent_node_data[0], parent_node_data[2]
        t = 0
        while t < constants.total_time:
            # Increment time
            t += constants.time_step
            # Get new  coordinates and orientation using time step
            x += (0.5 * constants.robot_radius * (l_vel + r_vel) * np.cos(theta) * constants.time_step *
                  constants.time_scaling)
            y += (0.5 * constants.robot_radius * (l_vel + r_vel) * np.sin(theta) * constants.time_step *
                  constants.time_scaling)
            theta += ((constants.robot_radius / constants.wheel_distance) * (r_vel - l_vel) *
                      constants.time_step * constants.time_scaling)
            theta_index = self.get_orientation_index(theta)
            if (check_node_validity(self.check_img, int(x), self.map_size[0] - int(y)) and
                    self.parent[int(y)][int(x)][theta_index] == constants.no_parent):
                # Plot curve on map
                inter_nodes.append(([prev_y, prev_x, prev_theta], [y, x, theta]))
                prev_x, prev_y, prev_theta = x, y, theta
            else:
                valid_path = False
                break
        if valid_path:
            last_node = None
            len_inter_nodes = len(inter_nodes)
            for i in range(len_inter_nodes):
                prev_node, current_node = inter_nodes[i]
                # Get orientation of child node in degrees
                prev_node[2] = self.get_orientation_index(prev_node[2])
                current_node[2] = self.get_orientation_index(current_node[2])
                # Update parent of the child node
                self.parent[int(current_node[0])][int(current_node[1])][int(current_node[2])] = \
                    np.ravel_multi_index([int(prev_node[0]), int(prev_node[1]), int(prev_node[2])], dims=self.map_size)
                cv2.arrowedLine(self.map_img, (int(prev_node[1]), self.map_size[0] - int(prev_node[0])),
                                (int(current_node[1]), self.map_size[0] - int(current_node[0])), self.grey)
                self.video_output.write(self.map_img)
                if i == len_inter_nodes - 1:
                    last_node = Node(current_node, parent_node_data, float('inf'), float('inf'), inter_nodes)
                    last_node.set_base_weight(get_base_cost(parent_node, current_node))
                    last_node.set_weight(self.get_final_weight(current_node, last_node.base_weight))
            return last_node
        return None

    def generate_path(self):
        """
        Generate path using back-tracking
        :return: a list containing path nodes
        """
        if not len(self.path_nodes):
            print('No path found')
            return False
        red = [0, 0, 255]
        blue = [255, 0, 0]
        green = [0, 255, 0]
        last_node = self.path_nodes[0]
        # print(np.where(self.parent == constants.start_parent))
        print('Finding path...')
        # Iterate until we reach the initial node
        while last_node.get_data() != self.start_node.get_data():
            for node in self.closed_nodes:
                if node.get_data() == last_node.get_parent():
                    self.path_nodes.append(node)
                    last_node = node
                    break
        print('Path found')
        # Show path
        for i in range(len(self.path_nodes) - 1, 0, -1):
            prev_node_data = self.path_nodes[i - 1].get_data()
            current_node_data = self.path_nodes[i].get_data()
            cv2.line(self.map_img, (int(prev_node_data[1]), self.map_size[0] - int(prev_node_data[0])),
                     (int(current_node_data[1]), self.map_size[0] - int(current_node_data[0])), blue)
            self.video_output.write(self.map_img)
        # Draw start and goal node to the video frame in the form of filled circle
        cv2.circle(self.map_img, (int(self.path_nodes[-1].data[1]), self.map_size[0] - int(self.path_nodes[-1].data[0])),
                   int(constants.robot_radius), red, -1)
        cv2.circle(self.map_img, (int(self.path_nodes[0].data[1]), self.map_size[0] - int(self.path_nodes[0].data[0])),
                   int(constants.robot_radius), green, -1)
        for _ in range(1000):
            self.video_output.write(self.map_img)
        return True
