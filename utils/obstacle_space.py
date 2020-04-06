# Import necessary standard libraries
import cv2
import numpy as np
# Import necessary constants
from utils import constants


class Map:
    def __init__(self):
        """
        Initialize map class with radius of the robot and clearance
        """
        # Various class parameters
        self.height = constants.map_size[0]
        self.width = constants.map_size[1]
        self.thresh = constants.robot_diameter + constants.robot_clearance
        self.scaling = constants.scaling_factor
        self.black = (0, 0, 0)
        # Define length of edge of squares
        # Same for all squares
        self.square_length = int(self.scaling * 1.501)
        # Define the coordinates of the top-left corner of the square obstacles
        self.square_coords = np.array([(225, 125),
                                       (25, self.height - 500 - 75),
                                       (self.width - 100 - 75, self.height - 500 - 75)],
                                      dtype=np.int32)
        # Define radius of circular obstacles
        # Radius is same for all circles
        self.circle_radius = self.scaling * 1
        # Define centers of all circles
        self.circle_centers = np.array([(self.width - 300, 200),
                                        (500, 500),
                                        (300, self.height - 200),
                                        (self.width - 300, self.height - 200)],
                                       dtype=np.int32)
        # Define empty world and add obstacles to it
        self.map_img = np.zeros((self.height, self.width, 3), dtype=np.uint8)
        self.map_img = self.draw_obstacles()
        # Get image to search for obstacles
        self.check_img = self.erode_image()

    def draw_circle(self, img, thresh=0):
        """
        Draw the 4 circular obstacles on the map-image
        :return: nothing
        """
        for center in self.circle_centers:
            # Draw the circle
            cv2.circle(img, (center[0], center[1]), self.circle_radius + thresh, self.black, -1)

    def draw_squares(self, img, thresh=0):
        """
        # Draw the 3 square obstacles on the map
        # :return: nothing
        """
        for corner in self.square_coords:
            top_corner = (corner[0] - thresh), (corner[1] - thresh)
            bottom_corner = (corner[0] + self.square_length + thresh), (corner[1] + self.square_length + thresh)
            # Draw the square on the map
            cv2.rectangle(img, top_corner, bottom_corner, self.black, -1)

    def check_node_validity(self, x, y):
        """
        Method to check whether point lies within any obstacle
        :param x: x-coordinate of the current node
        :param y: y-coordinate of the current node
        :return: false if point lies within any obstacle
        """
        # Check whether the current node lies within the map
        if x >= self.width or y >= self.height:
            return False
        # Check whether the current node lies within any obstacle
        elif self.check_img[y, x].all() == 0:
            return False

        return True

    def erode_image(self):
        """
        Get eroded image to check for obstacles considering the robot radius and clearance
        :return: image with obstacle space expanded to distance threshold between robot and obstacle
        """
        # Get map with obstacles
        eroded_img = self.map_img.copy()
        # Erode map image for rigid robot
        self.draw_squares(eroded_img, self.thresh)
        self.draw_circle(eroded_img, self.thresh)
        # if self.thresh:
            # kernel_size = (self.thresh * 2) + 1
            # erode_kernel = np.ones((kernel_size, kernel_size), np.uint8)
            # eroded_img = cv2.erode(eroded_img, erode_kernel, iterations=1)

        return eroded_img

    def draw_obstacles(self):
        """
        Draw map using half-plane equations
        :return: map-image with all obstacles
        """
        # Fill map-image with white color
        self.map_img.fill(255)
        # Draw various obstacles on the map
        self.draw_circle(self.map_img)
        self.draw_squares(self.map_img)

        return self.map_img
