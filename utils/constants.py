# Define minimum distance threshold in map
dist_thresh = 0.01
scaling_factor = int(1 / dist_thresh)
# Define map size
width, height = 10, 10
map_size = (scaling_factor * height), (scaling_factor * width)
map_center = (map_size[0] // 2), (map_size[1] // 2)
# Define robot parameters in cm
robot_diameter = 21
robot_radius = robot_diameter / 2
wheel_distance = 16
wheel_radius = 3.3
# Define threshold around goal
goal_thresh = robot_radius
# Define the maximum no. of possible actions
max_actions = 8
# Robot moves by 1 unit (cm): translation-step
# Orientation of the robot is a multiple of angular-step
angular_step = 20
# Define time between each movement and total time for each action
time_step = 1
total_time = 10
time_scaling = 0.5
# Define total angle of a complete circle
total_angle = 360
# Define exploration constants
no_parent = -1
node_generated = 1
start_parent = -99
