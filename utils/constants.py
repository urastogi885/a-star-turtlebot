# Define minimum distance threshold in map
dist_thresh = 0.01
scaling_factor = int(1 / dist_thresh)
# Define threshold around goal
goal_thresh = int(scaling_factor * 0.1)
# Define map size
width, height = 10, 10
map_size = (scaling_factor * height), (scaling_factor * width)
# Define robot parameters in cm
robot_diameter = 21
robot_radius = robot_diameter / 2
wheel_distance = 16
# Define all the possible no. of actions
max_actions = 8
half_actions = max_actions // 2
# Robot moves by 1 unit (cm): translation-step
# Orientation of the robot is a multiple of angular-step
angular_step = 30
# Define time between each movement and total time for each action
time_step = 0.1
total_time = 1
# Define total angle of a complete circle
total_angle = 360
# Define exploration constants
no_parent = -1
node_generated = 1
start_parent = -99
