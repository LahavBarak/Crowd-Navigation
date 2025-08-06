import numpy as np
from numpy import pi

# Simulation constants
FPS = 30
WIDTH, HEIGHT = 1200, 900
WALL_THICKNESS = 20
GAME_TIME = 3600.0 # seconds
ROUND_TIME = 180.0 # seconds
GOAL_CLEARANCE = 40 # how many pixels must the random goal be away from any wall
PIXELS_PER_METER = 10
MAP = "midwall"
EPSILON = 1e-6 # division-by-zero guard

# Robot constants
ROBOT_WIDTH, ROBOT_LENGTH = 40, 80
WHEELBASE_OFFSET = 10
GOAL_TOLERACE = 30 # pixels, how close to the goal must the robot be for it to count as having reached it
MAX_EUCLIDEAN_COST = np.sqrt(WIDTH**2 + HEIGHT**2)
NO_COLLISION_RANGE = np.sqrt(ROBOT_LENGTH**2 +ROBOT_WIDTH**2)//2 + WALL_THICKNESS # minimum distance such that the cm of the robot cannot cause collision with walls

# Agent constants
AGENT_NUM = 5
AGENT_RADIUS = 25
MIN_AGENT_DURATION, MAX_AGENT_DURATION = 3,7
MIN_AGENT_VELOCITY, MAX_AGENT_VELOCITY = -40, 40
SF = 500 # social force impact coefficient

# PRM parameters
PRM_RADIUS = 15 # radius of R-NN search
SAMPLE_FACTOR = 40 # modify the sample number. hyperparameter, empyrically tuned.
PRM_SAMPLES = int(WIDTH*HEIGHT/SAMPLE_FACTOR) # number of samples in the PRM graph. based on map dimensions and reduced by factor.
AVG_DISTANCE = 9 # average distance between 2 nodes in path, tested empyrically. see notes for more info.
LOOKAHEAD_INDEX = 20 # in waypoints, i.e. look X waypoints ahead.
MAP_FILEPATH = f"PRM_graphs/{MAP}_100000_samples" # path to pre-existing PRM map. place None if unsure which wall map the PRM map was generated for, as it does not check wall-map fitting

# RRT parameters
MAX_ITERATIONS = 500
MAX_RUNTIME = 1.5
MIN_VELOCITY, MAX_VELOCITY = 10,100 # pixels/second
MIN_ACCELERATION, MAX_ACCELERATION = -30,30 # pixels/second^2
MAX_DELTA = pi/3                 # radians
MIN_DURATION, MAX_DURATION = 1,2 # seconds
Wc = 0.1 # cost weight coefficient
Wx = 1 # state-space weight coefficient
SEARCH_RADIUS_COEFFICIENT = 1.5 # Expand the RRT search bubble by this multiplying this.
SEARCH_RADIUS = SEARCH_RADIUS_COEFFICIENT*AVG_DISTANCE*LOOKAHEAD_INDEX # limit the RRT state-space to a narrower field.
COLLISION_CHECK_RESOLUTION = (0.5*ROBOT_LENGTH)/MAX_VELOCITY # number of frames between each step of collision check

# Cost Function Coefficients
D = 1 # distance coefficient
G = 10 # distance-to-goal coefficient
P = 15 # proximity to agents coefficient
C = 70 # curvature coefficient
GR = 1000 # goal reached reward
# Colors
WHITE = (255, 255, 255)
BLACK = (0, 0, 0)
BLUE = (42, 125, 209)
RED = (148, 39, 12)
GREEN = (45, 135, 42)
PURPLE = (107, 50, 168)
BROWN = (74, 46, 1)
ORANGE = (201, 123, 28)
