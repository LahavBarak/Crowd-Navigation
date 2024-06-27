import numpy as np
from numpy import pi

# Simulation constants
FPS = 30
WIDTH, HEIGHT = 1200, 900
WALL_THICKNESS = 20

# Robot constants
ROBOT_WIDTH, ROBOT_LENGTH = 40, 80
WHEELBASE_OFFSET = 10
GOAL_TOLERACE = 30 # pixels
MAX_EUCLIDEAN_COST = np.sqrt(WIDTH**2 + HEIGHT**2)
NO_COLLISION_RANGE = np.sqrt(ROBOT_LENGTH**2 +ROBOT_WIDTH**2)//2 + WALL_THICKNESS # minimum distance such that the cm of the robot cannot cause collision with walls

# Agent constants
AGENT_NUM = 5
AGENT_RADIUS = 25
MIN_AGENT_DURATION, MAX_AGENT_DURATION = 3,7
MIN_AGENT_VELOCITY, MAX_AGENT_VELOCITY = 0, 40

# RRT parameters
MAX_ITERATIONS = 500
MAX_RUNTIME = 1.5 # seconds
MIN_VELOCITY, MAX_VELOCITY = 60,100 # pixels/second
MIN_ACCELERATION, MAX_ACCELERATION = -30,30 # pixels/second^2
MAX_DELTA = pi/3                 # radians
MIN_DURATION, MAX_DURATION = 1,2 # seconds
Wc = 0.1 # cost weight coefficient
Wx = 1 # state-space weight coefficient

# Colors
WHITE = (255, 255, 255)
BLACK = (0, 0, 0)
BLUE = (42, 125, 209)
RED = (148, 39, 12)
GREEN = (45, 135, 42)
PURPLE = (107, 50, 168)
BROWN = (74, 46, 1)
ORANGE = (201, 123, 28)
