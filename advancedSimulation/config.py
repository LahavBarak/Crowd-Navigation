import numpy as np

# Simulation constants
FPS = 30
WIDTH, HEIGHT = 800, 600
WALL_THICKNESS = 20

# Robot constants
ROBOT_WIDTH, ROBOT_LENGTH = 40, 80
WHEELBASE_OFFSET = 10
GOAL_TOLERACE = 20
MAX_EUCLIDEAN_COST = np.sqrt(WIDTH**2 + HEIGHT**2)
NO_COLLISION_RANGE = np.sqrt(ROBOT_LENGTH**2 +ROBOT_WIDTH**2)//2 + WALL_THICKNESS # minimum distance such that the cm of the robot cannot cause collision with walls

# Agent constants
AGENT_NUM = 2
AGENT_RADIUS = 15


# Colors
WHITE = (255, 255, 255)
BLACK = (0, 0, 0)
BLUE = (42, 125, 209)
RED = (148, 39, 12)
GREEN = (45, 135, 42)