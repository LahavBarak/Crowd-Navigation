import numpy as np

# Constants
WIDTH, HEIGHT = 800, 600
WALL_THICKNESS = 20
ROBOT_WIDTH, ROBOT_LENGTH = 40, 80
NO_COLLISION_RANGE = np.sqrt(ROBOT_LENGTH**2 +ROBOT_WIDTH**2)//2 + WALL_THICKNESS # minimum distance such that the cm of the robot cannot cause collision with walls
ARROW_HEIGHT = 100
AGENT_NUM = 2
AGENT_RADIUS = 15
ROTATION_ANGLE = np.deg2rad(5)
VELOCITY_UNIT = 1
TRANSLATION_UNIT = 10
OFFSET = 10
FPS = 30
GOAL_TOLERACE = 20

# Colors
WHITE = (255, 255, 255)
BLACK = (0, 0, 0)
BLUE = (42, 125, 209)
RED = (148, 39, 12)
GREEN = (45, 135, 42)