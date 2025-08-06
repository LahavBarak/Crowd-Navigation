import pgeng
from config import *
import random
from numpy import pi, sin, cos
from motion_models import linear_cart
from utils import minimal_distance,get_vertices
from robot import Robot
import math
import csv
import time

class Agent:
    def __init__(self, name, x, y, color, radius, map):
        self.name = name
        self.x = x
        self.y = y
        self.radius = radius
        self.v_x = 0
        self.v_y = 0
        self.move_duration = 0

        self.robot_x, self.robot_y, self.robot_theta = 0,0,0
        self.robot_delta, self.robot_v, self.robot_t = 0,0,0
        self.agents_poses = []
        self.agents_v = []
        self.map = map

        self.color = color
        self.poly = pgeng.Circle([self.x,self.y],
                                 self.radius,self.color)
        self.t0 = time.time()
        
        ## ---- DEBUG csv ---- ##
        agent_file = f"logs/agents/agent_{self.name}.csv"
        agent_fh = open(agent_file, 'w', newline='')
        self.agent_writer = csv.writer(agent_fh)
        self.agent_writer.writerow(["x",'y','v_x','v_y','duration', 'sf_x', 'sf_y'])

    def draw(self, screen):
        self.poly.render(screen)

    def rotate(self, theta_rad):
        self.theta += theta_rad

    def plan(self):
        self.v_x = random.randint(MIN_AGENT_VELOCITY,MAX_AGENT_VELOCITY)
        self.v_y = random.randint(MIN_AGENT_VELOCITY,MAX_AGENT_VELOCITY)
        self.move_duration = random.uniform(MIN_AGENT_DURATION,MAX_AGENT_DURATION)
        
    
    def move(self):
        
        if self.move_duration <= 0:
            self.plan()

        sf_x,sf_y = self.apply_social_forces()
        collision = self.collision_check()
        if collision == "vert":
            self.v_x = -self.v_x 
        elif collision == "horz":
            self.v_y = -self.v_y 

        self.x, self.y = linear_cart(self.x, self.y, self.v_x, self.v_y)
        self.poly = pgeng.Circle([self.x,self.y],
                                 self.radius,self.color)
        self.move_duration -= 1/FPS

        ## ---- DEBUG csv ---- ##
        self.agent_writer.writerow([self.x, self.y, self.v_x, self.v_y, self.move_duration, sf_x, sf_y])
        ## ----------------------------------- ##
    
    def apply_social_forces(self):
        # compute force vector
        f_x, f_y = 0.0, 0.0 # social force total
        
        # robot <-> agent center-to-center distance
        dx_robot = (self.x - self.robot_x) / PIXELS_PER_METER
        dy_robot = (self.y - self.robot_y) / PIXELS_PER_METER
        dist_robot = math.hypot(dx_robot,dy_robot)

        # robot <-> agent edge-to-edge distance
        rect_verts = get_vertices(self.robot_x, self.robot_y,
                                  self.robot_theta,
                                  ROBOT_LENGTH, ROBOT_WIDTH)
        clearance_px = minimal_distance((self.x, self.y),
                                        self.radius,
                                        rect_verts)
        dist_robot_edge = clearance_px / PIXELS_PER_METER

        if dist_robot > EPSILON and dist_robot_edge > EPSILON: # zero-guard
            # unit vector (agent <-robot)
            u_x = dx_robot / (dist_robot + EPSILON)
            u_y = dy_robot / (dist_robot + EPSILON)

            # compute force ~1/r^2
            f_x += u_x / (dist_robot_edge ** 2 + EPSILON)
            f_y += u_y / (dist_robot_edge ** 2 + EPSILON) 
        
        # agent <-> agent edge-to-edge distance
        for (agent_x, agent_y) in self.agents_poses:
            dx_agent = self.x - agent_x
            dy_agent = self.y - agent_y
            dist_agent = math.hypot(dx_agent, dy_agent)
            dist_agent_edge = (dist_agent - 2*self.radius) / PIXELS_PER_METER
            if dist_agent > EPSILON: # zero-guard
                u_x = (dx_agent/PIXELS_PER_METER) / (dist_agent + EPSILON)
                u_y = (dy_agent/PIXELS_PER_METER) / (dist_agent + EPSILON)
                f_x += u_x / (dist_agent_edge ** 2 + EPSILON)
                f_y += u_y / (dist_agent_edge ** 2 + EPSILON)

        # apply force vector to motion
        self.v_x += SF * f_x * (1/FPS)
        self.v_y -= SF * f_y * (1/FPS)

        self.v_x = max(MIN_AGENT_VELOCITY, min(self.v_x,MAX_AGENT_VELOCITY))
        self.v_y = max(MIN_AGENT_VELOCITY, min(self.v_y,MAX_AGENT_VELOCITY))

        return f_x,f_y ## DEBUG for data collection
        
    def collision_check(self):
        # 1) predict next position
        next_x, next_y = linear_cart(self.x, self.y, self.v_x, self.v_y)

        for wall in self.map:
            # decide if wall is horizontal or vertical
            if wall.width > wall.height:
                # ── horizontal wall ──
                # do we overlap in X (with a radius buffer)?
                if next_x > wall.left - self.radius and next_x < wall.right + self.radius:
                    # is Y within radius of the wall’s Y‐center?
                    wall_cy = wall.y + wall.height/2
                    if abs(next_y - wall_cy) < (self.radius + wall.height/2):
                        return "horz"
            else:
                # ││ vertical wall ││
                if next_y > wall.top - self.radius and next_y < wall.bottom + self.radius:
                    wall_cx = wall.x + wall.width/2
                    if abs(next_x - wall_cx) < (self.radius + wall.width/2):
                        return "vert"

        return 0
    
    def set_environment_data(self,agents, robot: Robot):
        agents_pose = []
        agents_vel = []
        for agent in agents:
            if agent.name == self.name:
                pass
            agents_pose.append(agent.get_pose())
            agents_vel.append(agent.get_velocity())
        self.agents_poses = agents_pose
        self.agents_vel = agents_vel
        self.robot_x = robot.x
        self.robot_y = robot.y
        self.robot_theta = robot.theta
        self.robot_delta = robot.delta
        self.robot_t = robot.duration
    
    def get_velocity(self):
        return [self.v_x, self.v_y]
    
    def get_pose(self):
        return [self.x,self.y]




