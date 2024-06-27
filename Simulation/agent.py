import pgeng
from config import *
import random
from numpy import pi, sin, cos
from motion_models import linear
import csv
import time

class Agent:
    def __init__(self, x, y, color, radius):
        self.x = x
        self.y = y
        self.theta = 0
        self.radius = radius
        self.velocity = 0
        self.move_duration = 0

        self.color = color
        self.poly = pgeng.Circle([self.x,self.y],
                                 self.radius,self.color)
        self.t0 = time.time()
        ## ---- DEBUG print ---- ##
        csv_name_agents = f"logs/agents/agents.csv"
        csv_header_agents = ["agent","x","y", "v","time"]
        with open(csv_name_agents, 'w', newline='') as csvfile:
            csv_writer = csv.writer(csvfile)
            csv_writer.writerow(csv_header_agents)
        ## ----------------------------------- ##

    def draw(self, screen):
        self.poly.render(screen)

    def rotate(self, theta_rad):
        self.theta += theta_rad

    def plan(self):
        self.velocity = random.randint(MIN_AGENT_VELOCITY,MAX_AGENT_VELOCITY)
        self.theta = random.uniform(0,2*pi)
        self.move_duration = random.uniform(MIN_AGENT_DURATION,MAX_AGENT_DURATION)
    
    def move(self, collide):
        if self.move_duration <= 0:
            self.plan()
        if collide == True:
            self.velocity = -self.velocity # if in collision - reverse.
        self.x, self.y = linear(self.x, self.y, self.theta, self.velocity)
        self.poly = pgeng.Circle([self.x,self.y],
                                 self.radius,self.color)
        self.move_duration -= 1/FPS

        ## ---- DEBUG print ---- ##
        csv_name_agents = f"logs/agents/agents.csv"
        csv_data_agents = [self.color,self.x,self.y, self.velocity,time.time()-self.t0]
        with open(csv_name_agents, 'a', newline='') as csvfile:
            csv_writer = csv.writer(csvfile)
            csv_writer.writerow(csv_data_agents)
        ## ----------------------------------- ##
    
    def get_velocity(self):
        return self.velocity
    def get_pose(self):
        return [self.x,self.y,self.theta]




