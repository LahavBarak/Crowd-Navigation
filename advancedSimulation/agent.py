import pygame
import pgeng
from config import *
import random
from numpy import pi, sin, cos

class Agent:
    def __init__(self, x, y, color, radius):
        self.x_cm = x
        self.y_cm = y
        self.theta = 0
        self.radius = radius
        self.velocity = 0
        self.move_duration = 0

        self.color = color
        self.poly = pgeng.Circle([self.x_cm,self.y_cm],
                                 self.radius,self.color)

    def draw(self, screen):
        self.poly.render(screen)

    def rotate(self, theta_rad):
        self.theta += theta_rad

    def plan(self):
        self.velocity = random.randint(-3,3)
        self.theta = random.uniform(0,2*pi)
        self.move_duration = random.uniform(0,5)
    
    def move(self, collide):
        if self.move_duration <= 0:
            self.plan()
        if collide == True:
            self.velocity = -self.velocity # if in collision - reverse.
        self.x_cm += self.velocity*cos(self.theta)
        self.y_cm += self.velocity*sin(self.theta)
        self.poly = pgeng.Circle([self.x_cm,self.y_cm],
                                 self.radius,self.color)
        self.move_duration -= 1/FPS
    
    def get_velocity(self):
        return self.velocity
    def get_pose(self):
        return [self.x_cm,self.y_cm,self.theta]




