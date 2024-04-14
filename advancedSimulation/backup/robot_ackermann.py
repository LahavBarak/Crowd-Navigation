import pygame
import pgeng
import numpy as np
from numpy import pi, sin, cos, tan
from config import *

class Robot:
    def __init__(self, x, y, width, length, theta = -pi/2, delta = 0, velocity = 0):
        self.x_cm = x 
        self.y_cm = y
        self.width = width
        self.length = length
        self.wheelbase = self.length - WHEELBASE_OFFSET
        self.theta = theta # angle between the main axis of the robot and y-axis
        self.delta = delta # angle between the wheels and main axis
        self.velocity = velocity
        self.vertices = [(self.x_cm-(self.length/2),self.y_cm-(self.width/2)),
                         (self.x_cm-(self.length/2),self.y_cm+(self.width/2)),
                         (self.x_cm+(self.length/2),self.y_cm+(self.width/2)),
                         (self.x_cm+(self.length/2),self.y_cm-(self.width/2))]
        self.poly = pgeng.Polygon(self.vertices,BLUE)
        

    def draw(self, screen):
        self.update_vertices()
        self.poly.set_points(self.vertices)
        self.poly.render(screen)

    def rotate(self, delta_rad):
        self.delta = max(-1,(min(1,self.delta + delta_rad)))
        print(self.delta)
    
    def change_velocity(self,velocity):
        self.velocity += velocity

    def move(self):
        self.x_cm  += self.velocity*cos(self.theta)
        self.y_cm  += self.velocity*sin(self.theta)
        self.theta += self.velocity*tan(self.delta)/self.wheelbase
        

    def update_vertices(self):
        '''
        reset vertices to parallel position, displaced to new center,
        then rotate by delta radians to reflect the delta of the robot
        '''
        self.vertices = [(self.x_cm-(self.length/2),self.y_cm-(self.width/2)),
                         (self.x_cm-(self.length/2),self.y_cm+(self.width/2)),
                         (self.x_cm+(self.length/2),self.y_cm+(self.width/2)),
                         (self.x_cm+(self.length/2),self.y_cm-(self.width/2))]
        R = np.array([[np.cos(self.theta),-np.sin(self.theta)],
                     [np.sin(self.theta),np.cos(self.theta)]])
    
        if self.delta == 0: ICR = 0
        else: ICR = self.wheelbase/tan(self.delta)
        rotation_cm = np.array([self.x_cm + ICR, self.y_cm])
        rotated_vertices = []
        for vertex in self.vertices:
            np_vertex = np.array(vertex)
            rotated_vertex = np.dot(np_vertex-rotation_cm,R)
            rotated_vertex += rotation_cm
            rotated_vertices.append((rotated_vertex[0],
                                    rotated_vertex[1]))
        self.vertices = rotated_vertices

    