import pygame
import pgeng
import numpy as np
from config import *
# backup of simple robot model, before implementing ackermann kinematics
# created 8/3/24

class Robot:
    def __init__(self, x, y, width, length, angle = 0):
        self.x_cm = x 
        self.y_cm = y
        self.width = width
        self.length = length
        self.angle = angle
        self.vertices = [(self.x_cm-(self.width/2),self.y_cm-(self.length/2)),
                         (self.x_cm-(self.width/2),self.y_cm+(self.length/2)),
                         (self.x_cm+(self.width/2),self.y_cm+(self.length/2)),
                         (self.x_cm+(self.width/2),self.y_cm-(self.length/2))]
        self.poly = pgeng.Polygon(self.vertices,BLUE)
        

    def draw(self, screen):
        self.update_vertices()
        self.poly.set_points(self.vertices)
        self.poly.render(screen)

    def rotate(self, angle_rad):
        self.angle += angle_rad

    def move(self, dx, dy):
        R = np.array([[np.cos(self.angle),np.sin(self.angle)],
                     [-np.sin(self.angle),np.cos(self.angle)]])
        displacement = np.array([[dx], [dy]])
        dx_rotated, dy_rotated = np.dot(R, displacement).flatten()
        self.x_cm += dx_rotated
        self.y_cm += dy_rotated

    def update_vertices(self):
        '''
        reset vertices to parallel position, displaced to new center,
        then rotate by angle radians to reflect the angle of the robot
        '''
        self.vertices = [(self.x_cm-(self.width/2),self.y_cm-(self.length/2)),
                         (self.x_cm-(self.width/2),self.y_cm+(self.length/2)),
                         (self.x_cm+(self.width/2),self.y_cm+(self.length/2)),
                         (self.x_cm+(self.width/2),self.y_cm-(self.length/2))]
        R = np.array([[np.cos(self.angle),-np.sin(self.angle)],
                     [np.sin(self.angle),np.cos(self.angle)]])
        cm = np.array([self.x_cm,self.y_cm])
        rotated_vertices = []
        for vertex in self.vertices:
            np_vertex = np.array(vertex)
            rotated_vertex = np.dot(np_vertex-cm,R)
            rotated_vertex += cm
            rotated_vertices.append((rotated_vertex[0],
                                    rotated_vertex[1]))
        self.vertices = rotated_vertices

    