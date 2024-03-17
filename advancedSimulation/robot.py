import random
import pgeng
import numpy as np
from numpy import pi, sin, cos
from config import *
from RRT import *
import csv
import time

# backup of simple robot model, before implementing ackermann kinematics
# created 8/3/24

class Robot:
    def __init__(self, x, y, width, length, theta = pi/2, goal = [200,300]):
        self.x_cm = x 
        self.y_cm = y
        self.width = width
        self.length = length
        self.theta = theta

        self.goal = goal
        self.velocity = 0
        self.move_duration = 0

        self.agents_pose = []
        self.agents_vel = []
        self.walls = []

        self.vertices = [(self.x_cm-(self.length/2),self.y_cm-(self.width/2)),
                         (self.x_cm-(self.length/2),self.y_cm+(self.width/2)),
                         (self.x_cm+(self.length/2),self.y_cm+(self.width/2)),
                         (self.x_cm+(self.length/2),self.y_cm-(self.width/2))]
        self.poly = pgeng.Polygon(self.vertices,BLUE)

        self.i = 0 ##debug variable, to count csv files
        

    def draw(self, screen):
        self.vertices = self.get_vertices(self.x_cm,self.y_cm,self.theta)
        self.poly.set_points(self.vertices)
        self.poly.render(screen)

    def move(self):
        if self.move_duration <= 0:
            plan = self.plan()
            self.i += 1
            time.sleep(2)
            self.velocity = plan.u[0]
            self.theta = plan.u[1]
            self.move_duration = plan.t

        self.change_cm(self.velocity*cos(self.theta), 
                       self.velocity*sin(self.theta))
        self.move_duration -= 1

    def rotate(self, theta_rad):
        self.theta += theta_rad

    def change_cm(self, dx, dy):
        self.x_cm += dx
        self.y_cm -= dy  # due to y axis being pointed downwards

    def get_vertices(self,x_cm,y_cm, theta):
        '''
        reset vertices to parallel position, displaced to new center,
        then rotate by theta radians to reflect the theta of the robot
        '''
        vertices = ([x_cm-(self.length/2),y_cm-(self.width/2)],
                    [x_cm-(self.length/2),y_cm+(self.width/2)],
                    [x_cm+(self.length/2),y_cm+(self.width/2)],
                    [x_cm+(self.length/2),y_cm-(self.width/2)])
        R = np.array([[cos(theta),-sin(theta)],
                     [sin(theta),cos(theta)]])
        cm = np.array([x_cm,y_cm])
        rotated_vertices = []
        for vertex in vertices:
            np_vertex = np.array(vertex)
            rotated_vertex = np.dot(np_vertex-cm,R)
            rotated_vertex += cm
            rotated_vertices.append((rotated_vertex[0],
                                    rotated_vertex[1]))
        return rotated_vertices

    def plan(self):  # KinoRRT
        ## ---- construct RRT tree ---- ##
        tree = RRTTree()
        state_initial = np.array([self.x_cm,self.y_cm,self.theta])
        tree.add_vertex(state_initial)
        goal_reached = False
        ## debug ##
        csv_node_header = ["vertex","x","y","theta","near_x","near_y"]
        csv_edge_header = ["eid","sid","v","d"]
        with open(f"tree_nodes_{self.i}.csv", 'w', newline='') as csvfile:
            csv_writer = csv.writer(csvfile)
            csv_writer.writerow(csv_node_header)
        with open(f"tree_edges_{self.i}.csv", 'w', newline='') as csvfile:
            csv_writer = csv.writer(csvfile)
            csv_writer.writerow(csv_edge_header)
        ## /debug ##
        while goal_reached is False:
            rand_x = random.randint(WALL_THICKNESS,WIDTH-WALL_THICKNESS-1)
            rand_y = random.randint(WALL_THICKNESS,HEIGHT-WALL_THICKNESS-1)
            rand_theta = random.uniform(0,2*pi)

            rand_state = np.array([rand_x,rand_y,rand_theta])
            rand_velocity = random.randint(5,10)
            rand_duration = random.randint(3,7)

            sid, vertex_near = tree.get_nearest_state(rand_state)
            new_theta = vertex_near.state[2] + rand_theta
            new_x = np.round(vertex_near.state[0] + 
                             rand_velocity*cos(new_theta)*rand_duration)
            new_y = np.round(vertex_near.state[1] - 
                             rand_velocity*sin(new_theta)*rand_duration)
            if self.collision_check(new_x,new_y,new_theta): 
                continue # if new node collides with walls, discard it and search new one

            new_state = (new_x,new_y,new_theta)
            ##--------------------------------##
            ## TODO -- occupancy grid updates ##
            ##--------------------------------##
            eid = tree.add_vertex(new_state)
            tree.add_edge(sid,eid,[rand_velocity,new_theta],rand_duration)
            goal_reached = self.goal_check(new_state, self.goal)

            ## debug
            csv_node_data = [eid,new_x,new_y,new_theta,vertex_near.state[0],vertex_near.state[1]]
            csv_edge_data = [eid,sid,rand_velocity,rand_duration]
            with open(f"tree_nodes_{self.i}.csv", 'a', newline='') as csvfile:
                csv_writer = csv.writer(csvfile)
                csv_writer.writerow(csv_node_data)
            with open(f"tree_edges_{self.i}.csv", 'a', newline='') as csvfile:
                csv_writer = csv.writer(csvfile)
                csv_writer.writerow(csv_edge_data)
            ## /debug ##
        return tree.get_first_move(state_initial)
    
    def set_environment_data(self,agents,walls):
        for agent in agents:
            self.agents_pose.append(agent.get_pose())
            self.agents_vel.append(agent.get_velocity())
            
        self.walls = walls

    def collision_check(self,x,y,theta):
        # find new vertices and min/max x/y of robot
        vertices = self.get_vertices(x,y,theta)
        x_min = WIDTH
        x_max = -1
        y_min = HEIGHT
        y_max = -1
        for vertex in vertices:
            if vertex[0] < x_min: x_min = vertex[0]
            if vertex[0] > x_max: x_max = vertex[0]
            if vertex[1] < y_min: y_min = vertex[1]
            if vertex[1] > y_max: y_max = vertex[1]

        # out of sbounds #
        if (x_min < 0 or x_max > WIDTH or y_min < 0 or y_max > HEIGHT):
            return True
        
        # collision with walls #
        for wall in self.walls:
            if (x_min < wall.x + wall.width and 
                x_max > wall.x and 
                y_min < wall.y + wall.height and 
                y_max > wall.y):
                # Calculate the x and y coordinates of the collision
                collision_x = min(max(x, wall.x), wall.x + wall.width)
                collision_y = min(max(y, wall.y), wall.y + wall.height)
                # print(f"collision: {collision_x},{collision_y}")
                return True
        return False

    def goal_check(self, state_new, state_goal):
        # check if state_new x and y are within goal +/- GOAL_TOLERANCE #
        if((state_new[0] <= (state_goal[0] + GOAL_TOLERACE) and 
            state_new[0] >= (state_goal[0] - GOAL_TOLERACE)) 
        and (state_new[1] <= (state_goal[1] + GOAL_TOLERACE) and 
             state_new[1] >= (state_goal[1] - GOAL_TOLERACE))):
                #print("goal check passed!") #DEBUG PRINTS
                return True
        return False
    