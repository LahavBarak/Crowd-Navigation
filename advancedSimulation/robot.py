import random
import pgeng
import numpy as np
from numpy import pi, sin, cos, tan
from config import *
from RRT import *
import csv
import time


class Robot:
    def __init__(self, x, y, width, length, theta = pi/2, goal = [200,300]):
        # state space #
        self.x_cm = x 
        self.y_cm = y
        self.theta = theta # angle between forward-axis and X-axis

        # command space #
        self.velocity = 0
        self.delta = 0 # ackermann model angle between wheels and forward-axis

        # motion plan parameters #
        self.goal = goal
        self.move_duration = 0

        # expanded state space #
        self.agents_pose = []
        self.agents_vel = []
        self.walls = []

        # physical parameters #
        self.width = width
        self.length = length
        self.wheelbase = self.length - WHEELBASE_OFFSET
        self.vertices = [(self.x_cm-(self.length/2),self.y_cm-(self.width/2)),
                         (self.x_cm-(self.length/2),self.y_cm+(self.width/2)),
                         (self.x_cm+(self.length/2),self.y_cm+(self.width/2)),
                         (self.x_cm+(self.length/2),self.y_cm-(self.width/2))]
        self.poly = pgeng.Polygon(self.vertices,BLUE)

        # debug parameters #
        self.i = 0 ## count csv files
        self.delta_dir = 1 # 1=ccw, -1=cw
        self.plan_set = 0
        self.plan_tree = []
        self.alpha_sum = 0

        

    def draw(self, screen):
        self.vertices = self.get_vertices(self.x_cm,self.y_cm,
                                          self.theta)
        self.poly.set_points(self.vertices)
        self.poly.render(screen)

    def move(self):
        if self.plan_set == 0:
            print(f"goal: {self.goal}")
            self.plan_tree = self.plan()
            self.plan_tree = self.plan_tree.get_move_set()
            self.plan_set = 1
            self.i = 0
            plan = self.plan_tree[self.i]

        if self.move_duration < 1/FPS and self.i<=len(self.plan_tree)-1:
            plan = self.plan_tree[self.i]
            self.i += 1

            self.velocity = plan.u[0]
            self.delta = plan.u[1]
            self.move_duration = plan.t
            print(f"location: ({self.x_cm},{self.y_cm}), theta {self.theta}")
            print(f"step {self.i}: vel {self.velocity}, delta {self.delta}, duration {self.move_duration}")
            print(f"edge ({plan.sid})")
        
        if self.i >= len(self.plan_tree) and self.move_duration < 1/FPS:
            print(f"final location: ({self.x_cm},{self.y_cm}), theta {self.theta}")
            while(1):
                self.velocity = 0
            
        

        # --debug, no motion plan routine--
        # if(self.plan_set == 0):
        #     self.velocity = 2
        #     self.delta = 0.19867654309073712
        #     self.move_duration = 2.518178664400505
        #     self.plan_set = 1
        #     exp_x,exp_y,exp_theta = self.ackermann(self.x_cm, self.y_cm, self.theta,
        #                                                   self.delta, self.velocity, self.wheelbase,self.move_duration)
        #     print(f"expected state: ({exp_x},{exp_y},{exp_theta})")

        # if(self.move_duration <= 1/FPS):
        #     print(f"current state: ({self.x_cm},{self.y_cm},{self.theta})")
        #     while(1):
        #         self.velocity = 0
        
        self.x_cm, self.y_cm, self.theta = self.ackermann(self.x_cm, self.y_cm, self.theta,
                                                          self.delta, self.velocity, self.wheelbase)
        self.move_duration -= 1/FPS
        # print(f"time remaining {self.move_duration}")

    def ackermann(self, x, y, theta, delta, velocity, wheelbase, duration = 1/FPS):
        '''
        given state, command and duration - progress dynamic model of vehicle
        returns new state after motion.
        '''
        if delta == 0: # linear motion
            x += velocity*(duration*FPS)*cos(theta) 
            y -= velocity*(duration*FPS)*sin(theta)
            alpha = 0
        else: 
            if (duration < 1/FPS):
                return x,y,theta
            else:
                x,y,theta = self.ackermann(x,y,theta,delta,velocity,wheelbase,duration-(1/FPS))
                rotation_radius = wheelbase/tan(delta)
                alpha = (velocity*(1))/rotation_radius # calculate angle of actual rotation, "1" being a placeholder for 1 time unit, meaning, alpha = v*t/r, where t=1

                x += rotation_radius*(cos(alpha+theta-pi/2) - cos(theta-pi/2))
                y -= rotation_radius*(sin(alpha+theta-pi/2) - sin(theta-pi/2))
                theta += alpha/2 

        return x,y,theta

    def get_vertices(self,x_cm, y_cm, theta):
        '''
        given x,y and theta - return vertex locations of the rectangle.

        reset vertices to parallel position, displaced to new center,
        then rotate by theta radians to reflect the theta of the robot
        '''
        vertices = ([x_cm-(self.length/2),y_cm-(self.width/2)],
                    [x_cm-(self.length/2),y_cm+(self.width/2)],
                    [x_cm+(self.length/2),y_cm+(self.width/2)],
                    [x_cm+(self.length/2),y_cm-(self.width/2)])
        cm = np.array([x_cm,y_cm])
        rotated_vertices = []
        for vertex in vertices:
            np_vertex = np.array(vertex)
            rotated_vertex = np.dot(np_vertex-cm,self.R(theta))
            rotated_vertex += cm
            rotated_vertices.append((rotated_vertex[0],
                                    rotated_vertex[1]))
        return rotated_vertices

    def plan(self):  # AO-KinoRRT
        ## ---- DEBUG print RRT graph init ---- ##
        csv_name_edges = "logs/tree_edges.csv"
        csv_name_nodes = "logs/tree_nodes.csv"
        csv_header_edges = ["eid","sid","u","t"]
        csv_header_nodes = ["vid","x","y","theta","cost"]
        with open(csv_name_edges, 'w', newline='') as csvfile:
            csv_writer = csv.writer(csvfile)
            csv_writer.writerow(csv_header_edges)
        with open(csv_name_nodes, 'w', newline='') as csvfile:
            csv_writer = csv.writer(csvfile)
            csv_writer.writerow(csv_header_nodes)

        ## ---- construct RRT tree ---- ##
        tree = RRTTree()
        state_initial = np.array([self.x_cm,self.y_cm,self.theta])
        tree.add_vertex(state_initial, 0) # first node has 0 cost
        goal_reached = False

        ## ---- DEBUG write 1st node to CSV ---- ##
        csv_data_nodes = [0,self.x_cm,self.y_cm,self.theta,0]
        with open(csv_name_nodes, 'a', newline='') as csvfile:
            csv_writer = csv.writer(csvfile)
            csv_writer.writerow(csv_data_nodes)
        ## ---- /DEBUG write to CSV ---- ##
                
        while goal_reached is False:
            # randomize state, cost, command and duration
            rand_x = random.randint(WALL_THICKNESS,WIDTH-WALL_THICKNESS-1)
            rand_y = random.randint(WALL_THICKNESS,HEIGHT-WALL_THICKNESS-1)
            rand_theta = random.uniform(0,2*pi)
            rand_state = np.array([rand_x,rand_y,rand_theta])
            rand_cost = random.uniform(0,MAX_EUCLIDEAN_COST)

            rand_velocity = random.randint(1,4)
            rand_delta = random.uniform(-pi/3,pi/3)
            rand_duration = random.uniform(1,4)
            #------------------------------------------#
            # get nearest vertex and propagate command, duration and cost
            sid, vertex_near = tree.get_nearest_state(rand_state, rand_cost)
            new_x, new_y, new_theta = self.ackermann(vertex_near.state[0], vertex_near.state[1], vertex_near.state[2],
                                                      rand_delta,rand_velocity,self.wheelbase,rand_duration)
            new_cost = vertex_near.cost + tree.compute_distance([new_x,new_y,new_theta],vertex_near.state)
            #------------------------------------------#

            if self.collision_check(new_x,new_y,new_theta): 
                continue # if new node collides with walls, discard it and search new one
            new_state = (new_x,new_y,new_theta)

            eid = tree.add_vertex(new_state, new_cost)
            tree.add_edge(eid,sid,[rand_velocity,rand_delta],rand_duration)
            goal_reached = self.goal_check(new_state, self.goal)

            ## ---- DEBUG write to CSV ---- ##
            print(f"node {eid}: vertex near = {sid},{vertex_near.state}, command = {rand_delta},{rand_velocity},{rand_duration}")
            csv_data_edges = [eid,tree.edges[eid].sid,tree.edges[eid].u,tree.edges[eid].t]
            csv_data_nodes = [eid,new_x,new_y,new_theta,new_cost]
            with open(csv_name_edges, 'a', newline='') as csvfile:
                csv_writer = csv.writer(csvfile)
                csv_writer.writerow(csv_data_edges)
            with open(csv_name_nodes, 'a', newline='') as csvfile:
                csv_writer = csv.writer(csvfile)
                csv_writer.writerow(csv_data_nodes)

        return tree
    
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

        # out of bounds #
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
                return True
        return False
    
    def R(self,theta):
        return np.array([[cos(theta),-sin(theta)],
                        [sin(theta),cos(theta)]])
    