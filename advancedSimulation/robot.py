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
        self.x = x 
        self.y = y
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
        self.vertices = [(self.x-(self.length/2),self.y-(self.width/2)),
                         (self.x-(self.length/2),self.y+(self.width/2)),
                         (self.x+(self.length/2),self.y+(self.width/2)),
                         (self.x+(self.length/2),self.y-(self.width/2))]
        self.poly = pgeng.Polygon(self.vertices,BLUE)

        # debug parameters #
        self.i = 0 ## count csv files
        self.delta_dir = 1 # 1=ccw, -1=cw
        self.plan_set = 0
        self.plan_tree = []
        self.alpha_sum = 0

        

    def draw(self, screen):
        self.vertices = self.get_vertices(self.x,self.y,
                                          self.theta)
        self.poly.set_points(self.vertices)
        self.poly.render(screen)

    def move(self):
        if self.plan_set == 0:
            print(f"goal: {self.goal}")
            time_start = time.time()
            self.plan_tree, goal_vid = self.plan()
            # print(f"execution time = {time.time()-time_start}")
            while(goal_vid == -1):
                print("plan failed, retrying")
                time_start = time.time()
                self.plan_tree, goal_vid = self.plan()
                print(f"execution time = {time.time()-time_start}")
            # print(f"node count = {len(self.plan_tree.vertices)}")
            self.plan_tree = self.plan_tree.get_move_set(goal_vid)
            self.plan_set = 1
            self.i = 0
            plan = self.plan_tree[self.i]
            print(f"goal_vid = {goal_vid}")

        if self.move_duration < 1/FPS and self.i<=len(self.plan_tree)-1:
            plan = self.plan_tree[self.i]
            self.i += 1

            self.velocity = plan.u[0]
            self.delta = plan.u[1]
            self.move_duration = plan.t
        
        if self.i >= len(self.plan_tree) and self.move_duration < 1/FPS:
            self.plan_set = 0
            while(1):
                self.velocity = 0
            
        

        ## --DEBUG, no motion plan routine-- ##
        # if(self.plan_set == 0):
        #     # self.theta = pi/4
        #     self.velocity = 0
        #     self.delta = 0
        #     self.move_duration = 2
        #     self.plan_set = 1
        #     exp_x,exp_y,exp_theta = self.ackermann(self.x, self.y, self.theta,
        #                                                   self.delta, self.velocity,self.move_duration)
        #     print(f"expected state: ({exp_x},{exp_y},{exp_theta})")
        #     print(f"expected cost {self.compute_cost(self.velocity,self.move_duration)}")
        #     vertex = RRTVertex([self.x,self.y,self.theta], 0)
        #     path = self.path_collision_check(vertex,self.velocity,self.delta,self.move_duration)
        #     print(path)
        # if(self.move_duration <= 1/FPS):
        #     print(f"current state: ({self.x},{self.y},{self.theta})")
        #     while(1):
        #         self.velocity = 0
        # -------------------------------- ##
        
        self.x, self.y, self.theta = self.ackermann(self.x, self.y, self.theta,
                                                          self.delta, self.velocity)
        self.move_duration -= 1/FPS

    def ackermann(self, x, y, theta, delta, velocity, duration = 1/FPS):
        '''
        given state, command and duration - progress dynamic model of vehicle
        returns new state after motion.
        '''
        if delta == 0: # linear motion
            x += velocity*(duration)*cos(theta) 
            y -= velocity*(duration)*sin(theta)
            alpha = 0
        else: 
            if (duration < 1/FPS):
                return x,y,theta
            else:
                x,y,theta = self.ackermann(x,y,theta,delta,velocity,duration-(1/FPS))
                rotation_radius = self.wheelbase/tan(delta)
                alpha = (velocity*(1/FPS))/rotation_radius # calculate angle of actual rotation, "1" being a placeholder for 1 time unit, meaning, alpha = v*t/r, where t=1

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
        ## ----------------------------------- ##

        ## ---- construct RRT tree ---- ##
        tree = RRTTree()
        state_initial = np.array([self.x,self.y,self.theta])
        tree.add_vertex(state_initial, 0) # first node has 0 cost

        ## ---- DEBUG write 1st node to CSV ---- ##
        csv_data_nodes = [0,self.x,self.y,self.theta,0]
        with open(csv_name_nodes, 'a', newline='') as csvfile:
            csv_writer = csv.writer(csvfile)
            csv_writer.writerow(csv_data_nodes)
        ## ----------------------------------- ##
        k = 0   
        goal_cost = 100000 # unrealistically large initial cost to goal
        goal_vid = -1 # vertex id of the optimal vertex that reached the goal
        max_cost = self.compute_cost(MAX_VELOCITY,MAX_DURATION) # starting random cost upper limit is the highest cost of a single action
        while k < MAX_ITERATIONS:
            k += 1
            # randomize state, cost, command and duration
            rand_x = random.randint(WALL_THICKNESS,WIDTH-WALL_THICKNESS-1)
            rand_y = random.randint(WALL_THICKNESS,HEIGHT-WALL_THICKNESS-1)
            rand_theta = random.uniform(0,2*pi)
            rand_state = np.array([rand_x,rand_y,rand_theta])
            rand_cost = random.uniform(0,max_cost)

            rand_velocity = random.randint(MIN_VELOCITY,MAX_VELOCITY)
            rand_delta = random.uniform(-MAX_DELTA,MAX_DELTA)
            rand_duration = random.uniform(MIN_DURATION,MAX_DURATION)
            
            # get nearest vertex and propagate command, duration and cost
            sid, vertex_near = tree.get_nearest_state(rand_state, rand_cost)
            new_x, new_y, new_theta = self.ackermann(vertex_near.state[0], vertex_near.state[1], vertex_near.state[2],
                                                      rand_delta,rand_velocity,rand_duration)
            new_cost = vertex_near.cost + self.compute_cost(rand_velocity,rand_duration)
            # collision check
            if self.collision_check(new_x,new_y,new_theta): 
                continue # if new node collides with walls, discard it and search new one
            if goal_vid != -1 and new_cost >= goal_cost: 
                continue # if goal was found and the new node has a higher cost - discard it
            if self.path_collision_check(vertex_near,rand_velocity,rand_delta,rand_duration):
                continue # if collision was found along the path - discard the node.

            new_state = (new_x,new_y,new_theta)
            if new_cost > max_cost and goal_vid == -1: # if new cost is the highest cost-to-node and goal is yet to be found...
                max_cost = new_cost + self.compute_cost(MAX_VELOCITY,MAX_DURATION) # ...max cost is new cost + maximum action cost

            eid = tree.add_vertex(new_state, new_cost)
            tree.add_edge(eid,sid,[rand_velocity,rand_delta],rand_duration)
            goal_reached = self.goal_check(new_state, self.goal)
            if goal_reached is True and new_cost < goal_cost:
                goal_cost = new_cost
                goal_vid = eid
                max_cost = goal_cost # if goal is reached - max cost is the best cost-to-goal
            
            ## ---- DEBUG write to CSV ---- ##
            csv_data_edges = [eid,tree.edges[eid].sid,tree.edges[eid].u,tree.edges[eid].t]
            csv_data_nodes = [eid,new_x,new_y,new_theta,new_cost]
            with open(csv_name_edges, 'a', newline='') as csvfile:
                csv_writer = csv.writer(csvfile)
                csv_writer.writerow(csv_data_edges)
            with open(csv_name_nodes, 'a', newline='') as csvfile:
                csv_writer = csv.writer(csvfile)
                csv_writer.writerow(csv_data_nodes)
        csv_data_nodes = [MAX_ITERATIONS+1,self.goal[0],self.goal[1],0,0]
        with open(csv_name_nodes, 'a', newline='') as csvfile:
                csv_writer = csv.writer(csvfile)
                csv_writer.writerow(csv_data_nodes)
            ## ---------------------------- ##
        return tree, goal_vid
    
    def compute_cost(self,velocity, duration):
        D = 1 # distance coefficient
        distance = velocity*duration # compute total distance passed
        cost = D*distance
        return cost

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
                # DEBUG Calculate the x and y coordinates of the collision
                # collision_x = min(max(x, wall.x), wall.x + wall.width)
                # collision_y = min(max(y, wall.y), wall.y + wall.height)
                # print(f"collision: {collision_x},{collision_y}")
                return True
        return False

    def path_collision_check(self, start_vertex, velocity, delta, duration, eid = -1, sid = -1):
        '''
        perfrom collision check for the path from starting state
        along a given command
        eid is DEBUG parameter
        '''
        ## ---- DEBUG print collisions log init ---- ##
        csv_name_collision = "logs/collisions.csv"
        csv_header_collision = ["eid","sid","collision","x","y"]
        with open(csv_name_collision, 'w', newline='') as csvfile:
            csv_writer = csv.writer(csvfile)
            csv_writer.writerow(csv_header_collision)
        ## ----------------------------------------- ##
        x_start, y_start, theta_start = start_vertex.state
        x = x_start
        y = y_start
        theta = theta_start

        while duration >= 1/FPS:
            x,y,theta = self.ackermann(x, y, theta, delta, velocity)
            if self.collision_check(x,y,theta) is True:
                ## ---- DEBUG print collisions log ---- ##
                csv_data_collisions = [eid,sid,True,x,y]
                with open(csv_name_collision, 'a', newline='') as csvfile:
                    csv_writer = csv.writer(csvfile)
                    csv_writer.writerow(csv_data_collisions)
                ## ----------------------------------------- ##
                return True
            duration -= 1/FPS
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
    