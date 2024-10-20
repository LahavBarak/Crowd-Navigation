import random
import pgeng
import numpy as np
from numpy import pi, sin, cos, tan
from config import *
from RRT import *
from utils import *
from motion_models import ackermann, linear
import csv
import time
from multiprocessing import Process, Queue


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
        self.plan_tree = []
        self.plan_set = 0

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

        # multiprocess parameters
        self.q = None
        self.plan_process = None
        self.boot = True # flag for the very first time beginning a move

        # debug parameters #
        self.i = 0
        self.t0 = time.time()

    def draw(self, screen):
        self.vertices = get_vertices(self.x,self.y, self.theta,
                                     self.length, self.width)
        self.poly.set_points(self.vertices)
        self.poly.render(screen)

    def move(self):
        if self.boot == True :
            ## print(f"goal: {self.goal}")
            self.q = Queue()
            self.plan_process = Process(target=self.plan, 
                                        args=(self.x,self.y,self.theta,self.q))
            self.plan_process.start()
            self.boot = False

        if self.plan_set == 0:
            self.plan_process.join() # finish current plan 
            self.plan_tree, goal_vid = self.q.get() # acquire data from plan
            ## print(f"goal vid = {goal_vid}")
            if goal_vid == -1: # when no moves are available, try a little reverse
                self.velocity = -MIN_VELOCITY
                self.delta = 0
                self.move_duration = MIN_DURATION
            else:
                plan = self.plan_tree.get_first_move(goal_vid)
                self.velocity = plan.u[0]
                self.delta = plan.u[1]
                self.move_duration = plan.t
                ## print(f"plan: {self.velocity}, {self.delta}, {self.move_duration}")
            self.plan_set = 1
            ## print(f"file #{self.i}, goal_vid = {goal_vid}") ## DEBUG for plotting graphs

            # begin next plan, starting from the end state of the current plan
            x,y,theta = ackermann(self.wheelbase,self.x,self.y,self.theta,
                                  self.delta,self.velocity,self.move_duration)
            self.q = Queue()
            self.i += 1 ## DEBUG
            self.plan_process = Process(target=self.plan, 
                                        args=(x,y,theta,self.q))
            self.plan_process.start() # begin next plan
        
        self.move_duration -= 1/FPS
        if self.move_duration < 1/FPS:
            self.plan_set = 0
            if self.goal_check((self.x,self.y),self.goal) is True:
                boot = True 
        
        self.x, self.y, self.theta = ackermann(self.wheelbase,self.x,self.y,
                                               self.theta,self.delta, self.velocity)        
        # print(f"pose t.{self.move_duration},{self.x},{self.y},{self.theta}")

    def plan(self, x_init,y_init,theta_init, q):  # AO-KinoRRT
        ## ---- DEBUG print RRT graph init ---- ##
        csv_name_edges = f"logs/edges/tree_edges_{self.i}.csv"
        csv_name_nodes = f"logs/nodes/tree_nodes_{self.i}.csv"
        csv_name_collisions = f"logs/collisions/collisions_{self.i}.csv"
        csv_header_edges = ["eid","sid","u","t"]
        csv_header_nodes = ["vid","x","y","theta","cost"]
        csv_header_collisions = ["agent","agent x","agent y","agent v","timestamp"]
        with open(csv_name_edges, 'w', newline='') as csvfile:
            csv_writer = csv.writer(csvfile)
            csv_writer.writerow(csv_header_edges)
        with open(csv_name_nodes, 'w', newline='') as csvfile:
            csv_writer = csv.writer(csvfile)
            csv_writer.writerow(csv_header_nodes)
        with open(csv_name_collisions, 'w', newline='') as csvfile:
            csv_writer = csv.writer(csvfile)
            csv_writer.writerow(csv_header_collisions)
        ## ----------------------------------- ##

        ## ---- construct RRT tree ---- ##
        tree = RRTTree()
        state_initial = np.array([x_init,y_init,theta_init])
        tree.add_vertex(state_initial, 0, 0) # first node has 0 cost and 0 distance

        ## ---- DEBUG write 1st node to CSV ---- ##
        csv_data_nodes = [0,x_init,y_init,theta_init,0]
        with open(csv_name_nodes, 'a', newline='') as csvfile:
            csv_writer = csv.writer(csvfile)
            csv_writer.writerow(csv_data_nodes)
        ## ----------------------------------- ##
        initial_time = time.time() 
        goal_cost = 100000 # unrealistically large initial cost to goal
        goal_vid = -1 # vertex id of the optimal vertex that reached the goal
        proximity = 1000 # arbitrarily large value
        max_cost, _ = self.compute_cost(x_init,y_init,MAX_VELOCITY,MAX_DURATION,
                                        MAX_DELTA, self.goal, 0, proximity) # starting random cost upper limit is the highest cost of a single action
        while time.time()-initial_time < MAX_RUNTIME:
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
            new_x, new_y, new_theta = ackermann(self.wheelbase,vertex_near.state[0],
                    vertex_near.state[1],vertex_near.state[2],rand_delta,rand_velocity,rand_duration)
            
            # filtering checks (collision, best cost)
            if self.collision_check(new_x,new_y,new_theta): 
                continue # if new node collides with walls, discard it and search new one
            if self.path_collision_check(vertex_near,rand_velocity,rand_delta,rand_duration):
                continue # if collision was found along the path - discard the node.
            if sid == 0 : # only perform collision check with agents if the new node is connected to starting position
                collision, proximity = self.agent_collision_check(vertex_near,rand_velocity,rand_delta,rand_duration,self.agents_pose,self.agents_vel,
                                                                  csv_name_collisions,new_x,new_y,new_theta)
                if collision is True:
                    continue

            new_cost, new_distance = self.compute_cost(new_x,new_y,rand_velocity,rand_duration,rand_delta,
                                                       self.goal, vertex_near.distance, proximity)
            new_state = (new_x,new_y,new_theta)
            if new_cost > max_cost and goal_vid == -1: # if new cost is the highest cost-to-node and goal is yet to be found...
                max_cost, _ = new_cost + self.compute_cost(x_init,y_init,MAX_VELOCITY,MAX_DELTA,
                                                           MAX_DURATION,self.goal, 0, proximity) # ...max cost is new cost + maximum action cost

            eid = tree.add_vertex(new_state, new_cost, new_distance)
            tree.add_edge(eid,sid,[rand_velocity,rand_delta],rand_duration)
            if new_cost < goal_cost:
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
        q.put((tree, goal_vid))
    
    def compute_cost(self, x, y, velocity, delta, duration, goal, prev_distance, proximity):
            distance = prev_distance + velocity*duration # compute total distance passed
            distance_to_goal = np.sqrt((goal[0]-x)**2 + (goal[1]-y)**2)
            cost = D*distance + G*distance_to_goal + P/(proximity+0.001) + C*abs(delta)
            if self.goal_check((x,y),goal) is True:
                cost = P/(proximity+0.001) # if a node reached the goal - go there unless it's REALLY close to an agent
            return cost, distance*D


    def collision_check(self,x,y,theta):
        # find new vertices and min/max x/y of robot
        x_min,x_max,y_min,y_max = self.get_x_y_min_max(x,y,theta)

        # out of bounds #
        if (x_min < 0 or x_max > WIDTH or y_min < 0 or y_max > HEIGHT):
            return True
        
        # collision with walls #
        for wall in self.walls:
            if (x_min < wall.x + wall.width and 
                x_max > wall.x and 
                y_min < wall.y + wall.height and 
                y_max > wall.y):
                return True
        return False

    def path_collision_check(self, start_vertex, velocity, delta, duration):
        x, y, theta = start_vertex.state
        while duration >= 1/FPS:
            x,y,theta = ackermann(self.wheelbase, x, y, theta, 
                                  delta, velocity)
            if self.collision_check(x,y,theta) is True:
                return True
            duration -= 1/FPS
        return False
    
    def agent_collision_check(self, start_vertex, velocity, delta, duration, agent_positions, agent_velocities,
                              csv_name_collisions,new_x,new_y,new_theta): ## these 4 params are DEBUG
        x, y, theta = start_vertex.state
        first_run = True # DEBUG flag to collect only agent path belief.
        while duration >= 1/FPS:
            x,y,theta = ackermann(self.wheelbase, x, y, theta, 
                                  delta, velocity)
            x_min,x_max,y_min,y_max = self.get_x_y_min_max(x,y,theta)
            proximity = 1000 # arbitrarily large value
            for agent in range(AGENT_NUM):
                # propagate agent position
                agent_x = agent_positions[agent][0] 
                agent_y = agent_positions[agent][1] 
                agent_theta = agent_positions[agent][2] 
                agent_x,agent_y = linear(agent_x,agent_y,agent_theta,
                                         agent_velocities[agent],1/FPS)
                # compute collision & proximity
                if (x_min < agent_x+AGENT_RADIUS and x_max > agent_x-AGENT_RADIUS 
                    and y_min < agent_y+AGENT_RADIUS and y_max > agent_y-AGENT_RADIUS):
                    return True, 0
                proximity = min(proximity, minimal_distance((
                    agent_x,agent_y),AGENT_RADIUS,get_vertices(x,y,theta, 
                                                    self.length, self.width)))
                agent_positions[agent] = agent_x,agent_y,agent_theta
                # ## ---- DEBUG write to CSV ---- ##
                if first_run is True:
                    csv_data_collisions = [agent,agent_x,agent_y,agent_velocities[agent],time.time()-self.t0]
                    with open(csv_name_collisions, 'a', newline='') as csvfile:
                        csv_writer = csv.writer(csvfile)
                        csv_writer.writerow(csv_data_collisions)
                # ## ---------------------------- ##
            duration -= 1/FPS
            first_run = False # DEBUG first loop ended.
        return False, proximity

    def goal_check(self, state_new, state_goal):
        # check if state_new x and y are within goal +/- GOAL_TOLERANCE #
        if((state_new[0] <= (state_goal[0] + GOAL_TOLERACE) and 
            state_new[0] >= (state_goal[0] - GOAL_TOLERACE)) 
        and (state_new[1] <= (state_goal[1] + GOAL_TOLERACE) and 
             state_new[1] >= (state_goal[1] - GOAL_TOLERACE))):
                return True
        return False
    
    def set_environment_data(self,agents,walls):
        agents_pose = []
        agents_vel = []
        for agent in agents:
            agents_pose.append(agent.get_pose())
            agents_vel.append(agent.get_velocity())

        self.agents_pose = agents_pose
        self.agents_vel = agents_vel   
        self.walls = walls

    def get_x_y_min_max(self,x,y,theta):
        vertices = get_vertices(x,y,theta,self.length, self.width)
        x_min = WIDTH
        x_max = -1
        y_min = HEIGHT
        y_max = -1
        for vertex in vertices:
            if vertex[0] < x_min: x_min = vertex[0]
            if vertex[0] > x_max: x_max = vertex[0]
            if vertex[1] < y_min: y_min = vertex[1]
            if vertex[1] > y_max: y_max = vertex[1]

        return x_min,x_max,y_min,y_max

    
    
