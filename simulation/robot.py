import random
import pgeng
import numpy as np
from numpy import pi, sin, cos, tan
from config import *
from RRT import *
from utils import *
from motion_models import ackermann, linear_cart, ackermann_samples
from global_planner import PRMGraph
import csv
import time
import math
from multiprocessing import Process, Queue


class Robot:
    def __init__(self, x, y, width, length, theta = pi/2, goal = [200,300]):
        # state space #
        self.x = x 
        self.y = y
        self.theta = theta # angle between forward-axis and X-axis

        # expanded state space #
        self.agents_pose = []
        self.agents_vel = []
        self.walls = []

        # action space #
        self.velocity = 0
        self.delta = 0 # ackermann model angle between wheels and forward-axis
        self.duration = 0

        # global plan parameters #
        self.goal = goal
        self.goal_reached = False
        self.PRM_graph = PRMGraph()
        self.global_plan = []

        # local plan parameters
        self.local_goal = None
        self.plan_set = 0

        # physical parameters #
        self.width = width
        self.length = length
        self.wheelbase = self.length - WHEELBASE_OFFSET
        self.vertices = [(self.x-(self.length/2),self.y-(self.width/2)),
                         (self.x-(self.length/2),self.y+(self.width/2)),
                         (self.x+(self.length/2),self.y+(self.width/2)),
                         (self.x+(self.length/2),self.y-(self.width/2))]
        self.poly = pgeng.Polygon(self.vertices,BLUE)
        self.move_steps = [] # breakdown of a single move for animation purpose

        # multiprocess parameters
        self.q = None
        self.plan_process = None
        self.boot = True # flag for the very first time beginning a move

        # debug parameters #
        self.i = 0
        self.t0 = time.time()
        self.timer = 0

    def draw(self, screen):
        self.vertices = get_vertices(self.x,self.y, self.theta,
                                     self.length, self.width)
        self.poly.set_points(self.vertices)
        self.poly.render(screen)

    def move(self):
        if self.boot == True :
            # plan global trajectory
            if self.PRM_graph.is_empty(): # only build PRM graph once per simulation
                if MAP_FILEPATH is not None: ## load a pregenerated map, mostly to save time generating it each run, so sort of DEBUG
                    self.PRM_graph.read(MAP_FILEPATH,self.walls)
                else:
                    self.PRM_graph.build_graph(PRM_SAMPLES, WIDTH, HEIGHT, self.walls)
                    self.PRM_graph.write(f"PRM_GRAPHS/{MAP}_{PRM_SAMPLES}_samples.json")

            start = (self.x,self.y) 
            self.global_plan = self.PRM_graph.find_path(start,self.goal) # plan global plan once per boot (= when goal is generated)
            if self.global_plan is None:
                print("No global plan found")
                return -1

            # plan local trajectory
            self.q = Queue()
            self.local_goal = self.get_local_goal() 
            self.plan_process = Process(target=self.plan, 
                                        args=(self.x,self.y,self.theta,self.q))
            self.plan_process.start()
            self.boot = False

        if self.plan_set == 0:      
            t0 = time.time() #DEBUG TIMING
            self.plan_process.join(timeout=MAX_RUNTIME+0.5) # finish current plan, wait for maximum runtime + 0.5 sec or force break
            if self.plan_process.exitcode is None:
                print("timeout")
            self.velocity, self.delta, self.duration = self.q.get() # acquire data from plan
            self.plan_set = 1
            self.move_steps = ackermann_samples(self.wheelbase,self.x,self.y,self.theta,
                                    self.delta, self.velocity,self.duration) # compute planned path and save its waypoints
            # begin next plan, starting from the end state of the current plan
            x,y,theta = self.move_steps[-1] # begin re-planning from the end of the current plan
            self.q = Queue()
            self.i += 1 ## DEBUG
            self.local_goal = self.get_local_goal()
            self.plan_process = Process(target=self.plan, 
                                        args=(x,y,theta,self.q))
            self.plan_process.start() # begin next plan
        
        self.x, self.y, self.theta = self.move_steps.pop(0) # execute waypoints one by one
        if not self.move_steps:
            self.plan_set = 0
            if self.goal_reached is True:
                self.boot = True 
                self.goal_reached = False       

    def plan(self, x_init,y_init,theta_init, q :Queue):  # AO-KinoRRT
         ## ---- DEBUG print RRT graph init ---- ##
        nodes_file = f"logs/nodes/tree_nodes_{self.i}.csv"
        edges_file = f"logs/edges/tree_edges_{self.i}.csv"
        collisions_file = f"logs/collisions/collisions_{self.i}.csv"
        nodes_fh = open(nodes_file, 'w', newline='')
        nodes_writer = csv.writer(nodes_fh)
        nodes_writer.writerow(["vid","x","y","theta","cost"])
        edges_fh = open(edges_file, 'w', newline='')
        edges_writer = csv.writer(edges_fh)
        edges_writer.writerow(["eid","sid","u","t", "rand_x", "rand_y"])
        collisions_fh = open(collisions_file, 'w', newline='')
        collisions_writer = csv.writer(collisions_fh)
        collisions_writer.writerow(["agent","agent x","agent y","agent v","timestamp"])
        ## ----------------------------------- ##
        ## ---- DEBUG profiling init ---- ##
        randomizing_acc,tree_finding_acc, col_acc, path_col_acc, ag_col_acc,graph_acc,tree_size = 0,0,0,0,0,0,0
        runtime_str = f"{MAX_RUNTIME:.2f}".replace('.', '_')
        csv_name_profiling = f"logs/profiles/runtime_{runtime_str}.csv"
        ## ------------------------------ ##

        ## ---- construct RRT tree ---- ##
        tree = RRTTree()
        state_initial = np.array([x_init,y_init,theta_init])
        tree.add_vertex(state_initial, 0, 0) # first node has 0 cost and 0 distance

        ## ---- DEBUG write 1st node to CSV ---- ##
        nodes_writer.writerow([0, x_init, y_init, theta_init, 0])
        ## ----------------------------------- ##

        initial_time = time.time()
        goal_cost = 100000 # unrealistically large initial cost to goal
        goal_vid = -1 # vertex id of the optimal vertex that reached the goal
        proximity = 1000 # arbitrarily large value
        max_cost, _ = self.compute_cost(x_init,y_init,MAX_VELOCITY,MAX_DURATION,
                                        MAX_DELTA, self.local_goal, 0, proximity) # starting random cost upper limit is the highest cost of a single action
        while time.time()-initial_time < MAX_RUNTIME:
            # randomize state, cost, command and duration
            t0 = time.time()#DEBUG profiling
            x_min, x_max, y_min, y_max = self.get_RRT_search_bounds()
            rand_x = random.randint(x_min,x_max)
            rand_y = random.randint(y_min,y_max)
            rand_theta = random.uniform(0,2*pi)
            rand_state = np.array([rand_x,rand_y,rand_theta])
            rand_cost = random.uniform(0,max_cost)

            rand_velocity = random.randint(MIN_VELOCITY,MAX_VELOCITY)
            rand_delta = random.uniform(-MAX_DELTA,MAX_DELTA)
            rand_duration = random.uniform(MIN_DURATION,MAX_DURATION)
            randomizing_acc += time.time() - t0#DEBUG profiling
            # get nearest vertex and propagate command, duration and cost
            t0 = time.time()#DEBUG profiling
            sid, vertex_near = tree.get_nearest_state(rand_state, rand_cost)
            new_x, new_y, new_theta = ackermann(self.wheelbase,vertex_near.state[0],
                    vertex_near.state[1],vertex_near.state[2],rand_delta,rand_velocity,rand_duration)
            tree_finding_acc += time.time() -t0#DEBUG profiling
            t0 = time.time()#DEBUG profiling
            if self.collision_check(new_x,new_y,new_theta): 
                continue # if new node collides with walls, discard it and search new one
            col_acc += time.time() -t0#DEBUG profiling
            t0 = time.time()#DEBUG profiling
            resolution = int((0.5*ROBOT_LENGTH)/rand_velocity)
            if self.path_collision_check(vertex_near,rand_velocity,rand_delta,rand_duration):
                continue # if collision was found along the path - discard the node.
            path_col_acc += time.time() -t0#DEBUG profiling
            t0 = time.time()#DEBUG profiling
            if sid == 0 and AGENT_NUM > 0 : # only perform collision check with agents if the new node is connected to starting position
                collision, proximity = self.agent_collision_check(vertex_near,rand_velocity,rand_delta,rand_duration,
                                                                  self.agents_pose,self.agents_vel)
                if collision is True:
                    continue
            ag_col_acc += time.time() -t0 #DEBUG profiling
            t0 = time.time()
            new_cost, new_distance = self.compute_cost(new_x,new_y,rand_velocity,rand_duration,rand_delta,
                                                       self.local_goal, vertex_near.cost, proximity)
            new_state = (new_x,new_y,new_theta)
            if new_cost > max_cost and goal_vid == -1: # if new cost is the highest cost-to-node and goal is yet to be found...
                max_cost, _ = new_cost + self.compute_cost(x_init,y_init,MAX_VELOCITY,MAX_DELTA,
                                                           MAX_DURATION,self.local_goal, 0, proximity) # ...max cost is new cost + maximum action cost

            eid = tree.add_vertex(new_state, new_cost, new_distance)
            tree.add_edge(eid,sid,[rand_velocity,rand_delta],rand_duration)
            if new_cost < goal_cost:
                goal_cost = new_cost
                goal_vid = eid
                max_cost = goal_cost # if goal is reached - max cost is the best cost-to-goal
            graph_acc += time.time() - t0
         ## ---- DEBUG write to CSV ---- ##
            nodes_writer.writerow([eid, new_x, new_y, new_theta, new_cost])
            edges_writer.writerow([eid, sid, [rand_velocity, rand_delta], rand_duration,  rand_x, rand_y])
        nodes_writer.writerow([MAX_ITERATIONS+1,self.goal[0],self.goal[1],0,0]) # add dummy-node at the end
        print(f"graph {self.i} has goal vid {goal_vid}")
        ## ---------------------------- ##
        ## DEBUG PROFILING ##
        total_time = time.time()-initial_time
        tree_size = len(tree.vertices)
        csv_data_profiling = [self.i,total_time,tree_finding_acc,col_acc,
                              path_col_acc,ag_col_acc,graph_acc,tree_size]
        with open(csv_name_profiling, 'a', newline='') as csvfile:
            csv_writer = csv.writer(csvfile)
            csv_writer.writerow(csv_data_profiling)
        ## ---------------------------- ##
        time.sleep(0.002) ## sleep timer to allow smooth multiprocessing. 
        velocity, delta, duration = self.extract_move(tree, goal_vid, x_init, y_init, theta_init)
        q.put((velocity, delta, duration))

    
    def compute_cost(self, x, y, velocity, delta, duration, goal, prev_cost, proximity):
            goal_reached = 1 if self.goal_check((x,y),goal) else 0
            distance = velocity*duration
            # distance = prev_distance + velocity*duration # compute total distance passed
            distance_to_goal = np.sqrt((goal[0]-x)**2 + (goal[1]-y)**2)
            cost = D*distance + G*distance_to_goal + P/(proximity+0.001) + C*abs(delta) - GR*goal_reached
            return cost + prev_cost, distance*D

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

    def path_collision_check(self, start_vertex, velocity, delta, duration, resolution = COLLISION_CHECK_RESOLUTION):
        x, y, theta = start_vertex.state
        dt = resolution
        while dt <= duration:
            x,y,theta = ackermann(self.wheelbase, x, y, theta, 
                                  delta, velocity, duration=resolution)
            if self.collision_check(x,y,theta) is True:
                return True
            dt += resolution
        return False
    
    def agent_collision_check(self, start_vertex, velocity, delta, duration, agent_positions, 
                              agent_velocities, resolution = COLLISION_CHECK_RESOLUTION):
        x, y, theta = start_vertex.state
        dt = resolution
        while dt <= duration:
            x,y,theta = ackermann(self.wheelbase, x, y, theta, 
                                  delta, velocity, duration=resolution)
            x_min,x_max,y_min,y_max = self.get_x_y_min_max(x,y,theta)
            proximity = 1000 # arbitrarily large value
            for agent in range(AGENT_NUM):
                # propagate agent position
                agent_x,agent_y = agent_positions[agent]
                agent_v_x, agent_v_y = agent_velocities[agent]
                agent_x,agent_y = linear_cart(agent_x,agent_y,agent_v_x,
                                         agent_v_y,duration=resolution)
                # compute collision & proximity
                if (x_min < agent_x+AGENT_RADIUS and x_max > agent_x-AGENT_RADIUS 
                    and y_min < agent_y+AGENT_RADIUS and y_max > agent_y-AGENT_RADIUS):
                    return True, 0
                proximity = min(proximity, minimal_distance((
                    agent_x,agent_y),AGENT_RADIUS,
                    get_vertices(x,y,theta,self.length, self.width))) / PIXELS_PER_METER
                agent_positions[agent] = agent_x,agent_y
            dt += resolution
        return False, proximity

    def goal_check(self, state_new, state_goal):
        # check if state_new x and y are within goal +/- GOAL_TOLERANCE #
        if((state_new[0] <= (state_goal[0] + GOAL_TOLERACE) and 
            state_new[0] >= (state_goal[0] - GOAL_TOLERACE)) 
        and (state_new[1] <= (state_goal[1] + GOAL_TOLERACE) and 
             state_new[1] >= (state_goal[1] - GOAL_TOLERACE))):
                return True
        return False
    
    def extract_move(self,tree: RRTTree,goal_vid, x0,y0,theta0):
        if goal_vid == -1: # when no moves are available, try a little reverse
            velocity = -50
            delta = 0
            duration = MIN_DURATION
        else:
            plan = tree.get_first_move(goal_vid)
            velocity = plan.u[0]
            delta = plan.u[1]
            duration = plan.t
        return velocity, delta, duration

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

    def get_global_path_points(self):
        points = []
        if self.global_plan:
            for node_id in self.global_plan:
                node_data = self.PRM_graph.graph.nodes[node_id]
                points.append((node_data['x'], node_data['y']))
        return points
    
    def get_local_goal(self):
        # if there's no global plan yet, nothing to do
        if not self.global_plan:
            return None

        # 1) convert node IDs in global_plan to (x, y) coords
        points = self.get_global_path_points()

        # 2) find the waypoint index closest to the robot's current pose
        #    we already have numpy imported as np and math imported
        dists = [math.hypot(px - self.x, py - self.y) for px, py in points]
        nearest_idx = int(np.argmin(dists))

        # 3) compute the lookahead index
        lookahead_idx = nearest_idx + LOOKAHEAD_INDEX
        # clamp so we don't run past the end
        lookahead_idx = min(lookahead_idx, len(points) - 1)

        # 4) return that exact waypoint as [x, y]
        x_la, y_la = points[lookahead_idx]
        return [x_la, y_la]
    
    def get_RRT_search_bounds(self):
        x_min = int(max((self.x - SEARCH_RADIUS), WALL_THICKNESS))
        x_max = int(min((self.x + SEARCH_RADIUS),WIDTH-WALL_THICKNESS-1))
        y_min = int(max((self.y - SEARCH_RADIUS),WALL_THICKNESS))
        y_max = int(min((self.y + SEARCH_RADIUS),HEIGHT-WALL_THICKNESS-1))
        return x_min, x_max, y_min, y_max