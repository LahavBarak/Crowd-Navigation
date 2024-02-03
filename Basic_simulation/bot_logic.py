import random
from simulation import GridGUI
import numpy as np
import operator


def bot_logic(grid: GridGUI, state_goal):
    ## ---- construct occupancy grid ---- ##
    occupancy_grid = np.zeros([grid.gridSize,grid.gridSize], dtype=int)
    occupancy_grid[grid.bot.agents_pos_x[0],      # mark agent 1 position
                   grid.bot.agents_pos_y[0]] = 1  
    occupancy_grid[grid.bot.agents_pos_x[1],      
                   grid.bot.agents_pos_y[1]] = 1 # mark agent 2 position
    ## ---- construct RRT tree ---- ##
    tree = RRTTree()
    state_initial = np.array([grid.bot.self_pos_x, grid.bot.self_pos_y])
    tree.add_vertex(state_initial)

    ## ---- randomize next state ---- ##
    goal_reached = False
    while(goal_reached is False):
        state_rand = np.array([random.randint(0,grid.gridSize-1),
                            random.randint(0,grid.gridSize-1)]) #random state for robot x,y
        speed = np.array([random.randint(-1,1),
                            random.randint(-1,1)]) # random control input
        duration = random.randint(2,5) # random duration in turns

        sid, vertex_near = tree.get_nearest_state(state_rand)
        state_new = (vertex_near.state + duration*speed) % grid.gridSize
        
        ## -- if needed, update occupancy grid and collision check -- ##
        if (state_new[0] == state_initial[0] and state_new[1] == state_initial[1]):
            occupancy_grid = predict(grid, occupancy_grid, duration)
            if(collision_check(grid, occupancy_grid, vertex_near, speed, duration) is False):
                eid = tree.add_vertex(state_new)
                tree.add_edge(sid,eid, speed, duration)
                goal_reached = goal_check(state_new, state_goal)
                
        else:
            eid = tree.add_vertex(state_new)
            tree.add_edge(sid, eid, speed, duration)
            goal_reached = goal_check(state_new, state_goal)
        
        

    return tree.get_first_move(state_initial) 
        

def predict(grid, occupancy_grid, duration): 
    agent1_current_pose = [grid.agent1.position_x,grid.agent1.position_y]
    agent2_current_pose = [grid.agent2.position_x,grid.agent2.position_y]
    counter = 1
    while (counter <= duration):
        agent1_next_pose = [(agent1_current_pose[0] + grid.agent1.speed_x)%grid.gridSize,
                            (agent1_current_pose[1] + grid.agent1.speed_y)%grid.gridSize] 
        agent2_next_pose = ([(agent2_current_pose[0] + grid.agent2.speed_x)%grid.gridSize,
                            (agent2_current_pose[1] + grid.agent2.speed_y)%grid.gridSize])
        
        occupancy_grid[agent1_next_pose[0]-1, agent1_next_pose[1]-1]= 1 + counter ## i.e. 'occupied' in [counter] turns 
        occupancy_grid[agent2_next_pose[0]-1, agent2_next_pose[1]-1]= 1 + counter ## i.e. 'occupied' in [counter] turns 
        counter = counter + 1
    
    return occupancy_grid

def collision_check(grid, occupancy_grid, vertex_initial, speed, duration):
    current_state = vertex_initial.state
    counter = 1
    while (counter <= duration):
        bot_next_state = (current_state + speed) % grid.gridSize
        if (occupancy_grid[bot_next_state[0]-1,bot_next_state[1]-1] 
            == (1 + counter)):
            return True
        counter = counter + 1
    return False

def goal_check(state_new, state_goal):
    # check if state_new x and y are within goal +/- 1 #
    print(f"new {state_new[0]},{state_new[1]} , goal {state_goal[0]},{state_goal[1]}")
    print(f"{state_new[0] <= (state_goal[0] + 1)} , {state_new[0] >= (state_goal[0] - 1)} , {state_new[1] <= (state_goal[1] + 1)} , {state_new[1] >= (state_goal[1] - 1)}")
    if((state_new[0] <= (state_goal[0] + 1) and state_new[0] >= (state_goal[0] - 1)) 
       and (state_new[1] <= (state_goal[1] + 1) and state_new[1] >= (state_goal[1] - 1))):
            print("goal check passed!")
            return True
    return False

## RRT Tree and Vertex object, credited to Adi Levi from Mobile Robots course
class RRTTree(object):
    '''
    RRT Tree class
    '''
    def __init__(self):
        self.vertices = {}
        self.edges = {}

    def add_vertex(self, final_state):
        '''
        Add a state to the tree.
        @param final_state: state to add to the tree
        @param time: the time of the new added state
        @param mid_states: Sequence of configurations that end up in the final_state configuration
        '''
        vid = len(self.vertices)
        self.vertices[vid] = RRTVertex(final_state)
        return vid

    def add_edge(self, sid, eid, u, t):
        '''
        Adds an edge in the tree.
        @param sid start state ID
        @param eid end state ID
        @param u command from start to end
        @param t duration of u
        '''
        self.edges[eid] = RRTEdge(sid,u,t)

    def get_nearest_state(self, state):
        '''
        Find the nearest vertex for the given state and returns its state index and state
        @param state Sampled state.
        '''
        # compute distances from all vertices
        dists = []
        for _, vertex in self.vertices.items():
            dists.append(self.compute_distance(state, vertex.state))

        # retrieve the id of the nearest vertex
        vid, _ = min(enumerate(dists), key=operator.itemgetter(1))

        return vid, self.vertices[vid]
    
    def get_first_move(self, state_initial):
        '''
        Retrace the graph to find the edge of the first move to take
        '''
        current_vid = len(self.vertices)-1
        current_vertex = self.vertices[current_vid] ## start from the last vertex
        first_edge = None
        while (current_vertex.state[0] != state_initial[0] or 
               current_vertex.state[1] != state_initial[1]):
            first_edge = self.edges[current_vid]
            current_vertex = self.vertices[first_edge.sid]
            current_vid = first_edge.sid
        return first_edge
        


    def compute_distance(self, first_config, second_config):
        '''
        Computes the distance between two configurations.
        :param first_config: (x,y,theta) configuration
        :param second_config: (x,y,theta) configuration
        :return: The Euclidean distance between the two (x,y) coordinates
        '''
        return np.linalg.norm(second_config[0:1] - first_config[0:1])


class RRTVertex(object):
    '''
    RRT node class
    '''
    def __init__(self, state):
        self.state = state

class RRTEdge(object):
    '''
    RRT edge class
    @param eid end state ID
    @param u command from start to end
    @param t duration of u
    '''
    def __init__(self, sid, u, t):
        self.sid = sid
        self.u = u
        self.t = t
