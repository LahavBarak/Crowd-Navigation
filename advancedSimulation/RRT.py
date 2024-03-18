import operator
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation

class RRTTree(object):
    '''
    RRT Tree class
    '''
    def __init__(self):
        self.vertices = {}
        self.edges = {}

    def add_vertex(self, state, cost):
        '''
        Add a state to the tree.
        @param state: state to add to the tree
        @param cost: the cost to reach this node
        '''
        vid = len(self.vertices)
        self.vertices[vid] = RRTVertex(state, cost)
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

    def get_nearest_state(self, state, cost):
        '''
        Find the nearest vertex for the given state and returns its state index and state
        @param state Sampled state.
        '''
        # compute distances from all vertices
        dists = []
        for _, vertex in self.vertices.items():
            dists.append(self.compute_distance(state, vertex.state, cost, vertex.cost))

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

    def compute_distance(self, state_1, state_2, cost_1 = 0, cost_2 = 0 ):
        '''
        Computes the distance between two configurations.
        @param state_i: (x,y,theta) configuration
        @param cost_i: cost to reach
        @return: The Euclidean distance between the two (x,y) coordinates
        '''
        vector_1 = np.array([state_1[0],state_1[1],cost_1])
        vector_2 = np.array([state_2[0],state_2[1],cost_2])
        return np.linalg.norm(vector_2 - vector_1)

class RRTVertex(object):
    '''
    RRT node class
    '''
    def __init__(self, state, cost):
        self.state = state
        self.cost = cost

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

class RRTVisualizer:
    def __init__(self, tree):
        self.tree = tree
        self.fig, self.ax = plt.subplots(figsize=(10, 8))
        self.ax.set_xlim(0, 800)
        self.ax.set_ylim(0, 600)
        self.ax.set_xlabel('X')
        self.ax.set_ylabel('Y')
        self.ax.set_title('RRT Tree Visualization')
        self.ax.grid(True)
        self.scatters = {}
        self.lines = {}
        self.init_plot()

    def init_plot(self):
        for vertex_id, vertex in self.tree.vertices.items():
            self.scatters[vertex_id] = self.ax.scatter(vertex.state[0], vertex.state[1], color='blue', s=100, zorder=2)
            self.ax.text(vertex.state[0], vertex.state[1], str(vertex_id), fontsize=12, ha='center', va='center', zorder=3)
            for edge_id, edge in self.tree.edges.items():
                if edge.sid == vertex_id:
                    start_vertex = self.tree.vertices[edge.sid]
                    end_vertex = self.tree.vertices[edge_id]
                    self.lines[edge_id] = self.ax.plot([start_vertex.state[0], end_vertex.state[0]],
                                                        [start_vertex.state[1], end_vertex.state[1]], color='black', zorder=1)[0]

    def update_plot(self, frame):
        # Update existing vertices
        for vertex_id, vertex in self.tree.vertices.items():
            self.scatters[vertex_id].set_offsets([vertex.state[0], vertex.state[1]])
            for edge_id, edge in self.tree.edges.items():
                if edge.sid == vertex_id:
                    start_vertex = self.tree.vertices[edge.sid]
                    end_vertex = self.tree.vertices[edge_id]
                    self.lines[edge_id].set_xdata([start_vertex.state[0], end_vertex.state[0]])
                    self.lines[edge_id].set_ydata([start_vertex.state[1], end_vertex.state[1]])

    def animate(self):
        ani = animation.FuncAnimation(self.fig, self.update_plot, frames=100, interval=100)
        plt.show(block=False)

