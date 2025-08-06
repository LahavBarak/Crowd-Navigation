import random
import math
import networkx as nx
import matplotlib.pyplot as plt
import pygame
import utils
from config import *

import json
import time ## DEBUG, data collection


class PRMGraph(object):
    def __init__(self, filepath=None, num_samples=None, x_max=None, y_max=None, walls=None, radius=PRM_RADIUS):
        self.graph = nx.Graph()
        self.walls = walls
        self.radius = radius
        self.nodes = {}
        
        # If parameters are provided, build the graph immediately
        if filepath is not None:
            self.walls = walls
            self.radius = radius
            self.read(filepath)
        elif num_samples is not None and x_max is not None and y_max is not None and walls is not None:
            self.build_graph(num_samples, x_max, y_max, walls)

    def inflate_obstacles(self):
        """
        Expand each wall by half the robot’s diagonal, so that
        we can treat the robot as a point in the inflated map.
        """
        # half the diagonal of the robot rectangle
        half_diag = math.hypot(ROBOT_LENGTH, ROBOT_WIDTH) / 2
        inflated = []
        for w in self.walls:
            # create a new Rect whose center is the same, but
            # width/height are increased by 2*half_diag
            # pygame.Rect.inflate grows around the center
            inflated_rect = w.inflate(2 * half_diag, 2 * half_diag)
            inflated.append(inflated_rect)
        self.walls = inflated
    
    def build_graph(self, num_samples, x_max, y_max, walls=None):
        x_range = (0,x_max)
        y_range = (0,y_max)

        if walls is not None:
            self.walls = walls
        self.inflate_obstacles()

        for i in range(num_samples):
            if i%10000 == 0:
                print(f"{i}th node built. {100*i/num_samples}% complete")
            x = random.randint(*x_range)
            y = random.randint(*y_range)
            while self.find_node(x, y) != -1 or self.node_collision((x, y), self.walls):  # Ensure no duplicate nodes exist
                x = random.randint(*x_range)
                y = random.randint(*y_range)
            self.graph.add_node(i, x=x, y=y)
            self.nodes[(x, y)] = i  # Store node by its coordinates
    
        radius = self.radius
        coordinate_offsets = [(dx, dy) for dx in range(-radius, radius) for dy in range(-radius, radius)]

        for (x0, y0), i in self.nodes.items():
            if i%10000 == 0:
                print(f"{i}th node connected. {100*i/num_samples}% complete")
            potential_neighbors = [
                (x0 + dx, y0 + dy) for (dx, dy) in coordinate_offsets if (x0 + dx, y0 + dy) in self.nodes
            ]

            for x, y in potential_neighbors:
                j = self.nodes[(x, y)]
                if i == j:
                    continue
                dist = math.hypot(x0 - x, y0 - y)
                if dist <= self.radius and not self.edge_collision({'x': x0, 'y': y0}, {'x': x, 'y': y}):
                    self.graph.add_edge(i, j, weight=round(dist, 2))

        return self.graph

    def is_empty(self):
        return len(self.graph.nodes) == 0
    
    def find_node(self, x_target, y_target):
        return self.nodes.get((x_target, y_target), -1)

    def node_collision(self, node, walls):
        x, y = node
        for wall in walls:
            x_min, y_min = wall.topleft
            x_max, y_max = wall.bottomright
            if (x <= x_max and x >= x_min) and (y <= y_max and y >= y_min):
                return True
        return False

    def edge_collision(self, node1, node2):
        for wall in self.walls:
            x_min, y_min = wall.topleft
            x_max, y_max = wall.bottomright

            edges = [
                ((x_min, y_min), (x_max, y_min)),
                ((x_min, y_max), (x_max, y_max)),
                ((x_min, y_min), (x_min, y_max)),
                ((x_max, y_min), (x_max, y_max))
            ]

            for e1, e2 in edges:
                if utils.line_segments_intersect((node1['x'], node1['y']), (node2['x'], node2['y']), e1, e2):
                    return True
        return False

    def add_node(self, node):
        x, y = node
        if self.find_node(x, y) != -1:
            return

        node_id = len(self.graph.nodes)
        self.graph.add_node(node_id, x=x, y=y)
        self.nodes[(x, y)] = node_id

        for (x_other, y_other), other_id in self.nodes.items():
            if other_id == node_id:
                continue
            dist = math.hypot(x - x_other, y - y_other)
            if dist <= self.radius and not self.edge_collision({'x': x, 'y': y}, {'x': x_other, 'y': y_other}):
                self.graph.add_edge(node_id, other_id, weight=round(dist, 2))

    def heuristic(self, node1, node2):
        return math.hypot(self.graph.nodes[node2]['x'] - self.graph.nodes[node1]['x'],
                           self.graph.nodes[node2]['y'] - self.graph.nodes[node1]['y'])

    def find_path(self, start, goal):
        self.add_node(start)
        self.add_node(goal)
        start_node = self.find_node(*start)
        goal_node = self.find_node(*goal)

        try:
            path = nx.astar_path(self.graph, start_node, goal_node, heuristic=self.heuristic, weight='weight')
            return path
        except nx.NetworkXNoPath:
            return None
        except nx.NodeNotFound:
            return None
    
    def write(self,filepath):
        ''' write PRM graph to file. filepath should end with .json
            does not write walls and radius, so keep in mind which map this was generated for'''
        data = {
            "nodes": [],
            "edges": []
        }
        # Collect node data
        for node_id, attr in self.graph.nodes(data=True):
            data["nodes"].append({
                "id": node_id,
                "x": attr["x"],
                "y": attr["y"]
            })

        # Collect edge data
        for u, v, attr in self.graph.edges(data=True):
            data["edges"].append({
                "u": u,
                "v": v,
                "weight": attr["weight"]
            })

        # Write JSON to file
        with open(filepath, "w") as f:
            json.dump(data, f, indent=2)

    def read(self, filepath, walls, radius=PRM_RADIUS):
        self.walls = walls
        self.radius = radius

        with open(filepath, "r") as f:
            data = json.load(f)

        # Clear any existing data
        self.graph.clear()
        self.nodes.clear()

        # Re-create all nodes
        for node_data in data["nodes"]:
            node_id = node_data["id"]
            x = node_data["x"]
            y = node_data["y"]
            self.graph.add_node(node_id, x=x, y=y)
            self.nodes[(x, y)] = node_id

        # Re-create all edges
        for edge_data in data["edges"]:
            u = edge_data["u"]
            v = edge_data["v"]
            weight = edge_data["weight"]
            self.graph.add_edge(u, v, weight=weight)

if __name__ == "__main__":

    # Create walls
    top_wall = pygame.Rect(0, 0, WIDTH, WALL_THICKNESS)
    left_wall = pygame.Rect(0, 0, WALL_THICKNESS, HEIGHT)
    bottom_wall = pygame.Rect(0, HEIGHT - WALL_THICKNESS, WIDTH, WALL_THICKNESS)
    right_wall = pygame.Rect(WIDTH - WALL_THICKNESS, 0, WALL_THICKNESS, HEIGHT)
    mid_wall = pygame.Rect(0,(HEIGHT+WALL_THICKNESS)/2,WIDTH*3/4,WALL_THICKNESS)

    ## choose layout by commenting out all others ##
    map = [top_wall, left_wall, bottom_wall, right_wall] # wall in the middle layout

    num_samples = 100000
    x_max = 1920
    y_max = 1080
    radius = 15
    t0 = time.time() ## DEBUG profiling
    G = PRMGraph()
    print(f"time to load graph {time.time()-t0}")
    G.build_graph(num_samples, x_max, y_max, map)
    print(f"time to build graph {time.time()-t0}")
    G.write(f"PRM_graphs/empty_{num_samples}_samples")
    start = (random.randint(50, 1000), random.randint(50, 400))
    goal  = (random.randint(50, 1000), random.randint(500, 850))
    path  = G.find_path(start, goal)
    print(f"time to load + find path {time.time()-t0}")


    if path:
        # extract coords & compute per‐waypoint distances
        coords   = [(G.graph.nodes[n]['x'], G.graph.nodes[n]['y']) for n in path]

    pos = {n: (G.graph.nodes[n]['x'], G.graph.nodes[n]['y']) for n in path}
    path_edges = list(zip(path, path[1:]))

    plt.figure(figsize=(8, 6))

    # Draw walls
    for wall in map:
        plt.gca().add_patch(plt.Rectangle(wall.topleft, wall.width, wall.height, color='black'))

    # Draw path
    nx.draw(G.graph.subgraph(path), pos, with_labels=False, node_color='lightblue', edge_color='green', node_size=5)

    plt.title("PRM Graph with A* Path and Walls")
    plt.xlim((0,x_max))
    plt.ylim((0,y_max))
    plt.show()

