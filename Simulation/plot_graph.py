import csv
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from numpy import sin, cos
import sys
from config import *
from utils import *

def read_csv(filename):
    """
    Read data from a CSV file.

    Args:
        filename (str): Name of the CSV file to read.

    Returns:
        list: List of lists containing the data from the CSV file.
    """
    data = []
    with open(filename, 'r') as csvfile:
        csv_reader = csv.reader(csvfile)
        next(csv_reader)  # Skip the header row
        for row in csv_reader:
            data.append(row)
    return data

def plot_graph(nodes, edges, agents, title):
    """
    Plot the graph.

    Args:
        nodes (list): List of lists containing vertex data.
        edges (list): List of lists containing edge data.

    Returns:
        None
    """
    fig, ax = plt.subplots()

    # Plot agents path as predicted by robot
    colors = [RED, GREEN, PURPLE, BROWN, ORANGE]
    color_seen = []
    for row in agents:
        x, y = float(row[1]), float(row[2])
        color = tuple(c / 255 for c in colors[int(row[0])])
        circle = patches.Circle((x, y), radius=2, color=color)
        if (color in color_seen) is False: # print starting position larger
            circle = patches.Circle((x, y), radius=10, color=color)
            color_seen.append(color)
        
        ax.add_patch(circle)

    # Plot vertices
    for idx, node in enumerate(nodes):
        x, y, theta = float(node[1]), float(node[2]), float(node[3])
        if idx == 0:
            ax.plot(x, y, 'go')  # green circle marker for the first node
            v = get_vertices(x,y,theta, ROBOT_LENGTH, ROBOT_WIDTH)
            ax.plot([v[0][0], v[1][0]], [v[0][1], v[1][1]], 'k-')  # upper line
            ax.plot([v[1][0], v[2][0]], [v[1][1], v[2][1]], 'k-')  # bottom line
            ax.plot([v[2][0], v[3][0]], [v[2][1], v[3][1]], 'k-')  # right line
            ax.plot([v[0][0], v[3][0]], [v[0][1], v[3][1]], 'k-')  # left line
        elif idx == len(nodes) - 1:
            ax.plot(x, y, 'ro')  # red circle marker for the last node
        else:
            ax.plot(x, y, 'bo')  # blue circle marker for other nodes

    # Plot edges
    for edge in edges:
        eid_x, eid_y = find_xy(edge[0], nodes)
        sid_x, sid_y = find_xy(edge[1], nodes)
        ax.plot([eid_x, sid_x], [eid_y, sid_y], 'k-')  # black line

    # Plot goal region
    last_node = nodes[-1]
    x_goal, y_goal = float(last_node[1]), float(last_node[2])

    x_left = x_goal - GOAL_TOLERACE
    x_right = x_goal + GOAL_TOLERACE
    y_top = y_goal - GOAL_TOLERACE
    y_bot = y_goal + GOAL_TOLERACE

    ax.plot([x_left, x_right], [y_top, y_top], 'k-')  # upper line
    ax.plot([x_left, x_right], [y_bot, y_bot], 'k-')  # bottom line
    ax.plot([x_right, x_right], [y_top, y_bot], 'k-')  # right line
    ax.plot([x_left, x_left], [y_top, y_bot], 'k-')  # left line

    # Set plot limits and labels
    ax.set_xlim([0, 1200])
    ax.set_ylim([900, 0])
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_title(title)
    # ax.legend()  # Add legend

def find_xy(vid, nodes):
    for node in nodes:
        if node[0] == vid:
            return float(node[1]), float(node[2])
    return -1, -1

def find_path(eid, input_file, output_file="path.csv"):
    path = [eid]
    with open(input_file, mode='r') as infile:
        reader = csv.DictReader(infile)
        rows = {row['eid']: row for row in reader}

        current_eid = eid
        while current_eid != '0':
            if current_eid not in rows:
                print(f"Eid {current_eid} not found in the input file.")
                return
            if rows[current_eid]['sid'] == '0':
                break
            path.append(rows[current_eid]['sid'])
            current_eid = rows[current_eid]['sid']

    with open(output_file, mode='w', newline='') as outfile:
        writer = csv.DictWriter(outfile, fieldnames=reader.fieldnames)
        writer.writeheader()
        for eid in reversed(path):
            if eid in rows:
                writer.writerow(rows[eid])

        

if __name__ == "__main__":
    if len(sys.argv) != 3:
        print("Usage: python3 plot_graph.py <plot_id>, <final_node_id>")
        sys.exit(1)

    eid = str(sys.argv[2])
    plot_id = str(sys.argv[1])

    # Read data from CSV files
    nodes_data = read_csv(f'logs/nodes/tree_nodes_{plot_id}.csv')
    edges_data = read_csv(f'logs/edges/tree_edges_{plot_id}.csv')
    agents_data = read_csv(f'logs/collisions/collisions_{plot_id}.csv')

    # Plot the graph
    plot_graph(nodes_data, edges_data, agents_data, "Full RRT Graph")
    if eid != "-1":
        find_path(eid, f'logs/edges/tree_edges_{plot_id}.csv', f"logs/paths/path_{plot_id}.csv")
        path_data = read_csv(f'logs/paths/path_{plot_id}.csv')
        plot_graph(nodes_data, path_data, agents_data, "Path Chosen by Robot")
    plt.show()
