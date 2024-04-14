import csv
import matplotlib.pyplot as plt
import sys

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

def plot_graph(nodes, edges):
    """
    Plot the graph.

    Args:
        nodes (list): List of lists containing vertex data.
        edges (list): List of lists containing edge data.

    Returns:
        None
    """
    fig, ax = plt.subplots()

    # Plot vertices
    for idx, node in enumerate(nodes):
        x, y = float(node[1]), float(node[2])
        if idx == 0:
            ax.plot(x, y, 'go')  # green circle marker for the first node
        elif idx == len(nodes) - 1:
            ax.plot(x, y, 'ro')  # red circle marker for the last node
        else:
            ax.plot(x, y, 'bo')  # blue circle marker for other nodes

    # Plot edges
    for edge in edges:
        print(edge)
        eid_x, eid_y = find_xy(edge[0], nodes)
        sid_x, sid_y = find_xy(edge[1], nodes)
        print(f"eid ({eid_x},{eid_y}), sid ({sid_x},{sid_y})")
        ax.plot([sid_x, eid_x], [sid_y, eid_y], 'k-')  # black line

    # Set plot limits and labels
    ax.set_xlim([0, 800])
    ax.set_ylim([600, 0])
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_title('Graph Visualization')

    plt.show()

def find_xy(vid, nodes):
    for node in nodes:
        if node[0] == vid:
            return node[1], node[2]
    return -1,-1


if __name__ == "__main__":
    if len(sys.argv) != 2:
        print("Usage: python script.py <i>")
        sys.exit(1)

    i = sys.argv[1]

    # Read data from CSV files
    # nodes_data = read_csv(f'tree_nodes_{i}.csv')
    # edges_data = read_csv(f'tree_edges_{i}.csv')

    nodes_data = read_csv(f'logs/tree_nodes.csv')
    edges_data = read_csv(f'logs/tree_edges.csv')
    # Plot the graph
    plot_graph(nodes_data, edges_data)
