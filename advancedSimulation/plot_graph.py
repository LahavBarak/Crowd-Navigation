import csv
import matplotlib.pyplot as plt

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
    for node in nodes:
        x, y = float(node[1]), float(node[2])
        ax.plot(x, y, 'bo')  # blue circle marker

        # Annotate vertices with their vertex number
        ax.text(x, y, f'{node[0]}', fontsize=10, ha='center', va='center')

    # Plot edges
    for edge in edges:
        start_node = [node for node in nodes if node[0] == edge[0]]
        end_node = [node for node in nodes if node[0] == edge[1]]

        if start_node and end_node:  # Check if both start and end nodes exist
            start_x, start_y = float(start_node[0][1]), float(start_node[0][2])
            end_x, end_y = float(end_node[0][1]), float(end_node[0][2])
            ax.plot([start_x, end_x], [start_y, end_y], 'k-')  # black line

    # Set plot limits and labels
    ax.set_xlim([0, 800])
    ax.set_ylim([0, 600])
    ax.set_xlabel('X')
    ax.set_ylabel('Y')
    ax.set_title('Graph Visualization')

    plt.show()
    
# Read data from CSV files
nodes_data = read_csv('tree_nodes.csv')
edges_data = read_csv('tree_edges.csv')

# Plot the graph
plot_graph(nodes_data, edges_data)
