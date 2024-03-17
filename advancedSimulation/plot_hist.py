import pandas as pd
import matplotlib.pyplot as plt

# Read the CSV file
df = pd.read_csv('random_nodes.csv')

# Separate x and y values
x_values = df['x']
y_values = df['y']

# Plot histogram for x values
plt.hist(x_values, bins=800, color='blue', alpha=0.7)
plt.title('Histogram of x values')
plt.xlabel('x')
plt.ylabel('Frequency')
plt.grid(True)
plt.show()

# Plot histogram for y values
plt.hist(y_values, bins=600, color='green', alpha=0.7)
plt.title('Histogram of y values')
plt.xlabel('y')
plt.ylabel('Frequency')
plt.grid(True)
plt.show()