import numpy as np

state_1 = [3,4,0.7]
state_2 = [6,8,0.7]
cost_1 = 1.0
cost_2 = 1.1
vector_1 = np.array([state_1[0],state_1[1],cost_1])
vector_2 = np.array([state_2[0],state_2[1],cost_2])
print(np.linalg.norm(vector_2 - vector_1))