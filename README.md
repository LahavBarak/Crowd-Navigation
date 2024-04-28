# Crowd-Navigation
Crowd navigation algorithm, BSc project

# Basic Simulation
Basic_simulation folder contains files for a basic, python based (not ros), grid navigation.
The sim contains a single controllable robot, and two mobile agents occupy a grid map (map is cyclic for now). 
Each agent, and the robot, may move 1 or 0 steps each turn, in either direction, on X and Y axis independently.
The agents movement is random, and dictated by the following rules:
1. A move scheme is comprised of random velocity in each axis (-1,0 or 1) and random duration (in turns)
2. Each turn, each agent moves according to its velocity in each axis, and duration decreases by 1.
3. If duration is 0 - generate a new move scheme.
This is done in order to (very roughly) simulate human walking behavior by assuming partially-contiuous, straight line motion.
The robot is given a random goal position, and using KinoRRT, it plans its motion, such that it will avoid collision with the agents.
When the goal is reached - a new one is generated.
## How to use
To launch - clone folder and run 
```
python3 simulation.py
```
Simulation timestep is changeable by altering SIM_TIMESTEP parameter in simulation.py file. 
Too short times might cause the algorithm to bottleneck the turn order, but any number above 0.1 should be fine

# Advanced Simulation
Advanced Simulation folder contains files for an advanced, python based navigation.
The sim contains a 600x800 pixel board, with a rectangular robot and circular agents.
agents move randomly about the map (elaborate, social forces model TBD), 
while the rectancular robot uses motion planning and physics model to plan its motion to a random goal.
different motion models will be held in different branches of this git.
currently KinodynamicRRT is implemented in its own branch, and AO-RRT is WIP in the main branch.
for now, motion model is differential (Ackermann TBD) and disregarding agents (accounting for them TBD)

## How to use
To launch - clone folder and run 
```
python3 main.py
```
required repositories in requirements.txt

Simulation parameters changeable in config.py file.
main parameters: 
- FPS controls simulation timestep
- HEIGHT,WIDTH control window size (and therefore navigation space complexity)

It is possible to view a plot of RRT Graph by using
```
python3 plot_graph.py <goal node id>
```
goal node id is given as a print by the simulation, and needs to be put in manually

# Algorithm
As previously stated, the algorithm for robot motion is based off KinoRRT, with additional collision check logic to incorporate the mobile nature of the agents.
Assumptions made for the algorithm:
1. The poses both robot and agents are known precicely (through odometry, measurement, etc...)
2. The velocities of each agent are known, rather than inferred (unrealistic - this assumption will be removed after the most basic version of the algorithm is implemented)
3. The agents act independently of the robot, and the robot can only control itself.

One important thing to note - the algorithm does not consider the agents a part of the state space, but rather, considers them 'stationary obstacles that move'
