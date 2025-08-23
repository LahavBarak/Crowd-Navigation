# Crowd-Navigation
Crowd navigation algorithm, BSc project

# Simulation
Simulation folder contains files for an advanced, python based navigation.
The sim contains a simple map, with a rectangular robot and circular agents.
agents move randomly about the map, 
while the rectancular robot uses motion planning and physics model to plan its motion to a random goal.

## How to use
go to simulation folder
install required packages by 
```
pip install -r requirements.txt
```
run code by
```
python3 main.py
```

Simulation parameters changeable in config.py file.
main parameters: 
- FPS controls simulation timestep
- HEIGHT,WIDTH control window size (and therefore navigation space complexity)
- PRM map file (if saved)


# Algorithm
As previously stated, the algorithm for robot motion is based off KinoRRT, with additional collision check logic to incorporate the mobile nature of the agents.
Assumptions made for the algorithm:
1. The poses both robot and agents are known precicely (through odometry, measurement, etc...)
2. The velocities of each agent are known, rather than inferred (unrealistic - this assumption will be removed after the most basic version of the algorithm is implemented)
3. The agents act independently of the robot, and the robot can only control itself.

One important thing to note - the algorithm does not consider the agents a part of the state space, but rather, considers them 'stationary obstacles that move'
