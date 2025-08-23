import pygame
import pgeng
import random
from config import *
from robot import Robot
from agent import Agent
import time
import imageio ## DEBUG 
import datetime ## DEBUG
import csv ## DEBUG

def main():
    ## DEBUG record flag and recorder for documentation
    record = True
    if record is True:
        now = datetime.datetime.now()
        # produce strings like "10/05/25" and "14-30-05"
        date_str = now.strftime("%d/%m/%y")
        time_str = now.strftime("%H-%M-%S")
        # filenames can’t contain "/", so swap to "-"
        safe_date = date_str.replace("/", "-")
        filename = f"logs/records/{safe_date}_{time_str}.mp4"
        writer = imageio.get_writer(filename, fps=FPS)
    else:
        writer = None

    # Create the display
    pygame.init()
    screen = pygame.display.set_mode((WIDTH, HEIGHT))
    pygame.display.set_caption("Simulation")
    clock = pygame.time.Clock()
    paused = False  # Add paused variable

    # Create walls
    top_wall = pygame.Rect(0, 0, WIDTH, WALL_THICKNESS)
    left_wall = pygame.Rect(0, 0, WALL_THICKNESS, HEIGHT)
    bottom_wall = pygame.Rect(0, HEIGHT - WALL_THICKNESS, WIDTH, WALL_THICKNESS)
    right_wall = pygame.Rect(WIDTH - WALL_THICKNESS, 0, WALL_THICKNESS, HEIGHT)
    mid_wall = pygame.Rect(0,(HEIGHT+WALL_THICKNESS)/2,WIDTH*3/4,WALL_THICKNESS)

    map = []
    ## choose layout by commenting out all others ##
    if MAP == "empty":
        map = [top_wall, left_wall, bottom_wall, right_wall] # empty room layout
    elif MAP == "midwall":
        map = [top_wall, left_wall, bottom_wall, right_wall,mid_wall] # wall in the middle layout
    ## ------------------------------------------ ##

    goal, robot, path, agents = generate_game(map)
    ## DEBUG statistics variables ##
    timer = time.time()
    round_timer = timer
    goals = 1
    score = 0
    collisions = 0
    ## ---------------- ##

    # Main loop
    running = True
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_p:  # Pause/resume with 'P' key
                    paused = not paused
            # keyboard teleop
            # elif event.type == pygame.KEYDOWN:
            #     if event.key == pygame.K_UP or event.key == pygame.K_w:
            #         robot.move(0, -TRANSLATION_UNIT)
            #     elif event.key == pygame.K_LEFT or event.key == pygame.K_a:
            #         robot.rotate(ROTATION_ANGLE)  # Rotate clockwise
            #     elif event.key == pygame.K_RIGHT or event.key == pygame.K_d:
            #         robot.rotate(-ROTATION_ANGLE)  # Rotate counter-clockwise

        if not paused:  # Only update the simulation if not paused
            # Clear the screen
            screen.fill(WHITE)

            ## DEBUG draw local search radius
            pygame.draw.circle(screen,PURPLE,(int(robot.x), int(robot.y)), SEARCH_RADIUS)
            pygame.draw.circle(screen,WHITE,(int(robot.x), int(robot.y)), SEARCH_RADIUS-2)

            # Draw the walls
            for wall in map:
                pygame.draw.rect(screen, BLACK, wall)

            # Draw the target
            pygame.draw.circle(screen, RED, goal, GOAL_TOLERACE)
            pygame.draw.circle(screen, WHITE, goal, GOAL_TOLERACE-2)

            # Draw the agents
            for agent in agents:
                agent.set_environment_data(agents, robot)
                agent.move()
                agent.draw(screen)

            # Draw the robot
            robot.set_environment_data(agents,map)
            if robot.move() == -1:
               return -1 
                # check if robot is within bounds
            if robot.x > WIDTH or robot.x < 0 or robot.y > HEIGHT or robot.y < 0:
                goal, robot, path, agents = generate_game(map) # reset game due to bug. don't count for any metric
            robot.draw(screen)
            if robot.goal_check([robot.x,robot.y],robot.goal):
                score += 1
                goals += 1
                goal = generate_goal(map)
                robot.goal = goal
                robot.goal_reached = True
            col = collision_check(robot, agents, map)
            if col > 0 and col < 10: ## robot collided with agent
                collisions += 1
                goals += 1
                goal, robot, path, agents = generate_game(map)
            
            # Draw global plan
            for point in robot.get_global_path_points():
                pygame.draw.circle(screen, RED, (int(point[0]), int(point[1])), 3)
            if robot.local_goal is not None:
                pygame.draw.circle(screen, GREEN, (int(robot.local_goal[0]), int(robot.local_goal[1])), 3)

            # Draw the robot's path
            path.append(pgeng.Circle((robot.x,robot.y),2,BLUE))
            for i in path:
                i.render(screen)

            # DEBUG video‐capture snippet   
            if record:
                frame = pygame.surfarray.array3d(screen)            # grab the pixels
                frame = frame.transpose((1, 0, 2))                  # to (h, w, 3)
                writer.append_data(frame)                           # write to MP4
            # ----------------------------- #

        # Update the display
        pygame.display.flip()

        # Cap the frame rate
        clock.tick(FPS)
        if (time.time() - round_timer) >= ROUND_TIME:
            goals += 1
            goal, robot, path, agents = generate_game(map)
            round_timer = time.time()
        if (time.time() - timer) >= GAME_TIME:
            print(f"time up! {goals} goals generated, {score} goals reached, {collisions} collisions occured")
            running = False

    if record is True:
        writer.close()
    # Quit Pygame
    pygame.quit()

def collision_check(robot, agents, map):
    '''
    check for collisions between each element on the screen.
    collision codes:
    no collision: 0
    robot -> wall: -(wall number)
    robot -> agent: (agent number)
    agent -> wall: 10*(agent number) + (wall number)
    agent -> agent 100*(first agent) + (second agent)


    assuming there are less than 10 agents, this creates a unique collision code.
    TODO - transfer prints to main, or delete.
    '''
    i=0
    for wall in map:
        i += 1
        if robot.poly.colliderect(wall):
            # print("robot -> wall collision")
            return -i
    
    i=0
    for agent in agents:
        i += 1
        if robot.poly.collidecircle(agent.poly):
            # print("robot -> agent collision")
            return i

    i=0 
     
    for wall in map:
        i += 1
        j=0  
        for agent in agents:
            j += 1
            if agent.poly.colliderect(wall):
                # print("agent -> wall collision")
                return 10*j+i
    i=0
    for agent1 in agents:
        i += 1
        j=0
        for agent2 in agents:
            j += 1
            if i!=j and agent1.poly.collide(agent2.poly):
                # print(f"agent{i} -> agent{j} collision")
                return 100*i+j
    return 0

def generate_goal(map):
    while True:
        goal_x = random.randint(WALL_THICKNESS + GOAL_CLEARANCE, WIDTH - WALL_THICKNESS - GOAL_CLEARANCE - 1)
        goal_y = random.randint(WALL_THICKNESS + GOAL_CLEARANCE, HEIGHT - WALL_THICKNESS - GOAL_CLEARANCE - 1)
        too_close = False
        for wall in map:
            x_min = wall.left - GOAL_CLEARANCE
            x_max = wall.right + GOAL_CLEARANCE
            y_min = wall.top - GOAL_CLEARANCE
            y_max = wall.bottom + GOAL_CLEARANCE
            if (x_min <= goal_x <= x_max) and (y_min <= goal_y <= y_max):
                too_close = True
                break
        if not too_close:
            break
    return [goal_x, goal_y]

def generate_game(map):
    # Create robot goal instance
    goal = generate_goal(map)

    # Create a robot instance
    robot = Robot((WIDTH - ROBOT_WIDTH) // 2, 
                HEIGHT - ROBOT_LENGTH - WALL_THICKNESS, 
                ROBOT_WIDTH, 
                ROBOT_LENGTH, goal= goal)
    
    path = [pgeng.Circle((robot.x,robot.y),2,BLUE)] ## DEBUG path printing
    # Generate random positions for agents
    colors = [RED, GREEN, PURPLE, BROWN, ORANGE, WHITE, RED, GREEN]
    # color_names= ["RED",  "GREEN", "PURPLE","BROWN",  "ORANGE","WHITE",  "RED2",  "GREEN2"]

    agents = []
    for i in range(AGENT_NUM):  # Create N agents
        color = colors[i]
        name = i
        radius = AGENT_RADIUS
        x = random.randint(WALL_THICKNESS + AGENT_RADIUS, 
                        WIDTH - WALL_THICKNESS - AGENT_RADIUS)
        y = random.randint(WALL_THICKNESS + AGENT_RADIUS, 
                        HEIGHT - WALL_THICKNESS - AGENT_RADIUS)
        agent = Agent(name, x, y, color, radius, map)
        while collision_check_agent(agent, robot, agents, map) > 0: # TODO FIX COLLISION CHECK WITH AGENT-ORIENTED LOGIC
            x = random.randint(WALL_THICKNESS + AGENT_RADIUS, 
                        WIDTH - WALL_THICKNESS - AGENT_RADIUS)
            y = random.randint(WALL_THICKNESS + AGENT_RADIUS, 
                        HEIGHT - WALL_THICKNESS - AGENT_RADIUS)
            agent = Agent(name, x, y, color, radius, map)
        agents.append(agent)
    # validate agent's starting locations
    return goal, robot, path, agents

def collision_check_agent(agent, robot, agents, map):
    # robot <-> agent collision
    if robot.poly.collidecircle(agent.poly):
        return 1
    # map <-> agent collision
    for i, wall in enumerate(map):
        if agent.poly.colliderect(wall):
            return i
    # agent <-> agent collision
    # if agents > 1:
    for target in agents:
        if target.name == agent.name:
            pass
        else:
            if agent.poly.collide(target.poly):
                return target.name
    return 0

if __name__ == "__main__":
    # profiler = cProfile.Profile()
    # profiler.enable()  # Start profiling
    main()  # Run the main function
    # profiler.disable()  # Stop profiling
    # profiler.dump_stats("profile.prof")  # Save the profile data to profile.prof