import pygame
import pgeng
import random
from config import *
from robot import Robot
from agent import Agent
import csv
import time

def main():
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

    ## choose layout by commenting out all others ##
    map = [top_wall, left_wall, bottom_wall, right_wall] # empty room layout
    # map = [top_wall, left_wall, bottom_wall, right_wall,mid_wall] # wall in the middle layout
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

            # Draw the walls
            for wall in map:
                pygame.draw.rect(screen, BLACK, wall)

            # Draw the target
            pygame.draw.circle(screen, RED, goal, GOAL_TOLERACE)
            pygame.draw.circle(screen, WHITE, goal, GOAL_TOLERACE-2)

            # Draw the agents
            i=0
            for agent in agents:
                i += 1
                collide = collision_check(robot, agents, map)
                if (collide == i or 10*i <= collide < 10*(i+1) or 
                    100*i <= collide < 100*(i+1)): # codes for this agent being involved in collision
                    collide = True
                agent.move(collide)
                agent.draw(screen)

            # Draw the robot
            robot.set_environment_data(agents,map)
            robot.move()
            robot.draw(screen)
            if robot.goal_check([robot.x,robot.y],robot.goal):
                score += 1
                goals += 1
                goal = [random.randint(NO_COLLISION_RANGE,WIDTH-NO_COLLISION_RANGE-1),
                        random.randint(NO_COLLISION_RANGE,HEIGHT-NO_COLLISION_RANGE-1)]
                robot.goal = goal
            col = collision_check(robot, agents, map)
            if col > 0 and col < 10: ## robot collided with agent
                print("collision")
                collisions += 1
                goals += 1
                goal, robot, path, agents = generate_game(map)
            
            # Draw the robot's path
            path.append(pgeng.Circle((robot.x,robot.y),2,BLUE))
            for i in path:
                i.render(screen)
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
        
def generate_game(map):
    collision = 1
    # Create robot goal instance
    # ## random goal ##
    goal = [random.randint(NO_COLLISION_RANGE,WIDTH-NO_COLLISION_RANGE-1),
            random.randint(NO_COLLISION_RANGE,HEIGHT-NO_COLLISION_RANGE-1)]
    # ## fixed goal, DEBUG ##
    # goal = [200, 150]
    # Create a robot instance
    robot = Robot((WIDTH - ROBOT_WIDTH) // 2, 
                HEIGHT - ROBOT_LENGTH - WALL_THICKNESS, 
                ROBOT_WIDTH, 
                ROBOT_LENGTH, goal= goal)
    
    path = [pgeng.Circle((robot.x,robot.y),2,BLUE)] ## DEBUG path printing
    # Generate random positions for agents
    colors = [RED, GREEN, PURPLE, BROWN, ORANGE]
    agents = []

    while collision > 0: # meaning there's a collision between agent and something else
        agents = []
        for i in range(AGENT_NUM):  # Create N agents
            x = random.randint(WALL_THICKNESS + AGENT_RADIUS, 
                            WIDTH - WALL_THICKNESS - AGENT_RADIUS)
            y = random.randint(WALL_THICKNESS + AGENT_RADIUS, 
                            HEIGHT - WALL_THICKNESS - AGENT_RADIUS)

            # x = x0[i] + 100 ## DEBUG fixed agent positions
            # y = y0[i] + 50 ## DEBUG fixed agent positions
            color = colors[i]
            radius = AGENT_RADIUS
            agent = Agent(x, y, color, radius)
            agents.append(agent)

        # validate agent's starting locations
        collision = collision_check(robot, agents, map)
    
    return goal, robot, path, agents

if __name__ == "__main__":
    main()


