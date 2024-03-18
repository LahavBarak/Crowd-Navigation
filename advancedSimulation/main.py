import pygame
import pgeng
import random
import math
from config import *
from robot import Robot
from agent import Agent

def main():
    # Create the display
    pygame.init()
    screen = pygame.display.set_mode((WIDTH, HEIGHT))
    pygame.display.set_caption("Simulation")
    clock = pygame.time.Clock()
    collision = 1

    # Create walls
    top_wall = pygame.Rect(0, 0, WIDTH, WALL_THICKNESS)
    left_wall = pygame.Rect(0, 0, WALL_THICKNESS, HEIGHT)
    bottom_wall = pygame.Rect(0, HEIGHT - WALL_THICKNESS, WIDTH, WALL_THICKNESS)
    right_wall = pygame.Rect(WIDTH - WALL_THICKNESS, 0, WALL_THICKNESS, HEIGHT)
    walls = [top_wall, left_wall, bottom_wall, right_wall]

    # Create robot goal instance
    goal = [random.randint(NO_COLLISION_RANGE,WIDTH-NO_COLLISION_RANGE-1),
            random.randint(NO_COLLISION_RANGE,HEIGHT-NO_COLLISION_RANGE-1)]

    # Create a robot instance
    robot = Robot((WIDTH - ROBOT_WIDTH) // 2, 
                HEIGHT - ROBOT_LENGTH - WALL_THICKNESS, 
                ROBOT_WIDTH, 
                ROBOT_LENGTH, goal= goal)
    
    
    # Generate random positions for agents
    colors = [RED, GREEN]
    agents = []
    while collision > 0: # meaning there's a collision between agent and something else
        agents = []
        for i in range(AGENT_NUM):  # Create N agents
            x = random.randint(WALL_THICKNESS + AGENT_RADIUS, 
                            WIDTH - WALL_THICKNESS - AGENT_RADIUS)
            y = random.randint(WALL_THICKNESS + AGENT_RADIUS, 
                            HEIGHT - WALL_THICKNESS - AGENT_RADIUS)
            color = colors[i]
            radius = AGENT_RADIUS
            agent = Agent(x, y, color, radius)
            agents.append(agent)

        # validate agent's starting locations
        collision = collision_check(robot, agents, walls)

    # Main loop
    running = True
    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            # keyboard teleop
            # elif event.type == pygame.KEYDOWN:
            #     if event.key == pygame.K_UP or event.key == pygame.K_w:
            #         robot.move(0, -TRANSLATION_UNIT)
            #     elif event.key == pygame.K_LEFT or event.key == pygame.K_a:
            #         robot.rotate(ROTATION_ANGLE)  # Rotate clockwise
            #     elif event.key == pygame.K_RIGHT or event.key == pygame.K_d:
            #         robot.rotate(-ROTATION_ANGLE)  # Rotate counter-clockwise

        # Clear the screen
        screen.fill(WHITE)

        # Draw the walls
        pygame.draw.rect(screen, BLACK, top_wall)
        pygame.draw.rect(screen, BLACK, left_wall)
        pygame.draw.rect(screen, BLACK, bottom_wall)
        pygame.draw.rect(screen, BLACK, right_wall)

        # Draw the target
        pygame.draw.circle(screen, RED, goal, GOAL_TOLERACE)
        pygame.draw.circle(screen, WHITE, goal, GOAL_TOLERACE-2)
        
        # Draw the circles
        i=0
        for agent in agents:
            i += 1
            collide = collision_check(robot, agents, walls)
            if (collide == i or 10*i <= collide < 10*(i+1) or 
                100*i <= collide < 100*(i+1)): # codes for this agent being involved in collision
                collide = True
            agent.move(collide)
            agent.draw(screen)

        # Draw the robot
        robot.set_environment_data(agents,walls)
        robot.move()
        robot.draw(screen)
        if robot.goal_check([robot.x_cm,robot.y_cm],robot.goal):
            print("goal reached")
            goal = [random.randint(NO_COLLISION_RANGE,WIDTH-NO_COLLISION_RANGE-1),
                    random.randint(NO_COLLISION_RANGE,HEIGHT-NO_COLLISION_RANGE-1)]
            robot.goal = goal
        collision_check(robot, agents, walls)
        


        # Update the display
        pygame.display.flip()

        # Cap the frame rate
        clock.tick(FPS)

    # Quit Pygame
    pygame.quit()

def collision_check(robot, agents, walls):
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
    for wall in walls:
        i += 1
        if robot.poly.colliderect(wall):
            print("robot -> wall collision")
            return -i
    
    i=0
    for agent in agents:
        i += 1
        if robot.poly.collidecircle(agent.poly):
            print("robot -> agent collision")
            return i

    i=0 
     
    for wall in walls:
        i += 1
        j=0  
        for agent in agents:
            j += 1
            if agent.poly.colliderect(wall):
                print("agent -> wall collision")
                return 10*j+i
    i=0
    for agent1 in agents:
        i += 1
        j=0
        for agent2 in agents:
            j += 1
            if i!=j and agent1.poly.collide(agent2.poly):
                print(f"agent{i} -> agent{j} collision")
                return 100*i+j
    return 0
        

if __name__ == "__main__":
    main()