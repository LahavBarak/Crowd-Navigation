import tkinter as tk
import time
import random
import agent_logic
import bot_logic


## global grid parameters ##
CELLSIZE = 40
GRIDSIZE = 10 
BOARDSIZE = CELLSIZE * GRIDSIZE
SIM_TIMESTEP = 0.5 # in seconds

## helper class definitions ##
class Element:
    def __init__(self, canvas, col, row, color, size):
        self.canvas = canvas
        self.col = col
        self.row = row
        self.color = color
        self.size = size
        self.shape = None

    def draw(self):
        pass

    def move(self, new_col, new_row):
        self.col = new_col
        self.row = new_row
        self.canvas.delete(self.shape)
        self.draw()

class Triangle(Element):
    def draw(self):
        x0, y0 = self.col * self.size, (self.row + 1) * self.size
        x1, y1 = x0 + self.size, (self.row + 1) * self.size
        x2, y2 = x0 + self.size / 2, self.row * self.size
        self.shape = self.canvas.create_polygon(x0, y0, x1, y1, x2, y2, fill=self.color, outline="black")

class Circle(Element):
    def draw(self):
        x0, y0 = self.col * self.size, self.row * self.size
        x1, y1 = x0 + self.size, y0 + self.size
        self.shape = self.canvas.create_oval(x0, y0, x1, y1, fill=self.color, outline="black")

class Agent():
    def __init__(self):
        self.shape = None
        self.move_duration = 0
        self.speed_x = 0
        self.speed_y = 0
        self.position_x = random.randint(0,GRIDSIZE-1)
        self.position_y = random.randint(0,GRIDSIZE-1)

class Bot():
    def __init__(self):

        self.shape = None
        # define state space
        ## self states
        self.self_pos_x = random.randint(0,GRIDSIZE-1)
        self.self_pos_y = random.randint(0,GRIDSIZE-1)
        self.self_vel_x = 0
        self.self_vel_y = 0
        self.duration = 0
        self.goal = None
        ## states of 2 agents, with memory of 5 states.
        self.agents_pos_x = [0 ,0] 
        self.agents_pos_y = [0 ,0]
        self.agents_vel_x = [0 ,0]
        self.agents_vel_y = [0 ,0]

## ----- main class ----- ##
class GridGUI(tk.Tk):
    ## ---- initialization ---- ##
    def __init__(self):
        super().__init__()

        self.title("Grid with Elements")
        self.geometry_string = f"{BOARDSIZE}x{BOARDSIZE}"
        self.geometry(self.geometry_string)

        self.gridSize = GRIDSIZE
        self.cellSize = CELLSIZE

        self.canvas = tk.Canvas(self, width=self.gridSize * self.cellSize, height=self.gridSize * self.cellSize)
        self.canvas.pack()

        self.draw_grid()
        self.place_elements()
        self.run_sim()

    def draw_grid(self):
        for i in range(self.gridSize + 1):
            x = i * self.cellSize
            self.canvas.create_line(x, 0, x, self.gridSize * self.cellSize, fill="black")
            y = i * self.cellSize
            self.canvas.create_line(0, y, self.gridSize * self.cellSize, y, fill="black")
            # Add coordinates to the grid at fixed positions
            if i < self.gridSize:
                self.canvas.create_text(x + 20, self.gridSize * self.cellSize + 20,
                                        text=str(i), font=("Helvetica", 8))
                self.canvas.create_text(self.gridSize * self.cellSize + 20, y + 20,
                                        text=str(i), font=("Helvetica", 8))

    def place_elements(self):
        self.agent1 = Agent()
        self.agent1.shape = Circle(self.canvas, self.agent1.position_x, 
                                   self.agent1.position_y, "brown", self.cellSize)  
        self.agent2 = Agent()
        self.agent2.shape = Circle(self.canvas, self.agent2.position_x, 
                                   self.agent2.position_y, "green", self.cellSize)  
        self.bot = Bot()
        self.bot.shape = Triangle(self.canvas, self.bot.self_pos_x, self.bot.self_pos_y, "blue", self.cellSize)   
        self.agent1.shape.draw()
        self.bot.shape.draw()
        self.agent2.shape.draw()
        self.update()

    def draw_goal(self, x, y):
        goal_size = self.cellSize / 2
        x_center = x * self.cellSize + goal_size
        y_center = y * self.cellSize + goal_size

        # Delete any existing goal items with the "goal" tag
        self.canvas.delete("goal")

        # Draw the new goal
        self.canvas.create_oval(
            x_center - goal_size, y_center - goal_size,
            x_center + goal_size, y_center + goal_size,
            outline="red", width=2, tags="goal"
        )

    ## ---- operation ---- ##
    def run_sim(self):
        for i in range(100): # Run simulation for 100 turns
            # update bot's measurements 
            self.bot.agents_pos_x = [self.agent1.position_x, self.agent2.position_x]
            self.bot.agents_pos_y = [self.agent1.position_y, self.agent2.position_y]
            
            # If previous move plan ended - plan a new one
            if (self.agent1.move_duration == 0): 
                self.agent_planner(self.agent1) 
            if (self.agent2.move_duration == 0): 
                self.agent_planner(self.agent2) 

            # cheat and update a-priori agent's move plans
            self.bot.agents_vel_x = [self.agent1.speed_x, self.agent2.speed_x]
            self.bot.agents_vel_y = [self.agent1.speed_y, self.agent2.speed_y]

            if (self.bot.goal is not None and bot_logic.goal_check(
                [self.bot.self_pos_x,self.bot.self_pos_y],self.bot.goal) is True):
                print("GOAL REACHED! calculating new goal")
                self.bot.goal = None
            if (self.bot.goal == None):
                self.bot.goal = [random.randint(0,GRIDSIZE-1),random.randint(0,GRIDSIZE-1)]
                self.draw_goal(*self.bot.goal)
                print(f"NEW GOAL: {self.bot.goal}")

            if (self.bot.duration == 0):
                print(f"self_pos_1 = ({self.bot.self_pos_x},{self.bot.self_pos_y})")                
                edge = bot_logic.bot_logic(self, self.bot.goal)
                self.bot.self_vel_x = edge.u[0]
                self.bot.self_vel_y = edge.u[1]
                self.bot.duration = edge.t
                print(f"vel = ({self.bot.self_vel_x},{self.bot.self_vel_y}), dur = {self.bot.duration}")
                
            
            # Update new positions
            time.sleep(SIM_TIMESTEP)
            self.agent1.position_x = (self.agent1.position_x + self.agent1.speed_x) % GRIDSIZE 
            self.agent1.position_y = (self.agent1.position_y + self.agent1.speed_y) % GRIDSIZE
            self.agent2.position_x = (self.agent2.position_x + self.agent2.speed_x) % GRIDSIZE
            self.agent2.position_y = (self.agent2.position_y + self.agent2.speed_y) % GRIDSIZE
            self.bot.self_pos_x = (self.bot.self_pos_x + self.bot.self_vel_x) % GRIDSIZE
            self.bot.self_pos_y = (self.bot.self_pos_y + self.bot.self_vel_y) % GRIDSIZE


            self.move_agent1(self.agent1.position_x, self.agent1.position_y)
            self.move_agent2(self.agent2.position_x, self.agent2.position_y)

            self.move_bot(self.bot.self_pos_x,self.bot.self_pos_y)
            print(f"self_pos_new = ({self.bot.self_pos_x},{self.bot.self_pos_y})")

            ## -- check for collisions between bot and agents -- ##
            if ((self.agent1.position_x == self.bot.self_pos_x
                and self.agent1.position_y == self.bot.self_pos_y)
                or (self.agent2.position_x == self.bot.self_pos_x
                and self.agent2.position_y == self.bot.self_pos_y)):
                print("Collision detected! Bot failed!")
                print(f"final bot position = ({self.bot.self_pos_x},{self.bot.self_pos_y})")
                break
            self.agent1.move_duration = self.agent1.move_duration - 1
            self.agent2.move_duration = self.agent2.move_duration - 1
            self.bot.duration = self.bot.duration - 1
            # print(f'dur={self.agent1.move_duration},pos=({self.agent1.position_x},{self.agent1.position_y}),spd=({self.agent1.speed_x},{self.agent1.speed_y})')
            # print(f'dur={self.agent2.move_duration},pos=({self.agent2.position_x},{self.agent2.position_y}),spd=({self.agent2.speed_x},{self.agent2.speed_y})')

    def move_bot(self, x:int, y:int):
        self.bot.shape.move(x, y)  # Move the bot to a new position
        self.update()
        # You can add more logic for movement as needed

    def move_agent1(self, x:int, y:int):
        self.agent1.shape.move(x, y)  # Move the bot to a new position
        self.update()

    def move_agent2(self, x:int, y:int):
        self.agent2.shape.move(x, y)  # Move the bot to a new position
        self.update()

    def agent_planner(self, agent: Agent):
        MIN_DURATION = 5
        MAX_DURATION = 10
        MAX_VELOCITY = 1
        
        agent.move_duration = random.randint(MIN_DURATION,MAX_DURATION)
        agent.speed_x = random.randint(-MAX_VELOCITY,MAX_VELOCITY)
        agent.speed_y = random.randint(-MAX_VELOCITY,MAX_VELOCITY)

if __name__ == "__main__":
    app = GridGUI()
    app.mainloop()
    
