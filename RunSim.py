import pandas as pd
import SingleAgentEnvironment
import MultiAgentEnvironment
import random
import pygame

# Colors
BLUE = (0, 0, 255)
RED = (255, 0, 0)

# Constants
width = 800  # Width of the simulation window
height = 608  # Height of the simulation window

# Function to generate random goals

def is_point_valid(x, y, obstacles):
    for obstacle in obstacles:
        if ((x - obstacle.x) ** 2 + (y - obstacle.y) ** 2) <= (OBSTACLE_RADIUS ** 2)+20:
            return False
    return True


# Define Constant Goals, Starts, and Obstacles
OBSTACLE_RADIUS = 30  # Radius of the obstacles


agent_1 = SingleAgentEnvironment.Agent(100, 100)
agents_1 = [
    MultiAgentEnvironment.Agent(1, 100, 100),
    MultiAgentEnvironment.Agent(2, 100, 140),
    MultiAgentEnvironment.Agent(3, 100, 170)
]
agents_random = MultiAgentEnvironment.create_random_agents(80, 140, 100, 500, 1)
agents_rand_line = MultiAgentEnvironment.create_agent_line(100, int(random.uniform(100, 550)), 5)
agents_center_line_10 = MultiAgentEnvironment.create_agent_line(100, 300, 10)
agents_center_line_5 = MultiAgentEnvironment.create_agent_line(100, 300, 5)
agents_center_line_3 = MultiAgentEnvironment.create_agent_line(100, 300, 3)
diagnostics_agents = [agents_center_line_3, agents_center_line_5, agents_center_line_10]



obstacles_1 = [
    SingleAgentEnvironment.Obstacle(300, 200, OBSTACLE_RADIUS),
    SingleAgentEnvironment.Obstacle(400, 200, OBSTACLE_RADIUS),
    SingleAgentEnvironment.Obstacle(500, 200, OBSTACLE_RADIUS),
    SingleAgentEnvironment.Obstacle(600, 200, OBSTACLE_RADIUS),
    SingleAgentEnvironment.Obstacle(300, 400, OBSTACLE_RADIUS),
    SingleAgentEnvironment.Obstacle(400, 400, OBSTACLE_RADIUS),
    SingleAgentEnvironment.Obstacle(500, 400, OBSTACLE_RADIUS),
    SingleAgentEnvironment.Obstacle(600, 400, OBSTACLE_RADIUS),
]

obstacles_2 = [
    SingleAgentEnvironment.Obstacle(200, 300, OBSTACLE_RADIUS),
    SingleAgentEnvironment.Obstacle(400, 300, OBSTACLE_RADIUS),
    SingleAgentEnvironment.Obstacle(600, 300, OBSTACLE_RADIUS),
    SingleAgentEnvironment.Obstacle(400, 100, OBSTACLE_RADIUS),
    SingleAgentEnvironment.Obstacle(400, 200, OBSTACLE_RADIUS),
    SingleAgentEnvironment.Obstacle(400, 400, OBSTACLE_RADIUS),
    SingleAgentEnvironment.Obstacle(400, 500, OBSTACLE_RADIUS),
    SingleAgentEnvironment.Obstacle(300, 200, 40),
    SingleAgentEnvironment.Obstacle(300, 400, 60),
    SingleAgentEnvironment.Obstacle(500, 200, 60),
    SingleAgentEnvironment.Obstacle(500, 400, 40),
    SingleAgentEnvironment.Obstacle(600, 200, 30),
    SingleAgentEnvironment.Obstacle(600, 400, 30),
]

obstacles_3 = [
    SingleAgentEnvironment.Obstacle(200, 100, OBSTACLE_RADIUS),
    SingleAgentEnvironment.Obstacle(600, 100, OBSTACLE_RADIUS),
    SingleAgentEnvironment.Obstacle(400, 300, OBSTACLE_RADIUS),
    SingleAgentEnvironment.Obstacle(600, 300, OBSTACLE_RADIUS),
    SingleAgentEnvironment.Obstacle(200, 500, OBSTACLE_RADIUS),
    SingleAgentEnvironment.Obstacle(400, 500, OBSTACLE_RADIUS),
    SingleAgentEnvironment.Obstacle(600, 500, OBSTACLE_RADIUS),
    SingleAgentEnvironment.Obstacle(300, 200, OBSTACLE_RADIUS),
    SingleAgentEnvironment.Obstacle(500, 200, OBSTACLE_RADIUS),
    SingleAgentEnvironment.Obstacle(300, 400, OBSTACLE_RADIUS),
    SingleAgentEnvironment.Obstacle(400, 400, OBSTACLE_RADIUS),
    SingleAgentEnvironment.Obstacle(400, 200, OBSTACLE_RADIUS),
    SingleAgentEnvironment.Obstacle(400, 100, OBSTACLE_RADIUS),
    # SingleAgentEnvironment.Obstacle(100, 300, 60),
    SingleAgentEnvironment.Obstacle(525, 375, 100),
    # SingleAgentEnvironment.Obstacle(700, 300, 10),
    SingleAgentEnvironment.Obstacle(400, 50, 70),
    SingleAgentEnvironment.Obstacle(400, 550, 80),
    # SingleAgentEnvironment.Obstacle(250, 350, 60),
    SingleAgentEnvironment.Obstacle(300, 100, 40),
    SingleAgentEnvironment.Obstacle(500, 100, 60),
    SingleAgentEnvironment.Obstacle(300, 500, 20),
    # SingleAgentEnvironment.Obstacle(500, 500, 40),
    # SingleAgentEnvironment.Obstacle(800, 600, 40),
]

diagnostics_obstacles = [obstacles_1, obstacles_2, obstacles_3]

obstacles_4 = SingleAgentEnvironment.create_obstacle_objects(200, 600, 50, 550, 10, 50, 15)

obstacles_array = [obstacles_1, obstacles_2, obstacles_3, obstacles_4]


def main():
    algo = "APF"
    

    

    
    obstacles_to_use = 0
    while obstacles_to_use > len(obstacles_array) or obstacles_to_use < 1:
        obstacles_to_use = int(input("\nWhich Obstacle Set would you like to use? (options: 1, 2, 3, 4)\n"))

    obstacles = obstacles_array[obstacles_to_use - 1]
    no =len(obstacles)
    for i in range(0,no):
        print(f"{obstacles[i].x}" +  "," +f"{obstacles[i].y}")


    num_rand_agents = 0
    while num_rand_agents > 15 or num_rand_agents < 1:
        num_rand_agents = int(input("\nHow many random agents would you like to generate?\n"))
          
            
        
           # agents = MultiAgentEnvironment.create_agent_line(100, int(random.uniform(500, 600)), num_rand_agents) #I changed it
    agents = MultiAgentEnvironment.create_random_agents(80, 140, 100, 500, num_rand_agents)
    for i in range(0,num_rand_agents):
        print(f"{agents[i].x}" +  "," +f"{agents[i].y}")
               
            # Generate random goals
            # r_goals = generate_random_goals(num_rand_agents, width, height)
    goal_apf=[]
    for _ in range(num_rand_agents):
        while True:
            x = random.randint(0, width)  # Adjust the width and height based on your environment
            y = random.randint(0, height)

            if is_point_valid(x, y, obstacles):
                goal_apf.append((x, y))
                break
    print("Goals are:")
    print(goal_apf)
    '''sheets = {}
        # Change algorithm being tested here (**CHANGE BACK TO LIST OF ALGOS)
        
    sheet = pd.DataFrame()
    for run in range(2):
        temp_sheet = MultiAgentEnvironment.run_scenario_multi_agent_diagnostics(diagnostics_obstacles, "APF",goal_apf,agents)
        sheet = pd.concat([sheet, temp_sheet], ignore_index=True)

    sheets["APF"] = sheet

    MultiAgentEnvironment.save_to_csv(sheets, 'Results.xlsx')'''

    MultiAgentEnvironment.run_scenario_multi_agent(obstacles, agents, goal_apf, algo,obstacles_to_use)

        


if __name__ == "__main__":
    main()
