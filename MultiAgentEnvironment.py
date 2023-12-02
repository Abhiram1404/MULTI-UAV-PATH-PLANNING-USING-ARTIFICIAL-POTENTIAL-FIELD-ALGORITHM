import random
import pygame
import heapq
import math
from matplotlib import pyplot as plt

#from maddpg_code import *
import statistics
import numpy as np
import imageio
import time
import csv
import pandas as pd



# Constants
WIDTH = 800  # Width of the simulation window
HEIGHT = 608  # Height of the simulation window
AGENT_RADIUS = 5  # Radius of the agent
OBSTACLE_RADIUS = 30  # Radius of the obstacles
MOVEMENT_SPEED = 3  # Movement speed of the agent

# For APF
SEARCH_RADIUS = 20

# Colors
BLACK = (0, 0, 0)
WHITE = (255, 255, 255)
RED = (255, 0, 0)
GREEN = (0, 255, 0)
BLUE = (0, 0, 255)


class Agent:
    def __init__(self, agent_id, x, y):
        self.agent_id = agent_id
        self.x = x
        self.y = y
        self.start = (self.x, self.y)
        self.path = []
        self.disp_goal_reached = False
        self.temp_path = []
        self.heading=0.0
        self.initial_pos = (x, y)  # For permanent pos storage

    def get_id(self):
        print(self.agent_id)

    def move(self):
        if self.path:
            next_pos = self.path[0]
            dx = next_pos[0] - self.x
            dy = next_pos[1] - self.y
            distance = (dx ** 2 + dy ** 2) ** 0.5
            
            if distance > 0:
                self.heading = math.atan2(dy, dx)
            
            self.x = next_pos[0]
            self.y = next_pos[1]
            self.path.pop(0)
            if len(self.path) == 0:
                print("agent path completed")

    def draw(self, screen):
        pygame.draw.circle(screen, GREEN, (self.x, self.y), AGENT_RADIUS)

    def reset(self):
        self.x, self.y = self.initial_pos
    def update_heading(self,angle):
        self.heading=angle




class Obstacle:
    def __init__(self, x, y, radius):
        self.x = x
        self.y = y
        self.radius = radius

    def draw(self, screen):
        pygame.draw.circle(screen, BLACK, (self.x, self.y), self.radius)


class Boundary:
    def __init__(self, x_i, y_i, x_f, y_f):
        self.start_x = x_i
        self.start_y = y_i
        self.final_x = x_f
        self.final_y = y_f




# ************************************************************

# Generate random agents
def create_random_agents(min_x, max_x, min_y, max_y, num_agents):
    agent_objects = []
    for agent_id in range(1, num_agents + 1):
        x = int(random.uniform(min_x, max_x))
        y = int(random.uniform(min_y, max_y))
        agent = Agent(agent_id, x, y)
        agent_objects.append(agent)
    return agent_objects


def create_agent_line(right_x, right_y, num_agents):
    agent_objects = []
    for agent_id in range(1, num_agents + 1):
        # agent = Agent(agent_id, right_x - 2*agent_id, right_y)
        agent = Agent(agent_id, right_x, right_y - 20 * agent_id)
        agent_objects.append(agent)
    return agent_objects



class Algorithm:
    def __init__(self, list_of_agents, obstacles):
        self.list_of_agents = list_of_agents
        self.apf_list_of_agents = list_of_agents.copy()
        self.obstacles = obstacles
        self.force_record = []
        self.angle_record = [0.0]

    

    def apf_search(self, goals):
        
        # Compute the attractive force between the agent and the goal
        def attractive_force(agent_pos,goal):    
            # id = agent_pos.id  
            # goal=goals[id]    
            # goal_pos = (goals[0],goals[1]) 
            k_att = 50.0  # Attractive force gain
            k_att2 = 3.0  # Velocity gain
            dx = (goal[0] - agent_pos[0])
            dy = (goal[1] - agent_pos[1])

            return k_att * dx + k_att2 * MOVEMENT_SPEED, k_att * dy + k_att2 * MOVEMENT_SPEED

        # Compute the repulsive force between the agent and an obstacle
        def repulsive_force(agent_pos, obstacle_pos, obstacle_radius,agent_heading):
            # k_rep = 10000000  # Repulsive force gain
            # k_rep = 500
            k_rep = 200
            buffer = SEARCH_RADIUS * (1/2)  # was 1/5
            p0 = obstacle_radius + buffer  # Influence radius of F_rep
            obst_dist_x = (agent_pos[0] - obstacle_pos[0])
            obst_dist_y = (agent_pos[1] - obstacle_pos[1])
            # dist = (obst_dist_x ** 2 + obst_dist_y ** 2)**0.5  # Dist btwn UAV and obstacle
            dist = (obst_dist_x ** 2 + obst_dist_y ** 2) ** 0.5 - obstacle_radius  # Dist btwn UAV and obstacle
            if dist <= p0:
                # x_rep = k_rep * ((1 / obst_dist_x - (1 / p0)) * (1 / obst_dist_x) ** 2)
                # y_rep = k_rep * ((1 / obst_dist_y - (1 / p0)) * (1 / obst_dist_y) ** 2)
                x_rep = k_rep**3 * ((1 / dist - 1 / p0) * (obst_dist_x / dist) * (1 / dist) ** 2)
                y_rep = k_rep**3 * ((1 / dist - 1 / p0) * (obst_dist_y / dist) * (1 / dist) ** 2)

                angle_to_obstacle = math.atan2(obst_dist_y, obst_dist_x)
                angle_diff = abs(agent_heading - angle_to_obstacle)
                dsafe = obstacle_radius
                # Adjust the penalty factor based on the angle difference (adjust as needed)
                if angle_diff <= math.pi /4 and dist <= dsafe:
                    penalty_factor = 2.0
                else:
                    penalty_factor = 1.0

                x_rep *= penalty_factor
                y_rep *= penalty_factor



                return x_rep, y_rep
                # return k_rep * (1/obst_dist_x), k_rep * (1/obst_dist_y)
            else:
                return (0.0, 0.0)

        def inter_agent_force(agent, agent_pos):
            dx_agent_pos = (goal[0] - agent_pos[0])
            dy_agent_pos = (goal[1] - agent_pos[1])
            dist_agent_pos = (dx_agent_pos ** 2 + dy_agent_pos ** 2) ** 0.5

            dx_agent = (goal[0] - agent.x)
            dy_agent = (goal[1] - agent.y)
            dist_agent = (dx_agent ** 2 + dy_agent ** 2) ** 0.5

            threshold = 50

            if (agent.x, agent.y) != agent_pos and (dist_agent_pos > threshold or dist_agent > threshold):
                k_rep = 100000  # Repulsive force gain
                buffer = AGENT_RADIUS + 1
                p0 = AGENT_RADIUS + buffer  # Influence radius of F_rep
                inter_dist_x = agent_pos[0] - agent.x
                inter_dist_y = agent_pos[1] - agent.y
                dist = (inter_dist_x ** 2 + inter_dist_y ** 2) ** 0.5  # Dist btwn UAV and obstacle
                if dist <= p0:
                    if inter_dist_x > 0:
                        # x_rep = k_rep * ((1 / inter_dist_x - (1 / p0)) * (1 / inter_dist_x) ** 2)
                        x_rep = k_rep**3 * ((1 / dist - 1 / p0) * (inter_dist_x / dist) * (1 / dist) ** 2)
                    else:
                        x_rep = 0
                    if inter_dist_y > 0:
                        # y_rep = k_rep * ((1 / inter_dist_y - (1 / p0)) * (1 / inter_dist_y) ** 2)
                        y_rep = k_rep**3 * ((1 / dist - 1 / p0) * (inter_dist_y / dist)* (1 / dist) ** 2)
                    else:
                        y_rep = 0
                    return x_rep, y_rep
                else:
                    return 0.0, 0.0
            else:
                return 0.0, 0.0

        # Compute the total force acting on the agent at its current position
        def total_force(agent_pos, obstacles,i):
            force_x, force_y = attractive_force(agent_pos,goals[i])
            # print("att_fx:", force_x, "att_fy:", force_y)
            

            for obstacle in obstacles:
                rep_force_x, rep_force_y = repulsive_force(agent_pos, (obstacle.x, obstacle.y), obstacle.radius,self.apf_list_of_agents[i].heading)
                force_x += rep_force_x
                force_y += rep_force_y
            # print("tot_fx:", force_x, "tot_fy:", force_y)

            for agent in self.apf_list_of_agents:
                rep_force_x, rep_force_y = inter_agent_force(agent, agent_pos)
                force_x += rep_force_x
                force_y += rep_force_y
            
            '''agent=self.apf_list_of_agents[i]
            obs=obstacles[0]
            dist=euclidean_distance(agent, (obs.x,obs.y))
            for obstacle in obstacles:
                
                if dist>euclidean_distance(agent, (obstacle.x,obstacle.y)):
                    dist=euclidean_distance(agent, (obstacle.x,obstacle.y))
                    obs=obstacle
            virtualSubTarget=virtual_sub_target(agent_pos, (obs.x,obs.y), obs.radius, agent.heading)
            if virtualSubTarget[0]:
                force_x_sub,force_y_sub=attractive_force(agent_pos,(virtualSubTarget[1][0],virtualSubTarget[1][1]))
                force_x-=force_x_sub
                force_y-=force_y_sub'''

            return (force_x, force_y)

        # Move the agent towards the goal position based on the total force
        def move_towards(episode, agent_pos, obstacles,i):
            force_x, force_y = total_force(agent_pos, obstacles,i)
            self.force_record.append((force_x, force_y))
            angle = math.atan2(force_y, force_x)
            self.angle_record.append(angle)
            force_magnitude = math.sqrt(force_x ** 2 + force_y ** 2)
            force_x /= force_magnitude
            force_y /= force_magnitude

            f = 1.3  # jitter coeff
            del_angle_x = angle - self.angle_record[episode-1]
            del_angle_y = math.pi/2 - del_angle_x
            del_angle = (del_angle_x**2 + del_angle_y**2)**0.5
            thresh = 178
            if thresh <= math.degrees(abs(del_angle)) < 180:
                jitter_buff_x = f * math.cos(self.angle_record[episode-1] + 0.5*del_angle_x)
                jitter_buff_y = f * math.cos((math.pi/2 - self.angle_record[episode-1]) + 0.5*del_angle_y)
                # print("jitter", episode)
            else:
                jitter_buff_x = 1
                jitter_buff_y = 1

            force_x *= MOVEMENT_SPEED * jitter_buff_x
            force_y *= MOVEMENT_SPEED * jitter_buff_y
            # print("angle", math.degrees(del_angle), "tot:", MOVEMENT_SPEED * f * math.cos(self.angle_record[episode-1] + 0.5*del_angle))
            
            '''obs=obstacles[0]
            dist=euclidean_distance(agent, (obs.x,obs.y))
            for obstacle in obstacles:
                
                if dist>euclidean_distance(agent, (obstacle.x,obstacle.y)):
                    dist=euclidean_distance(agent, (obstacle.x,obstacle.y))
                    obs=obstacle
            virtualSubTarget=virtual_sub_target(agent_pos, (obs.x,obs.y), obs.radius, agent.heading)
            if virtualSubTarget[0]:
                new_pos_x=virtualSubTarget[1][0]
                new_pos_y=virtualSubTarget[1][1]
            else:'''
            new_pos_x = agent_pos[0] + force_x
            new_pos_y = agent_pos[1] + force_y

            return (new_pos_x, new_pos_y)


        def virtual_sub_target(agent_pos, obstacle_pos, obstacle_radius, agent_heading):
        # Constants
            d_safe = obstacle_radius  # Adjust as needed
            alpha = 0.1  # Evaluation constant, adjust as needed
            rho_eff = 4.0 * d_safe + alpha  # Evaluation distance

            k1 = math.tan(agent_heading)  # Slope of UAV-obstacle line

            # Calculate virtual sub-target coordinates
            xdum_1 = obstacle_pos[0] - (d_safe ** 2) / ((1 + k1**2) ** 0.5)
            ydum_1 = obstacle_pos[1] - (d_safe ** 2) / ((1 + k1**2) ** 0.5)
            xdum_2 = obstacle_pos[0] + (d_safe ** 2) / ((1 + k1**2) ** 0.5)
            ydum_2 = obstacle_pos[1] + (d_safe ** 2) / ((1 + k1**2) ** 0.5)

            # Calculate evaluation factor J
            omega_1 = relative_distance(agent_pos, (xdum_1, ydum_1), obstacle_pos, obstacle_radius) - rho_eff - alpha
            omega_2 = relative_distance(agent_pos, (xdum_2, ydum_2), obstacle_pos, obstacle_radius) - rho_eff - alpha

            J = (alpha * omega_1) ** 2 * relative_distance(agent_pos, (xdum_1, ydum_1), obstacle_pos, obstacle_radius) + rho_eff + \
                (alpha * omega_2) ** 2 * relative_distance(agent_pos, (xdum_2, ydum_2), obstacle_pos, obstacle_radius) + rho_eff

            # Check if the drone needs to avoid the obstacle based on the virtual sub-target
            if omega_1 < 0 or omega_2 < 0:
                return True, (xdum_1, ydum_1), (xdum_2, ydum_2), J
            else:
                return False, None, None, J
            
            
        def relative_distance(point, line_point, obstacle_pos, obstacle_radius):
        # Calculate the relative distance ω as described in Equation (11)
            px, py = point
            xobs, yobs = obstacle_pos
            xdum, ydum = line_point

        # Calculate the slope of the UAV-obstacle line
            k1 = (ydum - py) / (xdum - px) if xdum != px else float('inf')
            
            # Calculate the angle ψ between the UAV's current flight path and the obstacle
            psi = math.atan2(yobs - py, xobs - px)

            # Calculate the distance from the obstacle to the line defined by the virtual sub-target
            omega = ((yobs - py) - k1 * (xobs - px)) / ((1 + k1 ** 2) ** 0.5)

            return omega



        paths = []
        num_iterations = 1000
        # Ensure you have the same number of goals as agents
        assert len(self.apf_list_of_agents) == len(goals) #"Number of agents should match the number of goals."
        for episode in range(0, 1000):
            all_goal = True

            for i, agent in enumerate(self.apf_list_of_agents):
                g=goals[i]
                goal = (g[0],g[1])
                # Check if the agent has reached the goal position (close enough)
                if math.sqrt((agent.x - goal[0]) ** 2 + (agent.y - goal[1]) ** 2) <= MOVEMENT_SPEED:
                    next_pos = goal
                    agent.path.append(goal)
                    agent.temp_path.append(goal)
                    # self.list_of_agents.remove(agent) # Need to remove the agent somehow if it reaches the goal

                else:
                    all_goal = False
                    next_pos = move_towards(episode, (agent.x, agent.y), self.obstacles,i)
                    
                    #next_pos = virtual_sub_target((agent.x, agent.y), obstacle_pos, obstacle_radius, agent.heading)
                    agent.temp_path.append(next_pos)
                    agent.path.append(next_pos)

                agent.x, agent.y = next_pos

            if all_goal:
                print("Episode:", episode)
                num_iterations = episode
                break

        for agent in self.list_of_agents:
            paths.append(agent.temp_path)
            print("last_point",paths[-1][-1])

        return paths, num_iterations

    

    

def save_to_csv(data_dict, file_name):
    # Create an Excel writer object
    writer = pd.ExcelWriter(file_name, engine='xlsxwriter')

    # Iterate through the dictionary and save each tab to the Excel writer
    for sheet_name, sheet_data in data_dict.items():
        # Save the DataFrame to the Excel writer with the sheet_name
        sheet_data.to_excel(writer, sheet_name=sheet_name, index=False)

    # Save the Excel writer to the file
    writer.close()

    print(f"Data saved to '{file_name}' successfully!")


def path_length_diagnostics(paths, goal, obstacles):
    total_path_length = 0
    incomplete_paths = 0
    complete_paths = 0
    i=0
    for path in paths:
        
        temp_length = 0
        path_complete = True
        prev_point = path[0]
        for point in path:
            if not path_complete:
                break
            for obstacle in obstacles:
                dx = point[0] - obstacle.x
                dy = point[1] - obstacle.y
                distance = math.sqrt(dx ** 2 + dy ** 2)
                if distance <= AGENT_RADIUS + obstacle.radius:
                    path_complete = False

            dx = point[0] - prev_point[0]
            dy = point[1] - prev_point[1]
            distance = math.sqrt(dx ** 2 + dy ** 2)
            temp_length += distance
            if distance > MOVEMENT_SPEED*10:
                path_complete = False
            prev_point = point

        if path[-1] == goal[i]:
            path_complete_csv = True
        else:
            path_complete_csv = False
            

        if path_complete_csv:
            total_path_length += temp_length
            complete_paths += 1
        else:
            incomplete_paths += 1
        i+=1
    if complete_paths > 0:
        average_path_length = float(total_path_length)/complete_paths
    else:
        average_path_length = 0
    completion_percentage = float(complete_paths)/(complete_paths + incomplete_paths)
    print("completion percentage: ")
    print(completion_percentage)

    return average_path_length, completion_percentage



def run_scenario_multi_agent_diagnostics(lo_obstacles, algorithm_type,goal_apf,apf_list_of_agents,req_list,obstacles_to_use):

    col_names = ['agent_list', 'num of agents', 'obstacle difficulty', 'time', 'path length', 'completion %', 'iterations used']
    
    data_dict = {}
    sheet = pd.DataFrame(data_dict)
            # input variables
            
    paths = []

    agent_list_name = "Agent " 
    agent_len = len(apf_list_of_agents)

            # time the length of the algorithm for results
    start_time = time.time()

            # Find paths for each agent depending on search method
            # Add the way your algorithm is accessed here
            
    paths=req_list[0]
    iterations=req_list[1]       
    '''algorithm = Algorithm(apf_list_of_agents, lo_obstacles)
    paths, iterations = algorithm.apf_search(goal_apf)'''
            
    end_time=req_list[2]
    elapsed_time=req_list[3]
    average_length=req_list[4]
    completion_percentage=req_list[5]
    '''end_time = time.time()
    elapsed_time = end_time - start_time
    average_length, completion_percentage = path_length_diagnostics(paths, goal_apf, lo_obstacles)'''

    new_dict = {}
    new_dict[col_names[0]] = [agent_list_name]
    new_dict[col_names[1]] = [agent_len]
    new_dict[col_names[2]] = [obstacles_to_use]
    new_dict[col_names[3]] = [elapsed_time]
    new_dict[col_names[4]] = [average_length]
    new_dict[col_names[5]] = [completion_percentage]
    new_dict[col_names[6]] = [iterations]
            
    print("datapoint complete")
    new_data_point = pd.DataFrame(new_dict, index=None)
    sheet = pd.concat([sheet, new_data_point], ignore_index=True)

    print(sheet.to_string())
    return sheet



#function for euclidean distance
def euclidean_distance(agent, goal):
    dx = agent.x - goal[0]
    dy = agent.y - goal[1]
    distance = math.sqrt(dx ** 2 + dy ** 2)
    return distance

#function to find smallest elements of the distance matrix
def find_smallest_n(matrix, n):
    index_pairs = []
    heap=[]
    agent_set=set()
    goal_set=set()
    for i in range(len(matrix)):
        for j in range(len(matrix[i])):
            heapq.heappush(heap, [matrix[i][j],i,j])
    while len(index_pairs)<n:
        dist,i,j=heapq.heappop(heap)
        if i not in agent_set and j not in goal_set:
            index_pairs.append((i,j))
            agent_set.add(i)
            goal_set.add(j)
            
        
    
    return index_pairs

def run_scenario_multi_agent(obstacles_in, agents_in, goal_in, algorithm_type,obstacles_to_use):
    # Initialize Pygame
    pygame.init()
    screen = pygame.display.set_mode((WIDTH, HEIGHT))
    pygame.display.set_caption("Path Planning Simulation")
    clock = pygame.time.Clock()

    # input variables
    agents = agents_in
    obstacles = obstacles_in
    goal_position = goal_in
    paths = []
    goals=[]   #for golas in order of uavs
    # time the length of the algorithm for results
    start_time = time.time()

    # Find paths for each agent depending on search method
    # Add the way your algorithm is accessed here
    
    if algorithm_type == "APF":
        algorithm = Algorithm(agents, obstacles)
        n=len(goal_in)                                     #y
        # Create a matrix to store the distances
        distance_matrix = np.zeros((n, n))                  #y
            #create a distance matrix between goals and agents
        for i in range(n):                                                  #y
            for j in range(n):
                distance_matrix[i][j] = euclidean_distance(agents[i], goal_position[j])
        index_pairs = find_smallest_n(distance_matrix, n)
        index_pairs.sort(key=lambda x: x[0])
        # Extract the second values (column subscripts) into a separate list        #y
        column_subscripts = [pair[1] for pair in index_pairs]
        for i in column_subscripts:
            goals.append(goal_position[i])                                              #y
        #paths, iterations = algorithm.apf_search(goal_position)
        print("goals_order",goals)
        paths, iterations = algorithm.apf_search(goals)
        
    

    end_time = time.time()
    elapsed_time = end_time - start_time
    print(f"The algorithm block took {elapsed_time} seconds to execute.")

    average_length, completion_percentage = path_length_diagnostics(paths, goal_position, obstacles)
    print(f"The average path length of the swarm was {average_length} points")
    print(f"The percentage of robots that made it to the goal was {completion_percentage * 100}%")

    frames = []

    # Game loop
    running = True
    while running:
        # Handle events
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

        # Update the agent's position
        
        
        for agent in agents:
            agent.move()
            if (agent.x, agent.y) == goal_position and not agent.disp_goal_reached:
                print("agent reached goal")
                agent.disp_goal_reached = True


        # Clear the screen
        screen.fill(WHITE)

        # Draw the search radius
        
        for agent in agents:
            pygame.draw.circle(screen, RED, (agent.x, agent.y), SEARCH_RADIUS)

        # Draw the agent
        # Draw the start and goal positions
        

        
        for agent in agents:
            agent.draw(screen)
            pygame.draw.circle(screen, BLUE, agent.start, 5)
            #pygame.draw.circle(screen, BLUE, goal_position, 5)

        for goal in goal_position:
            pygame.draw.circle(screen, RED, goal, 5)

            
        # Draw the obstacles
        for obstacle in obstacles:
            obstacle.draw(screen)

        # Draw the path
        for path in paths:
            # print(path)
            pygame.draw.lines(screen, BLUE, False, path)


        # Update the display
        pygame.display.flip()

        # Capture the screen as an image
        frame = pygame.surfarray.array3d(screen)
        # Flip the frame vertically
        frame = np.flipud(np.rot90(frame, k=1))
        frames.append(frame)

        clock.tick(60)
    
    
    req_list=[paths,iterations,end_time,elapsed_time,average_length,completion_percentage]
    
    sheets = {}
        # Change algorithm being tested here (**CHANGE BACK TO LIST OF ALGOS)
        
    sheet = pd.DataFrame()
    for run in range(2):
        temp_sheet = run_scenario_multi_agent_diagnostics(obstacles_in, algorithm_type,goal_in,agents_in,req_list,obstacles_to_use)
        sheet = pd.concat([sheet, temp_sheet], ignore_index=True)

    sheets["APF"] = sheet

    save_to_csv(sheets, 'Results_apf.xlsx')

    # Quit the simulation
    pygame.quit()

    # Ignore the imageio warning
    imageio.mimsave('simulation.mp4', frames, fps=60)
    
