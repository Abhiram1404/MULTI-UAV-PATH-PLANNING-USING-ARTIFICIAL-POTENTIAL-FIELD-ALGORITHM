import random
import pygame
import heapq
import math


# Constants
WIDTH = 800  # Width of the simulation window
HEIGHT = 600  # Height of the simulation window
AGENT_RADIUS = 10  # Radius of the agent
OBSTACLE_RADIUS = 30  # Radius of the obstacles
MOVEMENT_SPEED = 3  # Movement speed of the agent

# For APF
SEARCH_RADIUS = 150

# Colors
BLACK = (0, 0, 0)
WHITE = (255, 255, 255)
RED = (255, 0, 0)
GREEN = (0, 255, 0)
BLUE = (0, 0, 255)

class Agent:
    def __init__(self, x, y):
        self.x = x
        self.y = y
        self.start = (self.x, self.y)
        self.path = []

    def move(self):
        if self.path:
            next_pos = self.path[0]
            dx = next_pos[0] - self.x
            dy = next_pos[1] - self.y
            distance = (dx ** 2 + dy ** 2) ** 0.5
            if distance <= MOVEMENT_SPEED:
                self.x = next_pos[0]
                self.y = next_pos[1]
                self.path.pop(0)
            else:
                direction_x = int(dx / distance * MOVEMENT_SPEED)
                direction_y = int(dy / distance * MOVEMENT_SPEED)
                self.x += direction_x
                self.y += direction_y

    def draw(self, screen):

        pygame.draw.circle(screen, GREEN, (self.x, self.y), AGENT_RADIUS)



class Obstacle:
    def __init__(self, x, y, radius):
        self.x = x
        self.y = y
        self.radius = radius

    def draw(self, screen):
        pygame.draw.circle(screen, BLACK, (self.x, self.y), self.radius)


def euclidean_distance_obs(obstacle1,obstacle2):
    dx = obstacle1.x - obstacle2.x
    dy = obstacle1.y - obstacle2.y
    distance = math.sqrt(dx ** 2 + dy ** 2)
    return distance



def create_obstacle_objects(min_x, max_x, min_y, max_y, min_size, max_size, num_obstacles):
    obstacle_objects = []
    for _ in range(num_obstacles):
        
        while True:
            x = int(random.uniform(min_x, max_x))
            y = int(random.uniform(min_y, max_y))
            size = int(random.randint(min_size, max_size))
            obstacle = Obstacle(x, y, size)
            flag=True
            for obs in obstacle_objects:
                if euclidean_distance_obs(obs,obstacle)<obs.radius+size+10:
                    flag=False
                    break
            if flag:
                obstacle_objects.append(obstacle)
                break
            
    return obstacle_objects

class Algorithm:
    def __init__(self, agent, obstacles):
        self.agent = agent
        self.obstacles = obstacles

    def apf_search(self, goal):
        # Compute the attractive force between the agent and the goal
        def attractive_force(agent_pos, goal_pos):
            k_att = 50.0  # Attractive force gain
            dx = (goal_pos[0] - agent_pos[0])
            dy = goal_pos[1] - agent_pos[1]
            angle = math.atan2(dy, dx)  # The direction doesn't matter if the UAVs are represented as circles
            return k_att * dx, k_att * dy

        # Compute the repulsive force between the agent and an obstacle
        def repulsive_force(agent_pos, obstacle_pos, obstacle_radius):
            k_rep = 100.0  # Repulsive force gain
            p0 = AGENT_RADIUS + obstacle_radius  # Influence radius of F_rep
            obst_dist_x = agent_pos[0] - obstacle_pos[0]
            obst_dist_y = agent_pos[1] - obstacle_pos[1]
            dist = math.sqrt(obst_dist_x ** 2 + obst_dist_y ** 2)  # Dist btwn UAV and obstacle
            if dist <= SEARCH_RADIUS + obstacle_radius:  # checks if obstacle is in search radius
                if dist <= p0:
                    x_rep = k_rep * ((1/obst_dist_x - (1/p0)) * (1 / obst_dist_x)**2)
                    y_rep = k_rep * ((1 / obst_dist_y - (1 / p0)) * (1 / obst_dist_y) ** 2)
                    return x_rep, y_rep
                else:
                    return (0.0, 0.0)
            else:
                return (0.0, 0.0)


        # Compute the total force acting on the agent at its current position
        def total_force(agent_pos, goal_pos, obstacles):
            force_x, force_y = attractive_force(agent_pos, goal_pos)

            for obstacle in obstacles:
                rep_force_x, rep_force_y = repulsive_force(agent_pos, (obstacle.x, obstacle.y), obstacle.radius)
                force_x += rep_force_x
                force_y += rep_force_y
            return (force_x, force_y)


        # Move the agent towards the goal position based on the total force
        def move_towards(agent_pos, goal_pos, obstacles):
            force_x, force_y = total_force(agent_pos, goal_pos, obstacles)
            force_magnitude = math.sqrt(force_x ** 2 + force_y ** 2)
            if force_magnitude > MOVEMENT_SPEED:
                force_x /= force_magnitude
                force_y /= force_magnitude
                force_x *= MOVEMENT_SPEED
                force_y *= MOVEMENT_SPEED

            new_pos_x = agent_pos[0] + force_x
            new_pos_y = agent_pos[1] + force_y

            # Check for collision with obstacles and adjust the new position accordingly
            for obstacle in obstacles:
                dx = new_pos_x - obstacle.x
                dy = new_pos_y - obstacle.y
                distance = math.sqrt(dx ** 2 + dy ** 2)
                if distance <= AGENT_RADIUS + obstacle.radius:
                    angle = math.atan2(dy, dx)
                    new_pos_x = obstacle.x + (AGENT_RADIUS + obstacle.radius) * math.cos(angle)
                    new_pos_y = obstacle.y + (AGENT_RADIUS + obstacle.radius) * math.sin(angle)
                    break

            return (new_pos_x, new_pos_y)

        path = [self.agent.start]
        current_pos = self.agent.start

        while True:
            # Move towards the next position based on the total force
            next_pos = move_towards(current_pos, goal, self.obstacles)
            path.append(next_pos)

            # Check if the agent has reached the goal position
            if math.sqrt((next_pos[0] - goal[0]) ** 2 + (next_pos[1] - goal[1]) ** 2) <= MOVEMENT_SPEED:
                break

            current_pos = next_pos

        return path

    

def run_scenario_single_agent(obstacles_in, agent_in, goal_in, algorithm_type):
    # Initialize Pygame
    pygame.init()
    screen = pygame.display.set_mode((WIDTH, HEIGHT))
    pygame.display.set_caption("Path Planning Simulation")
    clock = pygame.time.Clock()

    # Create agent and obstacles
    agent = agent_in  # Initialize agent at the start point
    obstacles = obstacles_in

    # Set goal position
    goal = goal_in

    # Create an instance of the Algorithm class
    algorithm = Algorithm(agent, obstacles)

    # Find paths for each agent depending on search method
    # Add the way your algorithm is accessed here
    
    path = algorithm.apf_search(goal)
    agent.path = path.copy()
    

    # Game loop
    running = True
    while running:
        # Handle events
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False

        # Update the agent's position
        agent.move()

        # Clear the screen
        screen.fill(WHITE)

        # Draw the search radius
        if algorithm_type == "APF":
            pygame.draw.circle(screen, RED, (agent.x, agent.y), SEARCH_RADIUS)

        # Draw the agent
        agent.draw(screen)

        # Draw obstacles
        for obstacle in obstacles:
            obstacle.draw(screen)

        # Draw the start and goal positions
        pygame.draw.circle(screen, BLUE, agent.start, 5)
        pygame.draw.circle(screen, BLUE, goal, 5)

        # Draw the path
        if path:
            pygame.draw.lines(screen, BLUE, False, path)

        # Update the display
        pygame.display.flip()
        clock.tick(60)

    # Quit the simulation
    pygame.quit()
