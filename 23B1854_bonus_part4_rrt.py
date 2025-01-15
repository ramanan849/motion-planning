import numpy as np
import matplotlib.pyplot as plt
import time
import random
import math

class Graph:
    def __init__(self):
        self.size = 10 # it is a 10*10 map in the question  
        self.vertices = {} # dict
        self.edges = {} # dict, format: edges[vertex_i] = list of tuple(visualise, cost); i!=j
        self.obstacles = set() # the reason why it is a set() because, obstacles are unique and mustn't repeat;
    
    def Add_Vertex(self, vertex):
        if vertex not in self.vertices:
            self.vertices[vertex] = True
            self.edges[vertex] = [] # so, the format of edges would be like edges = {1:[(2,13),(3,5),(4,11.11)]}

    def Add_Edge(self, vertex1, vertex2, cost):
        if vertex1 in self.vertices and vertex2 in self.vertices:
            self.edges[vertex1].append((vertex2, cost)) # so, the format of edges would be like edges = {1:[(2,13),(3,5),(4,11.11)]}
            self.edges[vertex2].append((vertex1, cost)) # the map is symmetric, hence updating for vertex2 as well
    
    def Get_Cost(self, vertex1, vertex2): # the distance, the metric !!
        for neighbour, cost in self.edges[vertex1]:
            if neighbour == vertex2:
                return cost
        return float('Vertices not connected, dingbat!!')  # If not connected, show this msg (sorry for swearing)
    
    def get_k_nearest_neighbours(self, vertex, k):
        neighbours = self.edges[vertex] # neighbours is the list of all the neighbours of the particular vertex, along with the distance to each neighbour; bascially a copy of the edges[vertex]
        neighbours.sort(key=lambda x: x[1])  # Sort by cost; basically, the lambda method compares the costs of each edge and then the final expression .sort(the lamda expression) returns the neighbours sorted in ascending order
        return neighbours[:k] # we want only the first 'k' nearest neighbours

    def state_validity_checker(self, vertex):
        x, y = vertex
        if 0 <= x < self.size and 0 <= y < self.size and vertex not in self.obstacles: # BoundaryPPPP vertices: (0,0), (9,0), (9,9), (0,9); so, in the 1st quadrant
            return True
        return False

    def visualise(self, start, goal, path=[]):
        
        grid = np.zeros((self.size, self.size)) # The graph is made up of 0s and 1s. 0s are where there are no obstacles and 1s where there are obstacles
        for (x, y) in self.obstacles:
            grid[x][y] = 1  # marks obstacles
        
        plt.imshow(grid, cmap='gray', origin='upper') # creates a gray scale plot; higher values (1s) are white and lower values(0s) are black. Therefore, obstacles are white in color
        if path:
            path_x, path_y = zip(*path)
            plt.plot(path_y, path_x, color='red', linewidth=2) # explain!!!!!

        plt.scatter(start[1], start[0], color='green', s=100, label='Start')
        plt.scatter(goal[1], goal[0], color='blue', s=100, label='Goal')
        plt.legend()
        plt.show()

    def add_obstacle(self, obstacle):
        if obstacle!=(0,0) and obstacle!=(9,9): # making sure that the obstacle is not at the start or goal nodes
            self.obstacles.add(obstacle) # adding it to the set

def distance(v1, v2):
    return math.sqrt( (v1[0] - v2[0])**2 + (v1[1] - v2[1])**2 )

def random_node(): 
    while True: # just to make sure that the start and goal nodes aren't generated
        rn = (random.randint(0, 9), random.randint(0, 9))
        if rn not in ((0,0),(9,9)):
            break
        else:
            continue
    return rn


def nearest_vertex(graph, point):
    nearest = None
    min_dist = 15 # since the graph is of order 10*10 and we are excluding pts on boundary, the maximum distance b/w any 2 pts is 9root(2) 
    for vertex in graph.vertices:
        #print(graph.vertices)
        dist = distance(vertex, point)
        if dist < min_dist:
            nearest = vertex
            min_dist = dist
    return nearest


def biasing_near_goal(nearest): # HERE!!!!!!!!!!!!!! HEURISTIC
    if distance(nearest,goal)>6:
        return random_node()
    else:
        p = random.random()
        if p<0.7:
            return (9-random.randint(0,3),9-random.randint(0,3))
        else:
            return random_node

def new_state(nearest, random_node, step_size=1):
    # heuristic - biasing the random node generation near the goal node!!!
    direction = (random_node[0] - nearest[0], random_node[1] - nearest[1])
    length = math.sqrt(direction[0]**2 + direction[1]**2)
    if length == 0:
        return nearest
    step = (direction[0] / length, direction[1] / length)
    new_vertex = (nearest[0] + int(step[0]*step_size), nearest[1] + int(step[1]*step_size))
    return new_vertex



def rrt(graph, start, goal, max_iter=10000, step_size=1):
    start_time = time.time()
    graph.Add_Vertex(start)
    
    for i in range(max_iter):
        rand_point = random_node()
        #print(rand_point)
        nearest = nearest_vertex(graph, rand_point)
        new_vertex = new_state(nearest, rand_point, step_size)

        if graph.state_validity_checker(new_vertex):
            graph.Add_Vertex(new_vertex)
            graph.Add_Edge(nearest, new_vertex, distance(nearest, new_vertex))
            #print(graph.vertices)
            if distance(new_vertex, goal) <= step_size:
                
                graph.Add_Vertex(goal)
                graph.Add_Edge(new_vertex, goal, distance(new_vertex, goal))
                break
    
    # Backtracking to find the path
    path = []
    current = goal
    if current not in graph.vertices:  # Additional check to ensure the goal was added
        graph.Add_Vertex(goal)
    while current != start:
        for neighbor, cost in graph.edges[current]:
            if distance(neighbor, start) < distance(current, start):
                path.append(current)
                current = neighbor
                break
    path.append(start)
    path.reverse()

    end_time = time.time()
    return path, end_time - start_time
# Creating the graph
graph = Graph()

# Adding obstacles

obstacle_positions = set([(random.randint(0, 9), random.randint(0, 9)) for i in range(10)]) # randomly generating obstacles 
#bass = {(1,0),(1,1),(1,2),(1,3),(1,4),(1,5),(1,6),(1,7),(1,8),(4,9),(4,1),(4,2),(4,3),(4,4),(4,5),(4,6),(4,7),(4,8)}
for obs in obstacle_positions:
    graph.add_obstacle(obs)

# Start and goal positions
start = (0, 0)
goal = (9, 9)

# Run the RRT algorithm and visualise
path, duration = rrt(graph, start, goal)
graph.visualise(start, goal, path)
print(f"Time taken: {duration:.6f} seconds")
