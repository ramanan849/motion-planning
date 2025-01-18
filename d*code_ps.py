
"""
Grid/playing field - 100 * 100 pixels
waypoints (goals) - originally, 40*40
"""

import numpy as np
import matplotlib.pyplot as plt
import time
import random
import math

'''
So, the alterred algo would be like, we have a list of coordinates where the crystals are present, but we keep it hidden from the algo, in a sense, the algo will not know the entire information at the start - initially, just one crystal would be visible and as soon one crystal is visible, the next crystal would be visible (given) to the algo to create a path

1. generate an array of 20 crystals outside the cave and 14 inside the cave

2. data from the perception module would give data of the position of crystals and that of the obstacles - 2 in number - 2 containers, one placed diagonally opposite and the walls

3. each box dimensions = 20*20

4. the D* must be modified such a way that the goal is to reach a set of coordinates by a very efficient path 


'''
import numpy as np
import matplotlib.pyplot as plt
import random
import math
import heapq

class DStar:
    def __init__(self, start, goal, grid):
        self.start = start
        self.goal = goal
        self.grid = grid
        self.grid_width = len(grid)
        self.grid_height = len(grid[0])
        self.cost_map = np.inf * np.ones_like(grid, dtype=float)
        self.heuristic_map = np.zeros_like(grid, dtype=float)
        self.open_list = []
        self.parent = {}
        
        # Calculate heuristic (Manhattan distance to goal)
        for i in range(self.grid_width):
            for j in range(self.grid_height):
                self.heuristic_map[i][j] = abs(self.goal[0] - i) + abs(self.goal[1] - j)
        
        self.cost_map[self.goal] = 0
        self.push(self.goal, 0)

    def push(self, node, cost):
        heapq.heappush(self.open_list, (cost + self.heuristic_map[node[0], node[1]], node))
        self.cost_map[node] = cost

    def pop(self):
        return heapq.heappop(self.open_list)[1]

    def update_cost(self, node, new_cost):
        if new_cost < self.cost_map[node]:
            self.cost_map[node] = new_cost
            self.push(node, new_cost)

    def valid(self, x, y):
        return 0 <= x < self.grid_width and 0 <= y < self.grid_height and self.grid[x][y] != 1

    def expand_neighbors(self, current_node):
        neighbors = []
        x, y = current_node
        for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1)]:
            nx, ny = x + dx, y + dy
            if self.valid(nx, ny):
                neighbors.append((nx, ny))
        return neighbors

    def plan(self):
        while self.open_list:
            current_node = self.pop()
            if current_node == self.start:
                break

            neighbors = self.expand_neighbors(current_node)
            for neighbor in neighbors:
                new_cost = self.cost_map[current_node] + 1  # Assuming uniform cost for simplicity
                self.update_cost(neighbor, new_cost)

        # Backtrack to generate the path
        path = []
        node = self.start
        while node != self.goal:
            path.append(node)
            min_cost = np.inf
            next_node = None
            for neighbor in self.expand_neighbors(node):
                if self.cost_map[neighbor] < min_cost:
                    min_cost = self.cost_map[neighbor]
                    next_node = neighbor
            node = next_node
        path.append(self.goal)
        return path[::-1]

    def update_map(self, new_obstacle_position):
        x, y = new_obstacle_position
        self.grid[x][y] = 1  # Mark as obstacle
        # After adding a new obstacle, re-run D* from the goal
        self.__init__(self.start, self.goal, self.grid)


class PathPlanner:
    def __init__(self, Start, Goal, Occup_Grid):
        self.Start = Start
        self.Goal = Goal
        self.grid_width = len(Occup_Grid)
        self.grid_height = len(Occup_Grid[0])
        self.occup_grid = Occup_Grid

    def visualize_path(self, path):
        """Visualizes the grid with the path highlighted."""
        grid = np.array(self.occup_grid)
        # Mark the start and goal
        grid[self.Start[0]][self.Start[1]] = 2  # Start is marked with 2
        grid[self.Goal[0]][self.Goal[1]] = 3  # Goal is marked with 3

        # Mark the path on the grid
        for x, y in path:
            grid[int(x)][int(y)] = 4  # Path is marked with 4

        # Visualize using matplotlib
        plt.figure(figsize=(8, 8))
        plt.imshow(grid, cmap="viridis", origin="upper")
        plt.colorbar(label="Grid Values")
        plt.title("Path Traversed")
        plt.show()



Occup_Grid = [[0] * 100 for _ in range(100)]  # 100x100 grid
Occup_Grid[5][5] = 1  # Adding an obstacle
Occup_Grid[10][10] = 1  # Adding another obstacle


crystals = [(random.randint(0, 99), random.randint(0, 99)) for _ in range(20)]
for crystal in crystals:
    Occup_Grid[crystal[0]][crystal[1]] = 2  # Mark crystals with value 2

# Fixed obstacles (2 obstacles placed)
Occup_Grid[20][20] = 1  # Obstacle 1
Occup_Grid[30][30] = 1  # Obstacle 2

# Starting position
Start = (0, 0)

# Planner setup
dstar_planner = DStar(Start, crystals[0], Occup_Grid)  # Initial goal is the first crystal

path_planner = PathPlanner(Start, crystals[0], Occup_Grid)

# Loop through each crystal
for crystal in crystals:
    print(f"Planning path to crystal {crystal}...")
    dstar_planner.goal = crystal
    path = dstar_planner.plan()
    path_planner.Start = path[-1]  # Set new start as the last point of the path
    path_planner.Goal = crystal
    
    # After reaching one crystal, re-plan for the next crystal
    # (You can add logic here to stop when all crystals are visited)
path_planner.visualize_path(path)
