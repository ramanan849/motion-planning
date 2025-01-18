
"""
Grid/playing field - 100 * 100 pixels
waypoints (goals) - originally, 40*40
"""

import numpy as np
import matplotlib.pyplot as plt
import time
import random
import math
import heapq

'''
So, the alterred algo would be like, we have a list of coordinates where the crystals are present, but we keep it hidden from the algo, in a sense, the algo will not know the entire information at the start - initially, just one crystal would be visible and as soon one crystal is visible, the next crystal would be visible (given) to the algo to create a path

1. generate an array of 20 crystals outside the cave and 14 inside the cave

2. data from the perception module would give data of the position of crystals and that of the obstacles - 2 in number - 2 containers, one placed diagonally opposite and the walls

3. each box dimensions = 20*20

4. the D* must be modified such a way that the goal is to reach a set of coordinates by a very efficient path 


'''
import numpy as np
import matplotlib.pyplot as plt
import heapq
import random

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

        # Initialize heuristic map (Manhattan distance to goal)
        for i in range(self.grid_width):
            for j in range(self.grid_height):
                self.heuristic_map[i][j] = abs(goal[0] - i) + abs(goal[1] - j)

        # Start with the goal
        self.cost_map[goal] = 0
        self.push(goal, 0)

    def push(self, node, cost):
        heapq.heappush(self.open_list, (cost + self.heuristic_map[node[0], node[1]], node))
        self.cost_map[node] = cost

    def pop(self):
        return heapq.heappop(self.open_list)[1]

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

    def update_cost(self, node, new_cost):
        if new_cost < self.cost_map[node]:
            self.push(node, new_cost)

    def plan(self):
        while self.open_list:
            current_node = self.pop()
            if current_node == self.start:
                break
            neighbors = self.expand_neighbors(current_node)
            for neighbor in neighbors:
                new_cost = self.cost_map[current_node] + 1  # Uniform cost
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
            if next_node is None:  # No valid neighbor
                break
            node = next_node
        path.append(self.goal)
        return path[::-1]

    def update_map(self, new_goal):
        """Update the map with a new goal without reinitializing."""
        self.goal = new_goal
        self.cost_map = np.inf * np.ones_like(self.grid, dtype=float)
        self.heuristic_map = np.zeros_like(self.grid, dtype=float)
        for i in range(self.grid_width):
            for j in range(self.grid_height):
                self.heuristic_map[i][j] = abs(new_goal[0] - i) + abs(new_goal[1] - j)
        self.cost_map[new_goal] = 0
        self.push(new_goal, 0)


class PathPlanner:
    def __init__(self, start, goal, grid):
        self.start = start
        self.goal = goal
        self.grid = grid
        self.fig, self.ax = plt.subplots(figsize=(8, 8))  # Set up a single figure
        plt.ion()  # Enable interactive mode

    def visualize_grid(self, path, current_crystal):
        """Visualizes the grid with the path and updates interactively."""
        grid = np.array(self.grid)
        grid[self.start[0]][self.start[1]] = 2  # Start point
        grid[current_crystal[0]][current_crystal[1]] = 3  # Current crystal
        for x, y in path:
            grid[x][y] = 4  # Path

        # Clear and redraw the updated grid
        self.ax.clear()
        self.ax.imshow(grid, cmap="viridis", origin="upper")
        self.ax.set_title("Path Traversed and Grid State")
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()


# Grid setup (10x10 for simplicity)
Occup_Grid = [[0] * 10 for _ in range(10)]
random.seed(42)

# Randomly place 10 crystals
crystals = [(random.randint(0, 9), random.randint(0, 9)) for _ in range(10)]
for crystal in crystals:
    Occup_Grid[crystal[0]][crystal[1]] = 2  # Mark crystals as value 2

# Fixed obstacles
Occup_Grid[2][7] = 1  # Obstacle 1
Occup_Grid[8][4] = 1  # Obstacle 2

# Starting position
Start = (0, 0)

# Planner setup
path_planner = PathPlanner(Start, crystals[0], Occup_Grid)
dstar_planner = DStar(Start, crystals[0], Occup_Grid)

# Pathfinding loop
for crystal in crystals:
    
    print(f"Planning path to crystal {crystal}...")
    dstar_planner.update_map(crystal)  # Update goal
    path = dstar_planner.plan()
    path_planner.visualize_grid(path, crystal)
    

    # Mark the crystal as visited
    Occup_Grid[crystal[0]][crystal[1]] = 0  # Reset crystal location
    Start = crystal  # Update start position
    dstar_planner.start = Start  # Update D* start
    path_planner.visualize_grid(path, crystal)
    plt.show()
# Keep the plot open at the end




