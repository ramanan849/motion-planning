
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
import math
import random
import matplotlib.pyplot as plt

class DStarPathPlanner:
    def __init__(self, Start, Goal, Occup_Grid):
        self.Start = Start
        self.Goal = Goal
        self.grid_width = len(Occup_Grid)
        self.grid_height = len(Occup_Grid[0])
        self.grid = Occup_Grid
        self.open_list = []
        self.g_values = {}
        self.rhs_values = {}
        self.parent = {}

        # Initialize grid values
        for i in range(self.grid_width):
            for j in range(self.grid_height):
                self.g_values[(i, j)] = float('inf')
                self.rhs_values[(i, j)] = float('inf')
                self.parent[(i, j)] = None

        # Set start and goal values
        self.g_values[self.Goal] = 0
        self.rhs_values[self.Goal] = 0
        self.open_list.append((self.goal_key(self.Goal), self.Goal))

    def goal_key(self, position):
        g = self.g_values[position]
        rhs = self.rhs_values[position]
        return (min(g, rhs), g)

    def get_neighbors(self, node):
        x, y = node
        neighbors = [
            (x + dx, y + dy)
            for dx, dy in [(-1, 0), (1, 0), (0, -1), (0, 1)]
            if 0 <= x + dx < self.grid_width and 0 <= y + dy < self.grid_height
            and self.grid[x + dx][y + dy] != 1  # Check for obstacles
        ]
        return neighbors

    def update_cell(self, node):
        if node == self.Goal:
            return
        min_rhs = float('inf')
        for neighbor in self.get_neighbors(node):
            new_rhs = self.g_values[neighbor] + 1
            if new_rhs < min_rhs:
                min_rhs = new_rhs
                self.parent[node] = neighbor
        self.rhs_values[node] = min_rhs
        self.open_list.append((self.goal_key(node), node))

    def compute_shortest_path(self):
        while self.open_list:
            _, current = heapq.heappop(self.open_list)
            if self.g_values[current] > self.rhs_values[current]:
                self.g_values[current] = self.rhs_values[current]
            else:
                self.g_values[current] = float('inf')
            for neighbor in self.get_neighbors(current):
                self.update_cell(neighbor)

    def reconstruct_path(self):
        path = [self.Start]
        current = self.Start
        while current != self.Goal:
            current = self.parent[current]
            path.append(current)
        return path

    def plan(self):
        self.compute_shortest_path()
        return self.reconstruct_path()

# Initialize grid size
grid_width, grid_height = 50, 50
Occup_Grid = [[0] * grid_height for _ in range(grid_width)]

# Add two 10x10 obstacles diagonally apart
# First obstacle at (10, 10)
for i in range(10):
    for j in range(10):
        if 0 <= 10 + i < grid_width and 0 <= 10 + j < grid_height:
            Occup_Grid[10 + i][10 + j] = 1

# Second obstacle at (30, 30)
for i in range(10):
    for j in range(10):
        if 0 <= 30 + i < grid_width and 0 <= 30 + j < grid_height:
            Occup_Grid[30 + i][30 + j] = 1

# Add 30 random crystals, making sure they don't overlap with obstacles or each other
crystals = []
crystal_size = 5  # Each crystal will be a 5x5 square

def is_free(x, y):
    """Check if a position is free for a crystal (not an obstacle and not too close to others)."""
    if not (0 <= x < grid_width and 0 <= y < grid_height):
        return False
    if Occup_Grid[x][y] == 1:
        return False  # It's an obstacle
    for cx, cy in crystals:
        # Make sure crystals are not placed too close to each other
        if abs(cx - x) < crystal_size * 2 and abs(cy - y) < crystal_size * 2:
            return False
    return True

# Place 30 random crystals
for _ in range(30):
    while True:
        rand_x = random.randint(0, grid_width - crystal_size)
        rand_y = random.randint(0, grid_height - crystal_size)
        if is_free(rand_x, rand_y):
            crystals.append((rand_x, rand_y))
            # Mark the 5x5 area as occupied by the crystal
            for i in range(crystal_size):
                for j in range(crystal_size):
                    if 0 <= rand_x + i < grid_width and 0 <= rand_y + j < grid_height:
                        Occup_Grid[rand_x + i][rand_y + j] = 2  # Marking as a crystal area
            break

# Set start and goal positions
Start = (0, 0)
Goal = (49, 49)

# Visualize the grid
def plot_grid():
    fig, ax = plt.subplots(figsize=(10, 10))
    ax.imshow(Occup_Grid, cmap="binary", origin="upper")
    
    # Mark the start and goal positions
    ax.text(Start[1], Start[0], 'S', color='blue', fontsize=12, ha='center', va='center')
    ax.text(Goal[1], Goal[0], 'G', color='red', fontsize=12, ha='center', va='center')

    # Mark the crystals
    for (cx, cy) in crystals:
        ax.add_patch(plt.Rectangle((cy, cx), crystal_size, crystal_size, linewidth=1, edgecolor='green', facecolor='green'))

    plt.grid(True)
    plt.show()

plot_grid()

# Instantiate D* path planner
planner = DStarPathPlanner(Start, Goal, Occup_Grid)

# Get the path from start to goal
path = planner.plan()

# Output the found path
print("Path found:")
print(path)
