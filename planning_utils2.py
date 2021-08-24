from enum import Enum
from queue import PriorityQueue
import numpy as np
from bresenham import bresenham


def create_grid(data, drone_altitude, safety_distance):
    """
    Returns a grid representation of a 2D configuration space
    based on given obstacle data, drone altitude and safety distance
    arguments.
    """

    # minimum and maximum north coordinates
    north_min = np.floor(np.min(data[:, 0] - data[:, 3]))
    north_max = np.ceil(np.max(data[:, 0] + data[:, 3]))

    # minimum and maximum east coordinates
    east_min = np.floor(np.min(data[:, 1] - data[:, 4]))
    east_max = np.ceil(np.max(data[:, 1] + data[:, 4]))

    # given the minimum and maximum coordinates we can
    # calculate the size of the grid.
    north_size = int(np.ceil(north_max - north_min))
    east_size = int(np.ceil(east_max - east_min))

    # Initialize an empty grid
    grid = np.zeros((north_size, east_size))

    # Populate the grid with obstacles
    for i in range(data.shape[0]):
        north, east, alt, d_north, d_east, d_alt = data[i, :]
        if alt + d_alt + safety_distance > drone_altitude:
            obstacle = [
                int(np.clip(north - d_north - safety_distance - north_min, 0, north_size-1)),
                int(np.clip(north + d_north + safety_distance - north_min, 0, north_size-1)),
                int(np.clip(east - d_east - safety_distance - east_min, 0, east_size-1)),
                int(np.clip(east + d_east + safety_distance - east_min, 0, east_size-1)),
            ]
            grid[obstacle[0]:obstacle[1]+1, obstacle[2]:obstacle[3]+1] = 1

    return grid, int(north_min), int(east_min)


# Assume all actions cost the same.
class Action(Enum):
    """
    An action is represented by a 3 element tuple.

    The first 2 values are the delta of the action relative
    to the current grid position. The third and final value
    is the cost of performing the action.
    """

    WEST = (0, -1, 1)
    EAST = (0, 1, 1)
    NORTH = (-1, 0, 1)
    SOUTH = (1, 0, 1)
    
    # diagonal motions
#     NORTH_WEST = (-1, -1, np.sqrt(2))
#     NORTH_EAST = (-1, 1, np.sqrt(2))
#     SOUTH_WEST = (1, -1, np.sqrt(2))
#     SOUTH_EAST = (1, 1, np.sqrt(2))
    NORTH_WEST = (-1, -1, 1)
    NORTH_EAST = (-1, 1, 1)
    SOUTH_WEST = (1, -1, 1)
    SOUTH_EAST = (1, 1, 1)
    

    @property
    def cost(self):
        return self.value[2]

    @property
    def delta(self):
        return (self.value[0], self.value[1])


def valid_actions(grid, current_node):
    """
    Returns a list of valid actions given a grid and current node.
    """
    valid_actions = list(Action)
    n, m = grid.shape[0] - 1, grid.shape[1] - 1
    x, y = current_node

    # check if the node is off the grid or
    # it's an obstacle

    if x - 1 < 0 or grid[x - 1, y] == 1:
        valid_actions.remove(Action.NORTH)
    if x + 1 > n or grid[x + 1, y] == 1:
        valid_actions.remove(Action.SOUTH)
    if y - 1 < 0 or grid[x, y - 1] == 1:
        valid_actions.remove(Action.WEST)
    if y + 1 > m or grid[x, y + 1] == 1:
        valid_actions.remove(Action.EAST)
    
    # remove diagonal motions
    if (x - 1 < 0 or y - 1 < 0) or grid[x - 1, y - 1] == 1:
        valid_actions.remove(Action.NORTH_WEST)
    if (x - 1 < 0 or y + 1 > m) or grid[x - 1, y + 1] == 1:
        valid_actions.remove(Action.NORTH_EAST)
    if (x + 1 > n or y - 1 < 0) or grid[x + 1, y - 1] == 1:
        valid_actions.remove(Action.SOUTH_WEST)
    if (x + 1 > n or y + 1 > m) or grid[x + 1, y + 1] == 1:
        valid_actions.remove(Action.SOUTH_EAST)

    return valid_actions


def a_star(grid, h, start, goal):

    path = []
    path_cost = 0
    queue = PriorityQueue()
    queue.put((0, start))
    visited = set(start)

    branch = {}
    found = False
    
    print("search path")
    while not queue.empty():
        
        item = queue.get()
        current_node = item[1]
        if current_node == start:
            current_cost = 0.0
        else:              
            current_cost = item[0]
            
        if current_node == goal:        
            print('Found a path.')
            found = True
            break
        else:
            for action in valid_actions(grid, current_node):
                # get the tuple representation
                da = action.delta
                next_node = (current_node[0] + da[0], current_node[1] + da[1])
                branch_cost = current_cost + action.cost
                queue_cost = branch_cost + h(next_node, goal)
                
                if next_node not in visited:                
                    visited.add(next_node)               
                    branch[next_node] = (branch_cost, current_node, action)
                    queue.put((queue_cost, next_node))
             
    if found:
        # retrace steps
        n = goal
        path_cost = branch[n][0]
        path.append(goal)
        while branch[n][1] != start:
            path.append(branch[n][1])
            n = branch[n][1]
        path.append(branch[n][1])
    else:
        print('**********************')
        print('Failed to find a path!')
        print('**********************') 
    return path[::-1], path_cost



def heuristic(position, goal_position):
    return np.linalg.norm(np.array(position) - np.array(goal_position))


def collinearity_int(p1, p2, p3): 
    collinear = False
    # TODO: Calculate the determinant of the matrix using integer arithmetic 
    det = p1[0]*(p2[1] - p3[1]) + p2[0]*(p3[1] - p1[1]) + p3[0]*(p1[1] - p2[1])
    # TODO: Set collinear to True if the determinant is equal to zero
    if abs(det) <1e-6:
        collinear = True

    return collinear


# check collinearity
def collinearity_float(p1, p2, p3, epsilon=1e-6): 
    collinear = False
    # TODO: Create the matrix out of three points
    p1 = np.array([p1[0], p1[1], 1])
    p2 = np.array([p2[0], p2[1], 1])
    p3 = np.array([p3[0], p3[1], 1])
    # Add points as rows in a matrix
    mat = np.vstack([p1, p2, p3])
    # TODO: Calculate the determinant of the matrix. 
    det = np.linalg.det(mat)
    # TODO: Set collinear to True if the determinant is less than epsilon
    if abs(det) < epsilon:
        collinear = True
        
    return collinear


# Ray tracing check
def bresenham_check(grid, p1, p2):
    # assuming connection can be connected
    connect = True
    
    # apply bresenham method to extract cells crossed by the ray
    cells = list(bresenham(p1[0], p1[1], p2[0], p2[1])) 
    
    # if any cell in cells is obstacle, set connect to False
    for cell in cells:
        if grid[cell[0], cell[1]] == 1:
            connect = False
            break
            
    # retrun check result
    return connect
    

# deifne function for collinearity prune. 
# This function apply function "collinearity_int"
def collinearity_prune(path):
    path_prune = []
    path_prune.append(path[0])

    for i in range(len(path)-2):
        p1 = path[i]
        p2 = path[i+1]
        p3 = path[i+2]
        if collinearity_int(p1, p2, p3) == False:
            path_prune.append(p2)
    
    path_prune.append(path[-1])
    
    return path_prune

# define function for applying bresenham_check to prune path
def bresenham_prune(grid, path):
    # start from start cell = cell[0] and target cell indice = 1
    # search for the farthest cell that can be reached directly from target cell 
    # Append this farthes cell into path_prune2 list.
    # Then, target cell <- farthes cell
    # until start cell = last cell in path_prune
    path_prune = []
    path_prune.append(path[0])

    # initial start_Cell and target_cell
    start_cell = path[0]
    target_indice = 1

    while target_indice < len(path):
        target_cell = path[target_indice]
        if bresenham_check(grid, start_cell, target_cell) == True:
            target_indice = target_indice+1
        else:
            start_cell = path[target_indice-1]
            path_prune.append(start_cell)

    # append final goal cell
    path_prune.append(path[-1])
    
    return path_prune

# find nearest start and goal location from skelton grid
def find_start_goal(skel, start, goal):
    skel_cells = np.transpose(skel.nonzero())
    start_min_dist = np.linalg.norm(np.array(start) - np.array(skel_cells), axis=1).argmin()
    near_start = skel_cells[start_min_dist]
    goal_min_dist = np.linalg.norm(np.array(goal) - np.array(skel_cells), axis=1).argmin()
    near_goal = skel_cells[goal_min_dist]
    
    return near_start, near_goal



