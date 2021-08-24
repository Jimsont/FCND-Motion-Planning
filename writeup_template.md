## Project: 3D Motion Planning
![Quad Image](./misc/enroute.png)

---


# Required Steps for a Passing Submission:
1. Load the 2.5D map in the colliders.csv file describing the environment.
2. Discretize the environment into a grid or graph representation.
3. Define the start and goal locations.
4. Perform a search using A* or other search algorithm.
5. Use a collinearity test or ray tracing method (like Bresenham) to remove unnecessary waypoints.
6. Return waypoints in local ECEF coordinates (format for `self.all_waypoints` is [N, E, altitude, heading], where the drone’s start location corresponds to [0, 0, 0, 0].
7. Write it up.
8. Congratulations!  Your Done!

## [Rubric](https://review.udacity.com/#!/rubrics/1534/view) Points
### Here I will consider the rubric points individually and describe how I addressed each point in my implementation.  

---
### Writeup / README

#### 1. Provide a Writeup / README that includes all the rubric points and how you addressed each one.  You can submit your writeup as markdown or pdf.  

You're reading it! Below I describe how I addressed each rubric point and where in my code each point is handled.

### Explain the Starter Code

#### 1. Explain the functionality of what's provided in `motion_planning.py` and `planning_utils.py`
These scripts contain a basic planning implementation. In file `motion_planning.py`, function `plan_path` takes map center as the start and add 10 meters to both
[N, E] coordinates to calculate goal. Then, apply a-star function defined in file `planning_utils.py` to search the path. After a viable path has been found, the path will be store in waypoints list which will be used when function `self.waypoint_transition()` is executed.

And here's a lovely image of my results (ok this image has nothing to do with it, but it's a nice example of how to include images in your writeup!)
![Top Down View](./misc/high_up.png)

Here's | A | Snappy | Table
--- | --- | --- | ---
1 | `highlight` | **bold** | 7.41
2 | a | b | c
3 | *italic* | text | 403
4 | 2 | 3 | abcd

### Implementing Your Path Planning Algorithm

#### 1. Set your global home position
Here students should read the first line of the csv file, extract lat0 and lon0 as floating point values and use the self.set_home_position() method to set global home. 
Here is the code used to read the first line of the csv file, extract lat0 and lon0, and assign this global position as home position.

  1. define path of the file and read the first line\
  `path = './colliders.csv'`\
  `with open(path, 'r') as f:`\
  &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; `reader = csv.reader(f, delimiter=',')`\
  &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; `headers = next(reader)`

  2. split first line into individual string and convert string to float
  `lat0 = float(headers[0].split()[1])`
  `lon0 = float(headers[1].split()[1])`

  3. set home position to (lon0, lat0, 0)
  `self.set_home_position(lon0, lat0, 0)`

And here is a lovely picture of our downtown San Francisco environment from above!
![Map of SF](./misc/map.png)

#### 2. Set your current local position
  1. extract current global position. Then, transfer current global position into local position. The center of the coordinate system is map center. Here is the code.\
`local_position = global_to_local(self.global_position, self.global_home)`

Meanwhile, here's a picture of me flying through the trees!
![Forest Flying](./misc/in_the_trees.png)

#### 3. Set grid start position from local position
This is another step in adding flexibility to the start location. Because grid center is different from map center, there is an offset between two coordinate systems. Add offset to transfer local_position into grid start position.\
`grid_start = (int(local_position[0]-north_offset), int(local_position[1]-east_offset))`

#### 4. Set grid goal position from geodetic coords
This step is to add flexibility to the desired goal location. Should be able to choose any (lat, lon) within the map and have it rendered to a goal location on the grid.
The following is the code for transfer goal location to grid goal location.
  1. Transfer global goal location to local goal location. \
`goal_local_position = global_to_local(self.global_goal_position, self.global_home)`

  2. Calculate grid goal position by adding offset to local goal position\
`grid_goal = (int(goal_local_position[0]-north_offset), int(goal_local_position[1]-east_offset))`

#### 5. Modify A* to include diagonal motion (or replace A* altogether)
Minimal requirement here is to modify the code in planning_utils() to update the A* implementation to include diagonal motions on the grid that have a cost of sqrt(2), but more creative solutions are welcome. Explain the code you used to accomplish this step.

  1. Modified code in file `motion_planning2.py`. The following code provides A-start search method capability to search diagnoal motions.\
  Add following code into class Action(Enum)\
   `NORTH_WEST = (-1, -1, np.sqrt(2))`\
   `NORTH_EAST = (-1, 1, np.sqrt(2))`\
   `SOUTH_WEST = (1, -1, np.sqrt(2))`\
   `SOUTH_EAST = (1, 1, np.sqrt(2))`
   
  Add following code into function `valid_actions()`\
    `if (x - 1 < 0 or y - 1 < 0) or grid[x - 1, y - 1] == 1:`\
        &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; `valid_actions.remove(Action.NORTH_WEST)`\
    `if (x - 1 < 0 or y + 1 > m) or grid[x - 1, y + 1] == 1:`\
        &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; `valid_actions.remove(Action.NORTH_EAST)`\
    `if (x + 1 > n or y - 1 < 0) or grid[x + 1, y - 1] == 1:`\
        &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; `valid_actions.remove(Action.SOUTH_WEST)`\
    `if (x + 1 > n or y + 1 > m) or grid[x + 1, y + 1] == 1:`\
        &nbsp;&nbsp;&nbsp;&nbsp;&nbsp;&nbsp; `valid_actions.remove(Action.SOUTH_EAST)`
        
  2. Add following code to transfer grid to skeleton map\
`skeleton = medial_axis(invert(grid))`

  3. In skeleton map, search the nearest position of grid start and grid goal\
`near_start, near_goal = find_start_goal(skeleton, grid_start, grid_goal)`

  4. Use A-star method to find a path. If there is no path, use defaul goal position to generate waypoints

  5. Add two functions collinearity_prune() and bresenham_prune() to take out unnecessary waypoints. 

  6. In main(), add function that will randomly pick a grid goal position in the map. This grid goal position will be feed into drone object.\
  `drone = MotionPlanning(conn, global_goal)`

#### 6. Cull waypoints 
For this step you can use a collinearity test or ray tracing method like Bresenham. The idea is simply to prune your path of unnecessary waypoints. Explain the code you used to accomplish this step.
  1. prune path by function collinearity_prune() 
  2. prune path by function bresenham_prune()


### Execute the flight
#### 1. Does it work?
It works!

### Double check that you've met specifications for each of the [rubric](https://review.udacity.com/#!/rubrics/1534/view) points.
  
# Extra Challenges: Real World Planning

For an extra challenge, consider implementing some of the techniques described in the "Real World Planning" lesson. You could try implementing a vehicle model to take dynamic constraints into account, or implement a replanning method to invoke if you get off course or encounter unexpected obstacles.


