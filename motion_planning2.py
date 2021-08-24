import argparse
import time
import msgpack
from enum import Enum, auto

import numpy as np
import csv

from planning_utils2 import a_star, heuristic, create_grid, collinearity_int, collinearity_prune, bresenham_prune
from planning_utils2 import find_start_goal
from udacidrone import Drone
from udacidrone.connection import MavlinkConnection
from udacidrone.messaging import MsgID
from udacidrone.frame_utils import global_to_local
from bresenham import bresenham
from skimage.morphology import medial_axis
from skimage.util import invert
import utm

class States(Enum):
    MANUAL = auto()
    ARMING = auto()
    TAKEOFF = auto()
    WAYPOINT = auto()
    LANDING = auto()
    DISARMING = auto()
    PLANNING = auto()


class MotionPlanning(Drone):

    def __init__(self, connection, goal):
        super().__init__(connection)

        self.target_position = np.array([0.0, 0.0, 0.0])
        self.waypoints = []
        self.in_mission = True
        self.check_state = {}
        self.global_goal_position = goal # (lon, lat, att)

        # initial state
        self.flight_state = States.MANUAL

        # register all your callbacks here
        self.register_callback(MsgID.LOCAL_POSITION, self.local_position_callback)
        self.register_callback(MsgID.LOCAL_VELOCITY, self.velocity_callback)
        self.register_callback(MsgID.STATE, self.state_callback)

    def local_position_callback(self):
        if self.flight_state == States.TAKEOFF:
            if -1.0 * self.local_position[2] > 0.95 * self.target_position[2]:
                self.waypoint_transition()
        elif self.flight_state == States.WAYPOINT:
            if np.linalg.norm(self.target_position[0:2] - self.local_position[0:2]) < 1.0:
                if len(self.waypoints) > 0:
                    self.waypoint_transition()
                else:
                    if np.linalg.norm(self.local_velocity[0:2]) < 1.0:
                        self.landing_transition()

    def velocity_callback(self):
        if self.flight_state == States.LANDING:
            if self.global_position[2] - self.global_home[2] < 0.1:
                if abs(self.local_position[2]) < 0.01:
                    self.disarming_transition()

    def state_callback(self):
        if self.in_mission:
            if self.flight_state == States.MANUAL:
                self.arming_transition()
            elif self.flight_state == States.ARMING:
                if self.armed:
                    self.plan_path()
            elif self.flight_state == States.PLANNING:
                self.takeoff_transition()
            elif self.flight_state == States.DISARMING:
                if ~self.armed & ~self.guided:
                    self.manual_transition()

    def arming_transition(self):
        self.flight_state = States.ARMING
        print("arming transition")
        self.arm()
        self.take_control()

    def takeoff_transition(self):
        self.flight_state = States.TAKEOFF
        print("takeoff transition")
        self.takeoff(self.target_position[2])

    def waypoint_transition(self):
        self.flight_state = States.WAYPOINT
        print("waypoint transition")
        self.target_position = self.waypoints.pop(0)
        print('target position', self.target_position)
        self.cmd_position(self.target_position[0], self.target_position[1], self.target_position[2], self.target_position[3])

    def landing_transition(self):
        self.flight_state = States.LANDING
        print("landing transition")
        self.land()

    def disarming_transition(self):
        self.flight_state = States.DISARMING
        print("disarm transition")
        self.disarm()
        self.release_control()

    def manual_transition(self):
        self.flight_state = States.MANUAL
        print("manual transition")
        self.stop()
        self.in_mission = False

    def send_waypoints(self):
        print("Sending waypoints to simulator ...")
        data = msgpack.dumps(self.waypoints)
        self.connection._master.write(data)

    def plan_path(self):
        self.flight_state = States.PLANNING
        print("Searching for a path ...")
        TARGET_ALTITUDE = 5.0
        SAFETY_DISTANCE = 8.0

        self.target_position[2] = TARGET_ALTITUDE

        # TODO: read lat0, lon0 from colliders into floating point values
        path = './colliders.csv'
        with open(path, 'r') as f:
            reader = csv.reader(f, delimiter=',')
            headers = next(reader)

        # split first line into individual string and convert string to float
        lat0 = float(headers[0].split()[1])
        lon0 = float(headers[1].split()[1])
        
        # TODO: set home position to (lon0, lat0, 0)
        self.set_home_position(lon0, lat0, 0)

        # TODO: retrieve current global position
        # TODO: convert to current local position using global_to_local()
        local_position = global_to_local(self.global_position, self.global_home) 
        print('local position = ', local_position)
        
        print('global home {0}, position {1}, local position {2}'.format(self.global_home, self.global_position,
                                                                         self.local_position))
        
        # Read in obstacle map
        data = np.loadtxt('colliders.csv', delimiter=',', dtype='Float64', skiprows=2)

        # Define a grid for a particular altitude and safety margin around obstacles
        grid, north_offset, east_offset = create_grid(data, TARGET_ALTITUDE, SAFETY_DISTANCE)
        print("North offset = {0}, east offset = {1}".format(north_offset, east_offset))
        
        # transfer grid to skelton
        skeleton = medial_axis(invert(grid))
        print("transfer grid to skeleton")
        
        # Define starting point on the grid (this is just grid center)
#         grid_start = (-north_offset, -east_offset)
        # TODO: convert start position to current position rather than map center
        grid_start = (int(local_position[0]-north_offset), int(local_position[1]-east_offset))
        
        # Set goal as some arbitrary position on the grid
#         grid_goal = (int(grid_start[0] + 10), int(grid_start[1]-10))
        # TODO: adapt to set goal as latitude / longitude position and convert
        goal_local_position = global_to_local(self.global_goal_position, self.global_home) 
        grid_goal = (int(goal_local_position[0]-north_offset), int(goal_local_position[1]-east_offset))
    
        # find near_start and near_goal
        near_start, near_goal = find_start_goal(skeleton, grid_start, grid_goal)
        print('Local Start and Goal: ', grid_start, grid_goal)
        print('Near Start and Near Goal: ', near_start, near_goal)
        
        # Run A* to find a path from start to goal
        # TODO: add diagonal motions with a cost of sqrt(2) to your A* implementation
        # or move to a different search space such as a graph (not done here)
#         path, _ = a_star(grid, heuristic, grid_start, grid_goal)
        path, _ = a_star(invert(skeleton).astype(np.int), heuristic, tuple(near_start), tuple(near_goal))
        
        # TODO: prune path to minimize number of waypoints
        # TODO (if you're feeling ambitious): Try a different approach altogether!
        # apply collinearity to prune the path
        if len(path)>0:
            # prune path by collinearity
            path_prune = collinearity_prune(path)
        
            # appy bresenham check to prune the path again
            path_prune = bresenham_prune(grid, path_prune)
            
        else:
            # go back to default start position
            grid_goal = (450, 400)
            print("back to predefined grid position", grid_goal)
            
            # find near start and near goal
            near_start, near_goal = find_start_goal(skeleton, grid_start, grid_goal)
            
            # find path 
#             path, _ = a_star(grid, heuristic, grid_start, grid_goal)
            path, _ = a_star(invert(skeleton).astype(np.int), heuristic, tuple(near_start), tuple(near_goal))
            
            # prune path by collinearity
            path_prune = collinearity_prune(path)
        
            # appy bresenham check to prune the path again
            path_prune = bresenham_prune(grid, path_prune)
        
        # Convert path to waypoints
        waypoints = [[p[0] + north_offset, p[1] + east_offset, TARGET_ALTITUDE, 0] for p in path_prune]
        # Set self.waypoints
        self.waypoints = waypoints
        # TODO: send waypoints to sim (this is just for visualization of waypoints)
        self.send_waypoints()
               

    def start(self):
        self.start_log("Logs", "NavLog.txt")

        print("starting connection")
        self.connection.start()

        # Only required if they do threaded
        # while self.in_mission:
        #    pass

        self.stop_log()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--port', type=int, default=5760, help='Port number')
    parser.add_argument('--host', type=str, default='127.0.0.1', help="host address, i.e. '127.0.0.1'")
    args = parser.parse_args()

    # Read in obstacle map
    data = np.loadtxt('colliders.csv', delimiter=',', dtype='Float64', skiprows=2)

    # create grid
    TARGET_ALTITUDE = 5.0
    SAFETY_DISTANCE = 8.0
    grid, north_offset, east_offset = create_grid(data, TARGET_ALTITUDE, SAFETY_DISTANCE)

    # assuming start position is always at center
    grid_start = (-north_offset, -east_offset)
    # grid_goal =  (377, 468)
    # random select start cell which is not obstacle
    # north = np.random.uniform(0, grid.shape[0])
    # east = np.random.uniform(0, grid.shape[1])
    # grid_start = (int(north),int(east))
    # while grid[grid_start[0], grid_start[1]] == 1: 
    #     north = np.random.uniform(0, grid.shape[0])
    #     east = np.random.uniform(0, grid.shape[1])
    #     grid_start = (int(north),int(east))

    # random select goal cell which is not obstacle
    north = np.random.uniform(0, grid.shape[0])
    east = np.random.uniform(0, grid.shape[1])
    grid_goal = (int(north),int(east))
    while grid[grid_goal[0], grid_goal[1]] == 1 or grid_goal == grid_start or grid_goal[0]>650 or grid_goal[1]>650:
        north = np.random.uniform(0, grid.shape[0])
        east = np.random.uniform(0, grid.shape[1])
        grid_goal = (int(north),int(east))
        
    # transfer grid to skelton
    skeleton = medial_axis(invert(grid))
    near_start, near_goal = find_start_goal(skeleton, grid_start, grid_goal)

    # transfer near_goal to (lon0, lat0)
    global_home = (-122.397450, 37.792480, 0) # (long, lat, att)
    (east_home, north_home, zone_number, zone_letter) = utm.from_latlon(global_home[1], global_home[0]) # get NED home
    goal_local_position = (near_goal[0]+north_offset, near_goal[1]+east_offset) # (north, east) w.r.t map center
    goal_NED_position = (goal_local_position[0]+north_home, goal_local_position[1]+east_home) 
    (lat, lon) = utm.to_latlon(goal_NED_position[1], goal_NED_position[0], zone_number, zone_letter)
    global_goal = (lon, lat, TARGET_ALTITUDE)

    # print grid goal and grid start
    print("grid_start = ", grid_start)
    print("grid_goal = ", grid_goal)
    print("near start = ", near_start)
    print("near goal = ", near_goal)
    print("global start = (lon, lat)", global_home)
    print("global goal = (lon, lat)", global_goal)

    # test reverse from goal global position to grid_goal results the same grid_goal coordinate selected
    # goal_global_position = (global_goal[0], global_goal[1], TARGET_ALTITUDE)  # (lon, lat, att)
    goal_local_position = global_to_local(global_goal, global_home) 
    near_goal2 = (int(goal_local_position[0]-north_offset), int(goal_local_position[1]-east_offset))
    print(near_goal2)

    # initiate connection and create drone object
    conn = MavlinkConnection('tcp:{0}:{1}'.format(args.host, args.port), timeout=60)
    # global_goal = (-122.39908964612614, 37.796174728538944, 5.0)
    drone = MotionPlanning(conn, global_goal)
    time.sleep(1)
    drone.start()
