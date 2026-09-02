"""
Script setting up a full autonomy stack with a controller,
lidar perception, and path planning
"""
# import python modules
import math
import numpy as np
# import the autonomy modules from this project
from autonomy.autonomy_msgs import OccupancyGrid, Twist, Vector3, Path
from autonomy.planning.potential_field import PotentialFieldPlanner
from autonomy.planning.dwa_planner import DwaPlanner, RobotType
from autonomy.control.pure_pursuit import PurePursuitController
from autonomy.perception.lidar_perception import SlopeMap

class AutonomyStack(object):
    def __init__(self):
        #--------- Create the autonomy modules ----------------------------# 
        self.goal = Vector3([45.0, 0.0, 0.0])
        # create the path planners
        self.pf_planner = PotentialFieldPlanner()
        self.pf_planner.k_attract = 25.0 # strength of repuslive potential
        self.pf_planner.k_repulse = 20.0 # strength of attractive potential
        self.pf_planner.obs_cutoff_dist = 20.0 # planning horizon
        self.pf_planner.goal_thresh_dist = 4.0

        self.dwa_planner = DwaPlanner(robot_type=RobotType.rectangle)
        self.dwa_planner.config.max_speed = 10.0
        self.dwa_planner.config.max_accel = 3.0
        self.dwa_planner.config.max_delta_yaw_rate = 90.0 * math.pi / 180.0  # [rad/ss]
        self.dwa_planner.max_yaw_rate = 180.0 * math.pi / 180.0  # [rad/s]
        self.dwa_planner.config.v_resolution = 0.2  # [m/s]
        self.dwa_planner.config.yaw_rate_resolution = 0.5 * math.pi / 180.0  # [rad/s] 
        self.dwa_planner.config.dt = 0.1  # [s] Time tick for motion prediction
        self.dwa_planner.config.predict_time = 3.0  # [s]
        self.dwa_planner.config.to_goal_cost_gain = 3.0 #0.15
        self.dwa_planner.config.speed_cost_gain = 0.5
        self.dwa_planner.config.obstacle_cost_gain = 1.0
        self.dwa_planner.smoothness_cost_gain = 0.3
        self.dwa_planner.config.robot_stuck_flag_cons = 0.075  # constant prevent sticking
        # if robot_type == RobotType.rectangle
        self.dwa_planner.config.robot_width = 2.0 # [m] for collision check
        self.dwa_planner.config.robot_length = 4.0  # [m] for collision check
        self.planner = self.dwa_planner
        self.planner.SetGoal(self.goal.x, self.goal.y)

        # Create and occupancy grid for perception and resize it
        self.grid = OccupancyGrid([200, 200]) # set the number of cells
        self.grid.info.resolution = 0.5 # Set the resolution (in meters) of the grid
        self.grid.set_origin(-50.0,-50.0) # Set the origin of the grid (lower left corner), ENU meters
        # create the perception module
        self.perception = SlopeMap([self.grid.info.width, self.grid.info.height])
        self.perception.slope_thresh = 0.5 # Set the slope threshold for an obstacle
        self.perception.inflation = 1; # This will inflate the size of obstacles, default is zero

        # create the vehicle controller
        self.controller = PurePursuitController()
        self.controller.SetDesiredSpeed(5.0) # m/s
        self.controller.look_ahead_distance = 8.0
        self.controller.k = 2.0
        self.controller.wheelbase = 1.5

        self.current_path = []

    def AddRegisteredPointsToOccupancyGrid(self, registered_points):
        self.perception.add_registered_points(registered_points, self.grid)

    def TrajectoryToPath(self, trajectory : Path, grid : OccupancyGrid):
        path_enu = np.array([[pose.pose.position.x, pose.pose.position.y] for pose in trajectory.poses])
        path = []
        for point in path_enu:
            pidx = grid.coordinate_to_index(point[0], point[1])
            path.append(pidx)   
        return path, path_enu

    def UpdateController(self, cmd_vel : Twist, trajectory : Path):
        # convert the trajectory to a 2D python list
        path, path_enu = self.TrajectoryToPath(trajectory, self.grid)
        if path:
            # give the new path to the controller
            self.controller.SetDesiredPath(path_enu)
            self.current_path = path 
        speed = math.sqrt(cmd_vel.linear.x*cmd_vel.linear.x + cmd_vel.linear.y*cmd_vel.linear.y)
        #self.controller.SetDesiredSpeed(speed)

    def Display(self, position):
        if (len(self.current_path)>0):
            self.perception.Display(self.current_path, self.grid.coordinate_to_index(position[0], position[1]), self.grid.coordinate_to_index(self.planner.goal[0], self.planner.goal[1]), self.grid)

