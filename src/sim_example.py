"""
Script demonstrating a closed-loop autonomous navigation simulation.
The simulator creates the lidar point cloud and vehicle movement.
The autonomy stack uses perception, planning, and control modules
to automatically avoid an obstacle and proceed to a goal point.
"""
# import the autonomy modules
from autonomy_msgs import OccupancyGrid
from potential_field import PotentialFieldPlanner
from pure_pursuit import PurePursuitController
from lidar_perception import SlopeMap
# import the simulator - can be replaced with a different sim if desired
from simulation import Simulation

# -------------- Simulation setup, replace with simulator of your choice -----------------#
# create the simulation
sim = Simulation()
sim.lidar_height = 1.5 # mount height on vehicle, above ground
sim.world_limits = [[-50.0,-50.0],[50.0, 50.0]] # ENU meters
sim.max_time = 35.0 # seconds
    
# set up the obstacle avoidiance scenario
sim.scene.AddObject([-25.0,-8.25,0.0],[25.0,-8.0,1.0]) # right wall
sim.scene.AddObject([-25.0,6.0,0.0],[25.0,6.25,1.0]) # left wall
sim.scene.AddObject([-0.25,-0.25,0.0],[0.25,0.25,1.5]) #obstacle
    
# put the vehicle in the starting location
sim.vehicle.x = -45.0
sim.vehicle.y = 0.0
sim.vehicle.heading = 0.0 # radians North of East

# turn on or off display
sim.display_debug = True
sim.lock_to_real_time = False
#----------------------------------------------------------------------------------------#

#------------- Create the autonomy modules -----------------------------------------------# 
# create the path planner
planner = PotentialFieldPlanner()
planner.k_attract = 25.0 # strength of repuslive potential
planner.k_repulse = 20.0 # strength of attractive potential
planner.obs_cutoff_dist = 30.0 # planning horizon
planner.goal_thresh_dist = 4.0
planner.SetGoal(45.0, 0.0)

# Create and occupancy grid for perception and resize it
grid = OccupancyGrid([150, 150]) # set the number of cells
grid.info.resolution = 2.0/3.0 # Set the resolution (in meters) of the grid
grid.set_origin(-50.0,-50.0) # Set the origin of the grid (lower left corner), ENU meters
# create the perception module
perception = SlopeMap([grid.info.width, grid.info.height])
perception.slope_thresh = 0.5 # Set the slope threshold for an obstacle
perception.inflation = 1; # This will inflate the size of obstacles, default is zero

# create the vehicle controller
controller = PurePursuitController()
controller.SetDesiredSpeed(5.0) # m/s
#----------------------------------------------------------------------------------------#

n = 0 # loop counter
while (not planner.GoalReached(sim.vehicle.x, sim.vehicle.y)) and sim.IsValid():
    
    # Update the driving command using the controller based on the vehicles current state
    dc = controller.GetDrivingCommand(sim.vehicle.x,sim.vehicle.y, sim.vehicle.speed, sim.vehicle.heading)

    # Update the vehicle
    sim.Update(dc.throttle, dc.steering)

    # Simulation runs at 100 Hz, update the perception and path at 10 Hz
    if n%10==0 and n>0:
       
        registered_points = sim.lidar_points  # Get lidar point cloud registered to world coordinates
       
        perception.add_registered_points(registered_points, grid)  # add the points to the grid

        path, path_enu = planner.Plan(grid,sim.vehicle.x,sim.vehicle.y) # calculate the vehicle path

        if path:
            controller.SetDesiredPath(path_enu) # give the new path to the controller
        
        # Display debug info to the screen
        perception.Display(path, grid.coordinate_to_index(sim.vehicle.x, sim.vehicle.y), grid.coordinate_to_index(planner.goal[0], planner.goal[1]), grid)

    # Update the loop counter
    n = n + 1