"""
Script demonstrating a closed-loop autonomous navigation simulation
Using the MAVS simulator (https://www.cavs.msstate.edu/capabilities/mavs.php)
NOTE: This will only work if you have already installed MAVS on your computer
"""
# import the autonomy modules
from autonomy_msgs import OccupancyGrid
from potential_field import PotentialFieldPlanner
from pure_pursuit import PurePursuitController
from lidar_perception import SlopeMap
# import the MAVS simulator
import sys
# change the following line to the location of the mavs_python folder on your computer
sys.path.append(r'C:/your/full/path/to/mavs/src/mavs_python')
import mavs_interface as mavs
import mavs_python_paths
mavs_data_path = mavs_python_paths.mavs_data_path # Set the path to the mavs data folder

# -------------- MAVS Simulation setup -----------------------------------------------------#
# create and load a mavs scnee
scene = mavs.MavsEmbreeScene()
mavs_scenefile = "/scenes/forester_scene_less_trees_simple.json"
scene.Load(mavs_data_path+mavs_scenefile)

# create a MAVS environment and add the scene to it
env = mavs.MavsEnvironment()
env.SetScene(scene)
env.SetTime(13) # 0-23
env.SetFog(20.0) # 0.0-100.0
env.SetTurbidity(7.0) # 2-10

# create and load a MAVS vehicle
veh = mavs.MavsRp3d()
veh_file = 'mrzr4_tires_low_gear.json'
veh.Load(mavs_data_path+'/vehicles/rp3d_vehicles/' + veh_file)
veh.SetInitialPosition(-45.0, 0.0, 2.0) # Starting point for the vehiclein global ENU
veh.SetInitialHeading(0.0) # Initial Heading for the vehicle in radians north of east,
veh.Update(env, 0.0, 0.0, 1.0, 0.000001)

# create a MAVS camera
drive_cam = mavs.MavsCamera()
drive_cam.Initialize(512,512,0.0035,0.0035,0.0035)
drive_cam.SetOffset([-10.0,0.0,3.0],[1.0,0.0,0.0,0.0]) # offset of camera from vehicle CG
drive_cam.SetGammaAndGain(0.75,2.0) # Set camera compression and gain
drive_cam.RenderShadows(True)

# create a MAVS lidar
lidar = mavs.MavsLidar('VLP-16')
lidar.SetOffset([0.0, 0.0, 2.0],[1.0, 0.0, 0.0, 0.0])
#----------------------------------------------------------------------------------------#

#------------- Create the autonomy modules -----------------------------------------------# 
# create the path planner
planner = PotentialFieldPlanner()
planner.k_attract = 25.0 # strength of repuslive potential
planner.k_repulse = 20.0 # strength of attractive potential
planner.obs_cutoff_dist = 20.0 # planning horizon
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
controller.look_ahead_distance = 8.0
controller.k = 2.0
controller.wheelbase = 1.5
#----------------------------------------------------------------------------------------#

n = 0 # loop counter
while (not planner.GoalReached(veh.GetPosition()[0], veh.GetPosition()[1])):
    
    # Update the driving command using the controller based on the vehicles' current state
    position = veh.GetPosition()
    speed = veh.GetSpeed()
    heading = veh.GetHeading()
    dc = controller.GetDrivingCommand(position[0], position[1],speed, heading)

    # Update the mavs vehicle
    veh.Update(env, dc.throttle, dc.steering, 0.0, 0.01)
    
    # Simulation runs at 100 Hz, update the perception and path at 10 Hz
    if n%10==0 and n>0:
        orientation = veh.GetOrientation()
        # update the MAVS camera
        drive_cam.SetPose(position, orientation)
        drive_cam.Update(env,0.1)
        drive_cam.Display()
        
        # Update the MAVS lidar and get lidar point cloud registered to world coordinates
        lidar.SetPose(position, orientation)
        lidar.Update(env,0.1)
        registered_points = lidar.GetPoints() 
       
        perception.add_registered_points(registered_points, grid)  # add the points to the grid

        path, path_enu = planner.Plan(grid, position[0], position[1]) # calculate the vehicle path

        if path:
            controller.SetDesiredPath(path_enu) # give the new path to the controller
        
        # Display debug info to the screen
        perception.Display(path, grid.coordinate_to_index(position[0], position[1]), grid.coordinate_to_index(planner.goal[0], planner.goal[1]), grid)

    # Update the loop counter
    n = n + 1