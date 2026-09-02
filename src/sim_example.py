"""
Script demonstrating a closed-loop autonomous navigation simulation.
The simulator creates the lidar point cloud and vehicle movement.
The autonomy stack uses perception, planning, and control modules
to automatically avoid an obstacle and proceed to a goal point.
"""
# import the autonomy modules
from autonomy_stack import AutonomyStack

# -------- Simulation setup, replace with simulator of your choice -----------#
# simpl simulation with lidar and vehicle but no rendering
from simulation.simple_sim import SimpleSimulation
sim = SimpleSimulation()

# mavs 3D simulation, uncomment if you have MAVS installed
#from simulation.mavs_sim import MavsSimulation
#sim = MavsSimulation()
#-----------------------------------------------------------------------------#

stack = AutonomyStack()

while (not stack.planner.GoalReached()) and sim.IsValid():
    
    # Update the controller based on the vehicles current state at 100 Hz
    position, speed, heading = sim.GetPositionSpeedHeading()
    
    dc = stack.controller.GetDrivingCommand(position[0], position[1],speed, heading)

    # Update the simulation based on the output from the controller
    # will update the vehicle, lidar, and camera
    sim.Update(dc.throttle, dc.steering, dc.braking)

    # Simulation runs at 100 Hz, update the path planning at 20 Hz
    if sim.loop_counter%5==0 and sim.loop_counter>0:
        #update the perception at 10 Hz
        if sim.loop_counter%10:
            # Get lidar point cloud in to world coordinates and add them to the grid
            registered_points = sim.GetPoints() 
            stack.AddRegisteredPointsToOccupancyGrid(registered_points)  

        # update the planner using the current vehicle state
        odom = sim.GetVehicleStateAsOdometry()
        cmd_vel, trajectory = stack.planner.Update(odom, stack.goal, stack.grid)

        # update the controller using the cmd_vel and trajectory from the planner
        stack.UpdateController(cmd_vel, trajectory)
        
        # Display the current path, obstacles, position, and goal
        stack.Display(position)
