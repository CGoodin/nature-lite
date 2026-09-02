"""
A class for a simple MAVS sim with a scene, vehicle, lidar, and camera
Using the MAVS simulator (https://github.com/Mississippi-State-University-OTM/MAVS)
NOTE: This will only work if you have already installed MAVS on your computer
"""
from autonomy.autonomy_msgs import Odometry
from simulation.simulation import Simulation
import sys
# change the following line to the location of the mavs_python folder on your computer
sys.path.append(r'C:/your/full/path/to/mavs/src/mavs_python')
import mavs_interface as mavs
import mavs_python_paths
mavs_data_path = mavs_python_paths.mavs_data_path # Set the path to the mavs data folder

class MavsSimulation(Simulation):
    def __init__(self):
        # create and load a mavs scnee
        self.scene = mavs.MavsEmbreeScene()
        mavs_scenefile = "/scenes/odoa_scene_no_trees.json"
        #mavs_scenefile = "/scenes/obstacle_field.json"
        self.scene.Load(mavs_data_path+mavs_scenefile)

        # create a MAVS environment and add the scene to it
        self.env = mavs.MavsEnvironment()
        self.env.SetScene(self.scene)
        self.env.SetTime(13) # 0-23
        self.env.SetFog(20.0) # 0.0-100.0
        self.env.SetTurbidity(7.0) # 2-10

        # create and load a MAVS vehicle
        self.veh = mavs.MavsRp3d()
        veh_file = 'mrzr4_tires_low_gear.json'
        self.veh.Load(mavs_data_path+'/vehicles/rp3d_vehicles/' + veh_file)
        self.veh.SetInitialPosition(-45.0, 0.0, 2.0) # Starting point for the vehiclein global ENU
        self.veh.SetInitialHeading(0.0) # Initial Heading for the vehicle in radians north of east,
        #veh.Update(env, 0.0, 0.0, 1.0, 0.000001)

        # create a MAVS camera
        self.drive_cam = mavs.MavsCamera()
        self.drive_cam.Initialize(512,512,0.0035,0.0035,0.0035)
        self.drive_cam.SetOffset([-10.0,0.0,3.0],[1.0,0.0,0.0,0.0]) # offset of camera from vehicle CG
        self.drive_cam.SetGammaAndGain(0.75,2.0) # Set camera compression and gain
        self.drive_cam.RenderShadows(True)

        # create a MAVS lidar
        self.lidar = mavs.MavsLidar('VLP-16')
        self.lidar.SetOffset([0.0, 0.0, 2.0],[1.0, 0.0, 0.0, 0.0])

        # sim timing variables
        self.dt = 0.01 # 100 Hz
        self.loop_counter = 0

    def IsValid(self):
        valid = True
        if (self.loop_counter > 100000000):
            valid = False
        return valid

    def GetVehicleStateAsOdometry(self) -> Odometry:
        position,orientation,linear_velocity,angular_velocity,linear_acceleration,angular_acceleration = self.veh.GetFullState()
        odom = Odometry()
        odom.pose.position.x = position[0]
        odom.pose.position.y = position[1]
        odom.pose.position.z = position[2]
        odom.pose.orientation.w = orientation[0]
        odom.pose.orientation.x = orientation[1]
        odom.pose.orientation.y = orientation[2]
        odom.pose.orientation.z = orientation[3]
        odom.twist.linear.x = linear_velocity[0]
        odom.twist.linear.y = linear_velocity[1]
        odom.twist.linear.z = linear_velocity[2]
        odom.twist.angular.x = angular_velocity[0]
        odom.twist.angular.y = angular_velocity[1]
        odom.twist.angular.z = angular_velocity[2]
        return odom

    def GetPositionSpeedHeading(self):
        return self.veh.GetPosition(), self.veh.GetSpeed(), self.veh.GetHeading()

    def GetPoints(self):
        return self.lidar.GetPoints()
    
    def Update(self, throttle, steering, braking):

        # Update the mavs vehicle at 100 Hz
        self.veh.Update(self.env, throttle, steering, braking, self.dt)
    
        # update the camera at 20 Hz and the
        # Lidar at 10 Hz
        if self.loop_counter%5==0 and self.loop_counter>0:
            orientation = self.veh.GetOrientation()
            position = self.veh.GetPosition()
            # update the MAVS camera
            self.drive_cam.SetPose(position, orientation)
            self.drive_cam.Update(self.env,0.1)
            self.drive_cam.Display()
            if self.loop_counter%10==0:
                # Update the MAVS lidar 
                self.lidar.SetPose(position, orientation)
                self.lidar.Update(self.env,0.1)
        # Update the loop counter
        self.loop_counter = self.loop_counter + 1