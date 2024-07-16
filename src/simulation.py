"""
Simple lidar & vehicle simulation
Author: Chris Goodin, cgoodin@cavs.msstate.edu

This is a stand-in for a more realistic simulator like VxSim or MAVS.
It's here to demonstrate how to connect a simulator to the autonomy stack.
Simulates lidar in 3D, with the world represented by axis-aligned boundng boxes (ie Minecraft style).
Simulates the vehicle in 2D using the "bicycle" model with 3 degrees of freedom (x,y, heading).

By default, the vehicle roughly represents a MRZR-D4 and the lidar a VLP-16
"""
# import python modules
import math, time, sys
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
# import Vector3 class
from autonomy_msgs import Vector3 

class Intersection(object):
    def __init__(self, dist, intersected):
        self.dist = dist
        self.intersected = intersected
   
class Aabb(object):
    def __init__(self):
        self.llc = Vector3([0.0, 0.0, 0.0])
        self.urc = Vector3([0.0, 0.0, 0.0])
    def Intersect(self, ro, inv_rd):
        # see: https://www.scratchapixel.com/lessons/3d-basic-rendering/minimal-ray-tracer-rendering-simple-shapes/ray-box-intersection.html
        tmin = (self.llc.x - ro.x) * inv_rd.x
        tmax = (self.urc.x - ro.x) * inv_rd.x 
        if (tmin > tmax):
            tmin, tmax = tmax, tmin
        tymin = (self.llc.y - ro.y) * inv_rd.y 
        tymax = (self.urc.y - ro.y) * inv_rd.y 
        if (tymin > tymax):
            tymin, tymax = tymax, tymin
        if ((tmin > tymax) or (tymin > tmax)):
            return Intersection(-1.0, False) 
        if (tymin > tmin):
            tmin = tymin 
        if (tymax < tmax):
            tmax = tymax 
        tzmin = (self.llc.z - ro.z) * inv_rd.z
        tzmax = (self.urc.z - ro.z) * inv_rd.z; 
        if (tzmin > tzmax):
            tzmin, tzmax = tzmax, tzmin
        if ((tmin > tzmax) or (tzmin > tmax)):
            return Intersection(-1.0, False)
        if (tzmin > tmin):
            tmin = tzmin 
        if (tzmax < tmax):
            tmax = tzmax; 
        return Intersection(tmin, True)
        
class Scene(object):
    def __init__(self):
        self.objects = []
        floor = Aabb()
        floor.llc = Vector3([-10000.0,-10000.0,-0.01])
        floor.urc = Vector3([10000.0,10000.0,0.0])
        self.objects.append(floor)
    def AddObject(self,llc,urc):
        obj = Aabb()
        obj.llc = Vector3([llc[0], llc[1], llc[2]])
        obj.urc = Vector3([urc[0], urc[1], urc[2]])
        self.objects.append(obj)
    def GetClosestIntersection(self, ro, inv_rd):
        tclose = sys.float_info.max
        intersected = False
        for i in range(len(self.objects)):
            inter = self.objects[i].Intersect(ro, inv_rd)
            if (inter.intersected and inter.dist<tclose):
                tclose = inter.dist
                intersected = True
        inter = Intersection(tclose, intersected)
        return inter
        
class Lidar(object):
    def __init__(self, vlo_deg=-15.0, vhi_deg=0.0, num_v_steps=16, num_h_steps=512):
        self.max_range = 110.0
        self.scan_directions = [Vector3()]*(num_h_steps*num_v_steps)
        self.inv_scan_directions = [Vector3()]*(num_h_steps*num_v_steps)
        kDegToRad = math.pi/180.0
        hfov_low = kDegToRad*(-180.0)
        hfov_high = kDegToRad*180.0
        hres = 0.0
        if (num_h_steps > 1):
            hres = (hfov_high - hfov_low) / (num_h_steps - 1)
        vfov_low = kDegToRad*vlo_deg
        vfov_high = kDegToRad*vhi_deg
        vres = 0.0
        if (num_v_steps > 1):
            vres = (vfov_high - vfov_low) / (num_v_steps - 1)
        n = 0
        for i in range(num_h_steps):
            alpha = hfov_low + i * hres
            for j in range (num_v_steps):
                omega = vfov_low + j * vres
                self.scan_directions[n] = Vector3([math.cos(omega)*math.cos(alpha), math.cos(omega)*math.sin(alpha), math.sin(omega)] )
                dx = self.scan_directions[n].x
                dy = self.scan_directions[n].y
                dz = self.scan_directions[n].z
                if (dx==0.0):
                    dx = 1.0E-15
                if (dy==0.0):
                    dy = 1.0E-15
                if (dz==0.0):
                    dz = 1.0E-5
                self.inv_scan_directions[n] = Vector3( [1.0/dx, 1.0/dy, 1.0/dz] )
                n = n + 1     
    def Scan(self, position, scene):
        points = []
        for i in range(len(self.scan_directions)):
            inter = scene.GetClosestIntersection(position, self.inv_scan_directions[i])
            if (inter.intersected and inter.dist<self.max_range):
                point = position + self.scan_directions[i].scale(inter.dist)
                points.append(point)
        return points

class Vehicle(object):
    """
    see: "Trajectory Tracking Control for a Kinematic Bicycle Model", Amit Ailon and Shai Arogeti
    2020 28th Mediterranean Conference on Control and Automation (MED)
    https://ieeexplore.ieee.org/stamp/stamp.jsp?arnumber=9183161
    """
    def __init__(self, lr=1.57, lf=1.15, mass=1210, max_motor_force=60000, max_steer_angle=0.45, drag_coeff=0.125, width=1.35):
        self.heading = 0.0 #aka phi, radians
        self.x = 0.0
        self.y = 0.0
        self.speed = 0.0 # aka v, m/s
        self.acceleration = 0.0 #aka delta
        self.lf = lf # positive distance from front axle to c.g.
        self.lr = lr # positive distance from rear axle to c.g.
        self.length = lf + lr
        self.steer_coeff = lr/(lf + lr)
        self.mass = mass
        self.max_motor_force = max_motor_force
        self.acc_coeff = self.max_motor_force/self.mass
        self.max_steer_angle = max_steer_angle
        self.drag_coeff = drag_coeff
        self.width = width
    def Update(self, throttle, steering, dt):
        # get the current steering angle
        delta = steering*self.max_steer_angle
        # do a simple linear electric motor
        accel = max(0.0, throttle*self.acc_coeff - self.drag_coeff*self.speed*self.speed)
        # set the steering related variables
        beta = math.atan(self.steer_coeff*math.tan(delta))
        pb = beta + self.heading
        sb = math.sin(beta)
        # calculate the deltas
        dx = self.speed*math.cos(pb)
        dy = self.speed*math.sin(pb)
        dphi = (self.speed/self.lr)*sb
        # update the state
        self.x = self.x + dt*dx
        self.y = self.y + dt*dy
        self.heading = self.heading + dt*dphi
        self.speed = self.speed + dt*accel

class Simulation(object):
    def __init__(self):
        self.vehicle = Vehicle()
        self.scene = Scene()
        self.lidar = Lidar()
        self.lidar_points = []
        self.lidar_height = 2.0
        plt.ion()
        self.fig, self.ax = plt.subplots()
        self.fig.canvas.draw()
        self.fig.canvas.manager.set_window_title('Simulation')
        self.elapsed_time = 0.0
        self.max_time = 60.0
        self.dt = 0.01
        self.nsteps = 0
        self.lidar_update_steps = 10
        self.world_limits = [[-100.0, -100.0],[100.0, 100.0]]
        self.display_debug = False
        self.lock_to_real_time = False
    def __bool__(self):
        return self.IsValid()
    def IsValid(self):
        if (self.elapsed_time>=self.max_time):
            return False
        if (self.vehicle.x<self.world_limits[0][0]):
            return False
        if (self.vehicle.x>self.world_limits[1][0]):
            return False
        if (self.vehicle.y<self.world_limits[0][1]):
            return False
        if (self.vehicle.y>self.world_limits[1][1]):
            return False
        return True
    def Update(self, throttle, steering):
        t0 = time.time()
        self.vehicle.Update(throttle, steering, self.dt)
        if (self.nsteps%self.lidar_update_steps==0):
            
            lidar_pos = Vector3([self.vehicle.x, self.vehicle.y, self.lidar_height])
            self.lidar_points = self.lidar.Scan(lidar_pos, self.scene)
            if (self.display_debug):
                self.Draw()
        self.elapsed_time = self.elapsed_time + self.dt
        self.nsteps = self.nsteps + 1
        if self.lock_to_real_time:
            while (time.time()-t0<self.dt):
                pass
    def Draw(self):
        self.ax.clear()
        dx = self.vehicle.length*math.cos(self.vehicle.heading)
        dy = self.vehicle.length*math.sin(self.vehicle.heading)
        self.ax.set_xlim(self.world_limits[0][0], self.world_limits[1][0])
        self.ax.set_ylim(self.world_limits[0][1], self.world_limits[1][1])
        self.ax.set_xlabel('X (m)')
        self.ax.set_ylabel('Y (m)')
        time_string = "{:.2f}".format(self.elapsed_time)
        self.ax.annotate("Sim Time: "+time_string,[0.9*self.world_limits[0][0], 0.9*self.world_limits[1][1]])
        self.ax.arrow(self.vehicle.x-0.5*dx, self.vehicle.y-0.5*dy, dx, dy, head_width = self.vehicle.width, width = self.vehicle.width)
        for iob in range(1,len(self.scene.objects)):
            self.ax.add_patch(Rectangle((self.scene.objects[iob].llc.x, self.scene.objects[iob].llc.y), self.scene.objects[iob].urc.x-self.scene.objects[iob].llc.x, self.scene.objects[iob].urc.y-self.scene.objects[iob].llc.y, edgecolor = 'black',facecolor = 'red',fill=True,lw=0.5))
        x = []
        y = []
        for p in self.lidar_points:
            x.append(p.x)
            y.append(p.y)
        self.ax.plot(x,y,'.',markersize=1)
        self.fig.canvas.flush_events()
        
if __name__ == "__main__":
    sim = Simulation()
    sim.display_debug = True
    sim.lidar_height = 1.5
    sim.world_limits = [[-50.0,-50.0],[50.0, 50.0]]
    sim.max_time = 35.0 # seconds
    
    # set up the lane walls
    sim.scene.AddObject([-25.0,-4.25,0.0],[25.0,-4.0,1.0])
    sim.scene.AddObject([-25.0,4.0,0.0],[25.0,4.25,1.0])
    
    # put the vehicle in the starting location
    sim.vehicle.x = -45.0
    sim.vehicle.y = 0.0
    sim.vehicle.heading = 0.0
    
    sim.lock_to_real_time = True
    while sim.IsValid():
        throttle = 0.25
        steering = 0.0
        sim.Update(throttle, steering)
