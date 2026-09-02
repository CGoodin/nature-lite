"""

Mobile robot motion planning sample with Dynamic Window Approach

author: Atsushi Sakai (@Atsushi_twi), Göktuğ Karakaşlı

"""

import math
from enum import Enum
import matplotlib.pyplot as plt
import numpy as np
import sys
from autonomy.autonomy_msgs import Vector3, Odometry, OccupancyGrid, Twist, Path, Pose, PoseStamped
from autonomy.planning.planner import Planner

class RobotType(Enum):
    circle = 0
    rectangle = 1

class DwaConfig:
    """
    simulation parameter class
    """

    def __init__(self):
        # robot parameter
        self.max_speed = 1.0  # [m/s]
        self.min_speed = -0.5  # [m/s]
        self.max_yaw_rate = 40.0 * math.pi / 180.0  # [rad/s]
        self.max_accel = 0.2  # [m/ss]
        self.max_delta_yaw_rate = 40.0 * math.pi / 180.0  # [rad/ss]
        self.v_resolution = 0.01  # [m/s]
        self.yaw_rate_resolution = 0.1 * math.pi / 180.0  # [rad/s]
        self.dt = 0.1  # [s] Time tick for motion prediction
        self.predict_time = 3.0  # [s]
        self.to_goal_cost_gain = 0.15
        self.speed_cost_gain = 1.0
        self.obstacle_cost_gain = 1.0
        self.smoothness_cost_gain = 2.0
        self.robot_stuck_flag_cons = 0.001  # constant to prevent robot stucked
        self.robot_type = RobotType.circle

        # if robot_type == RobotType.circle
        # Also used to check if goal is reached in both types
        self.robot_radius = 1.0  # [m] for collision check
        
        self.goal_radius = 3.0

        # if robot_type == RobotType.rectangle
        self.robot_width = 0.5  # [m] for collision check
        self.robot_length = 1.2  # [m] for collision check
        # obstacles [x(m) y(m), ....]
        self.obstacles = np.array([])


    @property
    def robot_type(self):
        return self._robot_type

    @robot_type.setter
    def robot_type(self, value):
        if not isinstance(value, RobotType):
            raise TypeError("robot_type must be an instance of RobotType")
        self._robot_type = value

def StateFromOdom(odom_in : Odometry):
    state = [0.0, 0.0, 0.0, 0.0, 0.0]
    siny_cosp = 2.0 * (odom_in.pose.orientation.w * odom_in.pose.orientation.z + odom_in.pose.orientation.x * odom_in.pose.orientation.y)
    cosy_cosp = 1.0 - 2.0 * (odom_in.pose.orientation.y * odom_in.pose.orientation.y + odom_in.pose.orientation.z * odom_in.pose.orientation.z)
    
    state[2] = math.atan2(siny_cosp, cosy_cosp)
    state[0] = odom_in.pose.position.x
    state[1] = odom_in.pose.position.y
    state[3] = math.sqrt(odom_in.twist.linear.x*odom_in.twist.linear.x + odom_in.twist.linear.y* odom_in.twist.linear.y)
    state[4] = odom_in.twist.angular.z
    return state

def OdomFromState(state_in) -> Odometry:
    odom_out = Odometry()
    odom_out.pose.position.x = state_in[0]
    odom_out.pose.position.y = state_in[1]
    odom_out.twist.angular.z = state_in[4]
    odom_out.pose.orientation.w = math.cos(0.5*state_in[2])
    odom_out.pose.orientation.z = math.sin(0.5*state_in[2])
    odom_out.twist.linear.x = math.cos(state_in[2])*state_in[3]
    odom_out.twist.linear.y = math.sin(state_in[2])*state_in[3]
    return odom_out

def PoseFromState(state_in) -> Pose:
    pose_out = Pose()
    pose_out.position.x = state_in[0]
    pose_out.position.y = state_in[1]
    pose_out.orientation.w = math.cos(0.5*state_in[2])
    pose_out.orientation.z = math.sin(0.5*state_in[2])
    return pose_out

def OccupancyGridToObstacleList(grid : OccupancyGrid):
    obs_list = []
    for i in range(grid.info.width):
        for j in range(grid.info.height):
            if (grid.data[i,j]>0.01):
                obs = grid.index_to_coordinate(i,j)
                obs_list.append(obs)
    return obs_list

class DwaPlanner(Planner):
    def __init__(self, robot_type=RobotType.circle):
        self.finished = False
        self.show_animation = True
        self.goal = [10.0, 0.0]
        self.robot_state = np.array([0.0, 0.0, 0.0, 0.0, 0.0])
        self.config = DwaConfig()
        self.trajectory_history = np.array([])
        self.config.robot_type = robot_type
        self.current_u = None
        self.current_traj = None
        
    def SetObstacles(self, obstacles_in):
        self.config.obstacles = np.array(obstacles_in)

    def SetGoal(self, gx, gy):
        self.goal = [gx, gy]

    def DwaControl(self):
        """
        Dynamic Window Approach control
        """
        dw = self.CalcDynamicWindow()

        u, trajectory = self.CalcControlAndTrajectory(dw)

        return u, trajectory

    def Motion(self, x, u, dt):
        """
        motion model
        """
        # got to update them in THIS order
        x[2] += u[1] * dt # heading
        x[0] += u[0] * math.cos(x[2]) * dt # px
        x[1] += u[0] * math.sin(x[2]) * dt # py
        x[3] = u[0] # linear velocity
        x[4] = u[1] # angular velocity

        return x

    def CalcDynamicWindow(self):
        """
        calculation dynamic window based on current state x
        """

        # Dynamic window from robot specification
        Vs = [self.config.min_speed, self.config.max_speed,
              -self.config.max_yaw_rate, self.config.max_yaw_rate]

        # Dynamic window from motion model
        Vd = [self.robot_state[3] - self.config.max_accel * self.config.dt,
              self.robot_state[3] + self.config.max_accel * self.config.dt,
              self.robot_state[4] - self.config.max_delta_yaw_rate * self.config.dt,
              self.robot_state[4] + self.config.max_delta_yaw_rate * self.config.dt]

        #  [v_min, v_max, yaw_rate_min, yaw_rate_max]
        dw = [max(Vs[0], Vd[0]), min(Vs[1], Vd[1]),
              max(Vs[2], Vd[2]), min(Vs[3], Vd[3])]

        return dw


    def PredictTrajectory(self, x_init, v, y):
        """
        predict trajectory with an input
        """

        x_init = np.asarray(x_init, dtype=float)
        dt = self.config.dt

        # v and y are held constant for the whole rollout, so every step's
        # heading/position can be computed in closed form instead of looping.
        # n_steps replicates the exact iteration count of the original
        # `while time <= predict_time: time += dt` loop (same float accumulation).
        n_steps = 0
        t = 0.0
        while t <= self.config.predict_time:
            n_steps += 1
            t += dt

        k = np.arange(1, n_steps + 1)
        yaws = x_init[2] + k * y * dt          # heading is updated before position each step
        xs = x_init[0] + np.cumsum(v * np.cos(yaws) * dt)
        ys = x_init[1] + np.cumsum(v * np.sin(yaws) * dt)
        vs = np.full(n_steps, v)
        yaw_rates = np.full(n_steps, y)

        rest = np.column_stack([xs, ys, yaws, vs, yaw_rates])
        trajectory = np.vstack([x_init, rest])

        return trajectory


    def CalcControlAndTrajectory(self, dw):
        """
        calculation final input with dynamic window
        """

        x_init = self.robot_state #x[:]
        min_cost = float("inf")
        best_u = [0.0, 0.0]
        best_trajectory = np.array([self.robot_state])

        # evaluate all trajectory with sampled input in dynamic window
        for v in np.arange(dw[0], dw[1], self.config.v_resolution):
            for y in np.arange(dw[2], dw[3], self.config.yaw_rate_resolution):

                trajectory = self.PredictTrajectory(x_init, v, y)
                # calc cost
                to_goal_cost = self.config.to_goal_cost_gain * self.CalcToGoalCost(trajectory)
                speed_cost = self.config.speed_cost_gain * (self.config.max_speed - trajectory[-1, 3])/self.config.max_speed
                ob_cost = 0.0
                if (len(self.config.obstacles)>0):
                    ob_cost = self.config.obstacle_cost_gain * self.CalcObstacleCost(trajectory)
                    
                smoothness_cost = 0.0
                if self.current_u is not None:
                    smoothness_cost = self.config.smoothness_cost_gain * (abs(v - self.current_u[0]) + abs(y - self.current_u[1]))

                
                final_cost = to_goal_cost + speed_cost + ob_cost + smoothness_cost

                # search minimum trajectory
                if min_cost >= final_cost:
                    min_cost = final_cost
                    best_u = [v, y]
                    best_trajectory = trajectory

                    if abs(best_u[0]) < self.config.robot_stuck_flag_cons \
                            and abs(self.robot_state[3]) < self.config.robot_stuck_flag_cons:
                        # to ensure the robot do not get stuck in
                        # best v=0 m/s (in front of an obstacle) and
                        # best omega=0 rad/s (heading to the goal with
                        # angle difference of 0)
                        best_u[1] = -self.config.max_delta_yaw_rate
                        
        return best_u, best_trajectory


    def CalcObstacleCost(self, trajectory):
        """
        calc obstacle cost inf: collision
        """
        ox = self.config.obstacles[:, 0]
        oy = self.config.obstacles[:, 1]
        dx = trajectory[:, 0] - ox[:, None]
        dy = trajectory[:, 1] - oy[:, None]
        r = np.hypot(dx, dy)

        if self.config.robot_type == RobotType.rectangle:
            # dx/dy are already (n_obstacles, T): obstacle offset from the robot at each timestep.
            # Rotate each column t by that timestep's own yaw (not every other timestep's yaw).
            yaw = trajectory[:, 2]  # (T,)
            cos_y = np.cos(yaw)[None, :]  # (1, T) broadcasts against (n_obstacles, T)
            sin_y = np.sin(yaw)[None, :]
            local_x = dx * cos_y + dy * sin_y
            local_y = -dx * sin_y + dy * cos_y
            upper_check = local_x <= self.config.robot_length / 2
            right_check = local_y <= self.config.robot_width / 2
            bottom_check = local_x >= -self.config.robot_length / 2
            left_check = local_y >= -self.config.robot_width / 2
            if (np.logical_and(np.logical_and(upper_check, right_check),
                               np.logical_and(bottom_check, left_check))).any():
                return float("Inf")
        elif self.config.robot_type == RobotType.circle:
            if np.array(r <= self.config.robot_radius).any():
                return float("Inf")

        min_r = np.min(r)
        return 1.0 / min_r  # OK


    def CalcToGoalCost(self, trajectory):
        """
            calc to goal cost with angle difference
        """

        dx = self.goal[0] - trajectory[-1, 0]
        dy = self.goal[1] - trajectory[-1, 1]
        error_angle = math.atan2(dy, dx)
        cost_angle = error_angle - trajectory[-1, 2]
        cost = abs(math.atan2(math.sin(cost_angle), math.cos(cost_angle)))

        return cost

    def PlotArrow(self, x, y, yaw, length=0.5, width=0.1):  # pragma: no cover
        plt.arrow(x, y, length * math.cos(yaw), length * math.sin(yaw),
                  head_length=width, head_width=width)
        plt.plot(x, y)


    def PlotRobot(self, x, y, yaw):  # pragma: no cover
        if self.config.robot_type == RobotType.rectangle:
            outline = np.array([[-self.config.robot_length / 2, self.config.robot_length / 2,
                                 (self.config.robot_length / 2), -self.config.robot_length / 2,
                                 -self.config.robot_length / 2],
                                [self.config.robot_width / 2, self.config.robot_width / 2,
                                 - self.config.robot_width / 2, -self.config.robot_width / 2,
                                 self.config.robot_width / 2]])
            Rot1 = np.array([[math.cos(yaw), math.sin(yaw)],
                             [-math.sin(yaw), math.cos(yaw)]])
            outline = (outline.T.dot(Rot1)).T
            outline[0, :] += x
            outline[1, :] += y
            plt.plot(np.array(outline[0, :]).flatten(),
                     np.array(outline[1, :]).flatten(), "-k")
        elif self.config.robot_type == RobotType.circle:
            circle = plt.Circle((x, y), self.config.robot_radius, color="b")
            plt.gcf().gca().add_artist(circle)
            out_x, out_y = (np.array([x, y]) +
                            np.array([np.cos(yaw), np.sin(yaw)]) * self.config.robot_radius)
            plt.plot([x, out_x], [y, out_y], "-k")
        
    def GetRosOutputs(self, u, predicted_trajectory):
        # get a ROS cmd_vel from the "u" variable
        cmd_vel = Twist()
        heading = self.robot_state[2]
        cmd_vel.linear.x = u[0]*math.cos(heading)
        cmd_vel.linear.y = u[0]*math.sin(heading)
        cmd_vel.angular.z = u[1]
        # get a ROS path message from the predicted trajectory
        path = Path()
        for state in predicted_trajectory:
            pose_stamped = PoseStamped()
            pose_stamped.pose = PoseFromState(state)
            path.poses.append(pose_stamped)
        return cmd_vel, path
        
    def Update(self, robot_odom_in : Odometry, goal_in : Vector3, grid_in : OccupancyGrid):

        # update the class variables for obstacles, goal, and robot state
        obs_list = OccupancyGridToObstacleList(grid_in)
        self.config.obstacles = np.array(obs_list)
        
        self.goal = [goal_in.x, goal_in.y]     
        
        self.robot_state = StateFromOdom(robot_odom_in)
        
        self.LogTrajectory()
        
        # run the dwa control
        u, predicted_trajectory = self.DwaControl()
        
        self.current_u = u
        self.current_traj = predicted_trajectory
        
        # get the ROS format outputs
        cmd_vel_out, path_out = self.GetRosOutputs(u, predicted_trajectory)
          
        return cmd_vel_out, path_out

    def LogTrajectory(self):
        if (len(self.trajectory_history)==0):
            self.trajectory_history = np.array(self.robot_state)
        else:
            self.trajectory_history = np.vstack((self.trajectory_history, self.robot_state))

    def Animate(self, predicted_trajectory):
        if self.show_animation:
            plt.cla()
            # for stopping simulation with the esc key.
            plt.gcf().canvas.mpl_connect(
                'key_release_event',
                lambda event: [exit(0) if event.key == 'escape' else None])
            plt.plot(predicted_trajectory[:, 0], predicted_trajectory[:, 1], "-g")
            plt.plot(self.robot_state[0], self.robot_state[1], "xr")
            plt.plot(self.goal[0], self.goal[1], "xb")
            plt.plot(self.config.obstacles[:, 0], self.config.obstacles[:, 1], "ok")
            self.PlotRobot(self.robot_state[0], self.robot_state[1], self.robot_state[2])
            self.PlotArrow(self.robot_state[0], self.robot_state[1], self.robot_state[2])
            plt.axis("equal")
            plt.grid(True)
            plt.pause(0.0001)

    def GoalReached(self):
        # check reaching goal
        dist_to_goal = math.hypot(self.robot_state[0] - self.goal[0], self.robot_state[1] - self.goal[1])
        if dist_to_goal <= self.config.goal_radius:
            self.finished = True
        return self.finished
        
    def Finish(self):
        if self.show_animation:
            plt.plot(self.trajectory_history[:, 0], self.trajectory_history[:, 1], "-r")
            plt.pause(0.0001)
            plt.show()

if __name__ == '__main__':
    def SimulateVehicle(odom_in, u, dt):
        x = StateFromOdom(odom_in)

        x[2] += u[1] * dt
        x[0] += u[0] * math.cos(x[2]) * dt
        x[1] += u[0] * math.sin(x[2]) * dt
        x[3] = u[0]
        x[4] = u[1]
        
        odom_out = OdomFromState(x)
        return odom_out
    
    obstacle_list = [[-1, -1],
                [0, 2],
                [4.0, 2.0],
                [5.0, 4.0],
                [5.0, 5.0],
                [5.0, 6.0],
                [5.0, 9.0],
                [8.0, 9.0],
                [7.0, 9.0],
                [8.0, 10.0],
                [9.0, 11.0],
                [12.0, 13.0],
                [12.0, 12.0],
                [15.0, 15.0],
                [13.0, 13.0]]
    
    grid = OccupancyGrid()
    grid.resize(100, 100)
    grid.info.origin.x = -5.0
    grid.info.origin.y = -2.0
    grid.info.resolution = 0.25
    for obs in obstacle_list:
        [i, j] = grid.coordinate_to_index(obs[0], obs[1])
        grid.data[i,j] = 100

    goal = Vector3([10.0, 10.0, 0.0])
    
    #veh_position = [0.0, 0.0, math.pi / 8.0, 0.0, 0.0]
    vehicle_state = Odometry()
    vehicle_state.pose.orientation.w = math.cos(0.5*math.pi/8.0)
    vehicle_state.pose.orientation.z = math.sin(0.5*math.pi/8.0)

    dwa_planner = DwaPlanner(robot_type=RobotType.rectangle)
    
    while not dwa_planner.GoalReached():
        
        cmd_vel, path = dwa_planner.Update(vehicle_state, goal, grid)
        
        vehicle_state = SimulateVehicle(vehicle_state, dwa_planner.current_u, dwa_planner.config.dt)
       
        dwa_planner.Animate(dwa_planner.current_traj)
    
    dwa_planner.Finish()       
    print("Finished successfully!")