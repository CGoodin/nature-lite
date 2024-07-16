"""
Path tracking simulation with pure pursuit steering and PID speed control.

Modified from original author: 
author: Atsushi Sakai (@Atsushi_twi)
        Guillaume Jacquenot (@Gjacquenot)
From: https://github.com/AtsushiSakai/PythonRobotics/blob/master/PathTracking/pure_pursuit/pure_pursuit.py

Modified by CTG 7/15/2024 to be modular, class-based, and interface with full stack
"""
import numpy as np
import math
from autonomy_msgs import DrivingCommand

class State:

    def __init__(self, wb, x=0.0, y=0.0, yaw=0.0, v=0.0):
        self.x = x
        self.y = y
        self.yaw = yaw
        self.v = v
        self.rear_x = self.x - ((wb*0.5) * math.cos(self.yaw))
        self.rear_y = self.y - ((wb*0.5) * math.sin(self.yaw))

    def calc_distance(self, point_x, point_y):
        dx = self.rear_x - point_x
        dy = self.rear_y - point_y
        return math.hypot(dx, dy)

class States:

    def __init__(self):
        self.x = []
        self.y = []
        self.yaw = []
        self.v = []
        self.t = []

    def append(self, t, state):
        self.x.append(state.x)
        self.y.append(state.y)
        self.yaw.append(state.yaw)
        self.v.append(state.v)
        self.t.append(t)

class TargetCourse:

    def __init__(self, cx, cy):
        self.cx = cx
        self.cy = cy
        self.old_nearest_point_index = None

    def search_target_index(self, state, k, lookahead_dist):

        # To speed up nearest point search, doing it at only first time.
        if self.old_nearest_point_index is None:
            # search nearest point index
            dx = [state.rear_x - icx for icx in self.cx]
            dy = [state.rear_y - icy for icy in self.cy]
            d = np.hypot(dx, dy)
            ind = np.argmin(d)
            self.old_nearest_point_index = ind
        else:
            ind = self.old_nearest_point_index
            distance_this_index = state.calc_distance(self.cx[ind],
                                                      self.cy[ind])
            while True:
                distance_next_index = state.calc_distance(self.cx[ind + 1],
                                                          self.cy[ind + 1])
                if distance_this_index < distance_next_index:
                    break
                ind = ind + 1 if (ind + 1) < len(self.cx) else ind
                distance_this_index = distance_next_index
            self.old_nearest_point_index = ind

        Lf = k * state.v + lookahead_dist  # update look ahead distance

        # search look ahead target point index
        while Lf > state.calc_distance(self.cx[ind], self.cy[ind]):
            if (ind + 1) >= len(self.cx):
                break  # not exceed goal
            ind += 1

        return ind, Lf

def pure_pursuit_steer_control( state, trajectory, pind, wb, k, lookahead_dist):
        ind, Lf = trajectory.search_target_index(state, k, lookahead_dist)

        if pind >= ind:
            ind = pind

        if ind < len(trajectory.cx):
            tx = trajectory.cx[ind]
            ty = trajectory.cy[ind]
        else:  # toward goal
            tx = trajectory.cx[-1]
            ty = trajectory.cy[-1]
            ind = len(trajectory.cx) - 1

        alpha = math.atan2(ty - state.rear_y, tx - state.rear_x) - state.yaw

        delta = math.atan2(2.0 * wb * math.sin(alpha) / Lf, 1.0)

        return delta, ind

def proportional_control(target, current, kp):
    a = kp * (target - current)
    return a


class PurePursuitController(object):
    def __init__(self):
        self.k = 0.1  # look forward gain
        self.look_ahead_distance = 2.0  # [m] look-ahead distance
        self.throttle_kp = 1.0  # speed proportional gain
        self.wheelbase = 2.9  # [m] wheel base of vehicle
        self.show_animation = True
        self.state = None
        self.cx = []
        self.cy = []
        self.target_speed = 0.0
    def SetDesiredSpeed(self, speed):
        self.target_speed = speed
    def SetCurrentState(self, veh_x, veh_y, veh_speed, veh_heading):
        self.state = State(self.wheelbase, x=veh_x, y=veh_y, yaw=veh_heading, v=veh_speed)
    def SetDesiredPath(self, path):
        self.cx = []
        self.cy = []
        for i in range(len(path)):
            self.cx.append(path[i][0]) 
            self.cy.append(path[i][1]) 

    def GetDrivingCommand(self, veh_x, veh_y, veh_speed, veh_heading):
        self.SetCurrentState(veh_x, veh_y, veh_speed, veh_heading)
        dc = DrivingCommand()
        dc.throttle = 0.0
        dc.steering = 0.0
        if len(self.cx)<=0:
            return dc

        lastIndex = len(self.cx) - 1
        time = 0.0
        states = States()
        states.append(time, self.state)

        target_course = TargetCourse(self.cx, self.cy)
        target_ind, _ = target_course.search_target_index(self.state, self.k, self.look_ahead_distance)

        dc.throttle = proportional_control(self.target_speed, self.state.v, self.throttle_kp)
        dc.steering, target_ind = pure_pursuit_steer_control(self.state, target_course, target_ind, self.wheelbase, self.k, self.look_ahead_distance)
        assert lastIndex >= target_ind, "Cannot reach goal"
        return dc
