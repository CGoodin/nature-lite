"""
Lidar-based perception where the world is divided into a 2D map.
The slope in each cell is given by the elevation difference of the
highest lidar point and lowest lidar point divided by the cell width.
If the slope exceeds a user defined threshold, it's flagged as an obstacle
"""
import numpy as np
from autonomy_msgs import Quaternion
import matplotlib.pyplot as plt
from autonomy_msgs import Vector3

class SlopeMap(object):
    def __init__(self,dimensions=None):
        self.inflation = 0
        self.lowest = np.zeros(shape=(1,1))
        self.highest = np.zeros(shape=(1,1))
        self.slope_thresh = 0.5
        if dimensions:
            if len(dimensions) == 2:
                self.resize(dimensions[0],dimensions[1])
        plt.ion()
        self.fig, self.ax = plt.subplots()
        self.fig.canvas.manager.set_window_title('Perception')
        self.fig.canvas.draw()
    def resize(self, width, height):
        self.lowest = np.zeros(shape=(width,height))
        self.highest = np.zeros(shape=(width,height))
        self.lowest.fill(10000.0)
        self.highest.fill(-10000.0)
    def add_registered_points(self,points, occ_grid):
        for p in points:
            if (p[0]==0.0 and p[1]==0.0 and p[2]==0.0):
                continue
            idx = occ_grid.coordinate_to_index(p[0],p[1])
            i = idx[0]
            j = idx[1]
            if (occ_grid.coordinate_is_valid(idx)):
                if (p[2]>self.highest[i,j]):
                    self.highest[i,j] = p[2]
                if (p[2]<self.lowest[i,j]):
                    self.lowest[i,j] = p[2]
                cell_slope = (self.highest[i][j]-self.lowest[i,j])/occ_grid.info.resolution
                if cell_slope > self.slope_thresh:
                    occ_grid.data[i,j] = min(1.0, cell_slope)
                    if (self.inflation>0):
                        for iii in range(-self.inflation,self.inflation+1):
                            for jjj in range(-self.inflation,self.inflation+1):
                                ii = iii+i
                                jj = jjj+j
                                if ii>=0 and ii<occ_grid.info.width and jj>=0 and jj<occ_grid.info.height:
                                    occ_grid.data[ii,jj]=1.0
    def add_points(self,pos,quat,points, occ_grid):
        q = Quaternion([quat[0],quat[1],quat[2],quat[3]])
        position = Vector3([pos[0],pos[1],pos[2]])
        for p in points:
            v = Vector3([p[0],p[1],p[2]])
            vprime = q.rotate(v)
            vprime = vprime + position
            idx = occ_grid.coordinate_to_index(vprime.x,vprime.y)
            i = idx[0]
            j = idx[1]
            if occ_grid.coordinate_is_valid(idx):
                if (v.z>self.highest[i,j]):
                    self.highest[i,j] = v.z
                if (v.z<self.lowest[i,j]):
                    self.lowest[i,j] = v.z
                cell_slope = (self.highest[i][j]-self.lowest[i,j])/occ_grid.info.resolution
                if cell_slope > self.slope_thresh:
                    occ_grid.data[i,j] = min(1.0, cell_slope)
    def Display(self, path, start, goal, occ_grid):
        ny = occ_grid.info.height-1
        colorplot = np.zeros(shape=(occ_grid.info.width,occ_grid.info.height,3))
        for i in range(occ_grid.info.width):
            for j in range(occ_grid.info.height):
                colorplot[ny-j][i][0]=occ_grid.data[i][j]
        if path:
            for p in path:
                if (p[0]>=0 and p[0]<occ_grid.info.width and p[1]>=0 and p[1]<occ_grid.info.height):
                    colorplot[ny-int(p[1])][int(p[0])][2] = 1.0
        colorplot[ny-goal[1]][goal[0]][1] = 1.0
        colorplot[ny-start[1]][start[0]][0] = 1.0
        colorplot[ny-start[1]][start[0]][1] = 1.0
        self.ax.get_xaxis().set_visible(False)
        self.ax.get_yaxis().set_visible(False)
        self.ax.imshow(colorplot)
        self.fig.canvas.flush_events()
        