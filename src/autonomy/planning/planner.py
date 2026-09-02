from abc import ABC, abstractmethod
from typing import Tuple
from autonomy.autonomy_msgs import OccupancyGrid, Odometry, Vector3, Twist, Path

class Planner(ABC):

    @abstractmethod
    def Update(self, robot_odom_in : Odometry, goal_in : Vector3, grid_in : OccupancyGrid) -> Tuple[Twist, Path]:
        pass

    @abstractmethod
    def GoalReached(self) -> bool:
        pass