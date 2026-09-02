from abc import ABC, abstractmethod
from typing import Tuple
from autonomy.autonomy_msgs import OccupancyGrid, Odometry, Vector3, Twist, Path

class Simulation(ABC):

    @abstractmethod
    def Update(self, throttle : float, steering : float, braking : float):
        pass

    @abstractmethod
    def GetPositionSpeedHeading(self) -> Tuple[list[float], float, float]:
        pass

    @abstractmethod
    def GetVehicleStateAsOdometry(self) -> Odometry:
        pass

    @abstractmethod
    def IsValid(self) -> bool:
        pass