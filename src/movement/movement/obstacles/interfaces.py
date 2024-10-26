from abc import ABC, abstractmethod

from typing import Tuple

class Obstacle(ABC):
    
    @abstractmethod
    def is_colission(self, point: Tuple[float, float], padding: float = 90) -> bool:
        ''' Method to check collisions '''
        pass

class StaticObstacle(Obstacle):

    @abstractmethod
    def closest_outside_point(self, point: Tuple[float, float], offset: float = 90, padding: float = 90) -> Tuple[float, float]:
        ''' Return the closest point outside the obstacle '''
        pass
    
class DynamicObstacle(Obstacle):

    @abstractmethod
    def get_dynamic_range(self, delta: float) -> Tuple[Tuple[float, float], float]:
        ''' Returns the center position and radius of dinamic range '''
        pass

    @abstractmethod
    def update_state(self, point: Tuple[float, float]) -> None:
        pass
    
class DynamicObstacle(Obstacle):

    @abstractmethod
    def get_dynamic_range(self, delta: float) -> Tuple[Tuple[float, float], float]:
        ''' Returns the center position and radius of dinamic range '''
        pass

    @abstractmethod
    def update_state(self, point: Tuple[float, float]) -> None:
        pass