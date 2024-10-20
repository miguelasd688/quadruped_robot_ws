from __future__ import annotations
from typing import TYPE_CHECKING
if TYPE_CHECKING:
    from .states_manager import RobotPlayer

from abc import ABC, abstractmethod



class State(ABC):
    """
    The base State class declares methods that all Concrete State should
    implement and also provides a backreference to the Context object,
    associated with the State. This backreference can be used by States to
    transition the Context to another State.
    """

    def __init__(self):
        self.is_active = False

    @property
    def context(self) -> RobotPlayer:
        return self._robotPlayer

    @context.setter
    def context(self, robotPlayer: RobotPlayer) -> None:
        self._robotPlayer = robotPlayer

    @abstractmethod
    def handleRest(self) -> None:
        pass

    @abstractmethod
    def handleKill(self) -> None:
        pass
    
    @abstractmethod
    def handleStatic(self) -> None:
        pass

    @abstractmethod
    def handleDynamic(self) -> None:
        pass









