from __future__ import annotations
"""
from typing import TYPE_CHECKING
if TYPE_CHECKING:
    from .state_kill import KillState
    from .state_static import StaticState
"""

from .state import State
from . import state_kill
from . import state_static

class RestState(State):
    def handleKill(self) -> None:
        print("RestState handles going to KillState. Going KillState")
        self._robotPlayer.transitionTo(state_kill.KillState())

    def handleRest(self) -> None:
        if not (self.isActive):
            self.isActive = True
            print("RestState staying in rest mode.")
        self._robotPlayer.robotResting()

    def handleStatic(self) -> None:
        print("RestState handles going to StaticState. Going StaticState")
        self._robotPlayer.standMove()
        self._robotPlayer.transitionTo(state_static.StaticState())

    def handleDynamic(self) -> None:
        print("RestState handles going to DynamicState. Going to StaticState first")
        self._robotPlayer.transitionTo(state_static.StaticState())

