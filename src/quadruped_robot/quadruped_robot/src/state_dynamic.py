from __future__ import annotations

from .state import State
from . import state_kill
from . import state_static

class DynamicState(State):
    def handleKill(self) -> None:
        print("DynamicState handles going to KillState. Going KillState")
        self._robotPlayer.transitionTo(state_kill.KillState())
    
    def handleRest(self) -> None:
        print("DynamicState handle going to RestState. Going to StaticState first")
        self._robotPlayer.transitionTo(state_static.StaticState())

    def handleStatic(self) -> None:
        print("DynamicState staying in static mode.")
        self._robotPlayer.transitionTo(state_static.StaticState())

    def handleDynamic(self) -> None:
        if not (self.is_active):
            self.is_active = True
            print("DynamicState staying in dynamic mode")
        self._robotPlayer.dynamicControl()
        

        