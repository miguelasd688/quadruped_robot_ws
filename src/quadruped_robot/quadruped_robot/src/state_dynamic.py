from __future__ import annotations

from .state import State


class DynamicState(State):
    def handleKill(self) -> None:
        print("DynamicState handles going to KillState. Going KillState")
        from .state_kill import KillState
        self._robotPlayer.transitionTo(KillState())
    
    def handleRest(self) -> None:
        print("DynamicState handle going to RestState. Going to StaticState first")
        from .state_static import StaticState
        self._robotPlayer.transitionTo(StaticState())

    def handleStatic(self) -> None:
        print("DynamicState staying in static mode.")
        from .state_static import StaticState
        self._robotPlayer.transitionTo(StaticState())

    def handleDynamic(self) -> None:
        if not (self.isActive):
            self.isActive = True
            print("DynamicState staying in dynamic mode")
        self._robotPlayer.dynamicControl()
        

        