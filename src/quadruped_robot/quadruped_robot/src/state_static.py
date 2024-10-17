from __future__ import annotations

from .state import State


class StaticState(State):
    def handleKill(self) -> None:
        print("StaticState handles going to KillState. Going KillState")
        from .state_kill import KillState
        self._robotPlayer.transitionTo(KillState())
    
    def handleRest(self) -> None:
        print("StaticState handle going to RestState. Going to RestState")
        self._robotPlayer.layDownMove()
        from .state_rest import RestState
        self._robotPlayer.transitionTo(RestState())

    def handleStatic(self) -> None:
        if not (self.isActive):
            self.isActive = True
            print("StaticState staying in static mode.")
        self._robotPlayer.staticControl()

    def handleDynamic(self) -> None:
        print("StaticState handles going to DynamicState. Going to DynamicState")
        from .state_dynamic import DynamicState
        self._robotPlayer.transitionTo(DynamicState())

