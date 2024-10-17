from __future__ import annotations

from .state import State


class KillState(State):
    def handleKill(self) -> None:
        if not (self.isActive):
            self.isActive = True
            print("KillState killing program.")
        self._robotPlayer.killProgram()

    def handleRest(self) -> None:
        pass

    def handleStatic(self) -> None:
        pass
        
    def handleDynamic(self) -> None:
        pass
