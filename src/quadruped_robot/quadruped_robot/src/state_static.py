from __future__ import annotations

from .state import State
from . import state_kill
from . import state_rest
from . import state_dynamic
from . import state_calibration

class StaticState(State):
    def handleKill(self) -> None:
        print("StaticState handles going to KillState. Going KillState")
        self._robotPlayer.transitionTo(state_kill.KillState())
    
    def handleRest(self) -> None:
        if (self._robotPlayer.layDownMove()):
            print("StaticState handle going to RestState. Going to RestState")
            self._robotPlayer.transitionTo(state_rest.RestState())
    
    def handleCalibration(self) -> None:
        print("StaticState handle going to CalibrationState. Going to CalibrationState")
        self._robotPlayer.transitionTo(state_calibration.CalibrationState())

    def handleStatic(self) -> None:
        if not (self.is_active):
            self.is_active = True
            print("StaticState staying in static mode.")
        self._robotPlayer.staticControl()

    def handleDynamic(self) -> None:
        print("StaticState handles going to DynamicState. Going to DynamicState")
        self._robotPlayer.transitionTo(state_dynamic.DynamicState())

