from __future__ import annotations

from .state import State
from . import state_kill
from . import state_rest
from . import state_static
from . import state_dynamic

class CalibrationState(State):
    def handleKill(self) -> None:
        print("CalibrationState handles going to KillState. Going KillState")
        self._robotPlayer.transitionTo(state_kill.KillState())
    
    def handleRest(self) -> None:
        print("CalibrationState handles going to RestState. Terminate calibration first")
        pass

    def handleCalibration(self) -> None:
        if not (self.is_active):
            self.is_active = True
            print("CalibrationState staying in calibration mode.")
        self._robotPlayer.calibrationControl()

    def handleStatic(self) -> None:
        print("CalibrationState handles going to StaticState.")
        self._robotPlayer.transitionTo(state_static.StaticState())

    def handleDynamic(self) -> None:
        print("CalibrationState handles going to DynamicState. Terminate calibration first")
        pass

