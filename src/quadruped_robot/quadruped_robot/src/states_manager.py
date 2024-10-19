from __future__ import annotations
from typing import TYPE_CHECKING
if TYPE_CHECKING:
    from .state import State

import numpy as np

class StatesManager:
    """
    The StatesManager defines the interface of interest to clients. It also maintains
    a reference to an instance of a State subclass, which represents the current
    state of the StatesManager.
    """
    """
    A reference to the current state of the StatesManager.
    """
    
    def __init__(self, state: State) -> None:
        self._state = state
        self.transitionTo(state)

    def transitionTo(self, state: State):
        """
        The StatesManager allows changing the State object at runtime.
        """

        print(f"StatesManager: Transition to {type(state).__name__}")
        self._state = state
        self._state._robotPlayer = self

    """
    The StatesManager delegates part of its behavior to the current State object.
    """

    def updateKill(self):
        self._state.handleKill()

    def updateRest(self):
        self._state.handleRest()
    
    def updateStaticControl(self):
        self._state.handleStatic()

    def updateDynamicControl(self):
        self._state.handleDynamic()


