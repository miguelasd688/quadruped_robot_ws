from __future__ import annotations
from typing import TYPE_CHECKING
if TYPE_CHECKING:
    from .state import State

import sys
import time
import numpy as np
import rclpy

from simple_pid import PID
from .kinematic_model import RobotKinematics
from .gait_planner import TrotGait
from . import angleToPulse
from .move_feet import moveFeetZ , moveFeetTo 




class robotStateVariables:
    def __init__(self, body_to_feet_rest, orientation0, position0):
        self.to_feet = body_to_feet_rest
        self.orientation = orientation0
        self.position = position0
        self.linear_velocity = 0.
        self.linear_angle = 0.
        self.angular_velocity = 0.
        self.kinematics = RobotKinematics('><')

        print(f'foot: {self.to_feet[0,:]} | pos: {self.position} | orn: {self.orientation}')




class RobotPlayer:
    """
    The RobotPlayer defines the interface of interest to clients. It also maintains
    a reference to an instance of a State subclass, which represents the current
    state of the RobotPlayer.
    """
    """
    A reference to the current state of the RobotPlayer.
    """
    

    def __init__(self, state: State, parameters: dict) -> None:
        
        self.loop_latency = parameters['loop_latency']
        self.orientation0 = np.array(parameters['com_orientation0'])
        self.position0 = np.array(parameters['com_position0'])
        self.body_to_feet0 = np.array([parameters['body_to_feet0'][i:i + 3] for i in range(0, len(parameters['body_to_feet0']), 3)])
        self.body_to_feet_rest = np.array([parameters['body_to_feet_rest'][i:i + 3] for i in range(0, len(parameters['body_to_feet_rest']), 3)])
        self.gait_offset = np.array(parameters['gait_offset0'])
        self.step_period0 = parameters['step_period0']

        self.trot = TrotGait()

        self.body = robotStateVariables(self.body_to_feet_rest, 
                                        self.orientation0, 
                                        self.position0)

        

        self._state = state
        self.transitionTo(state)



    def transitionTo(self, state: State):
        """
        The RobotPlayer allows changing the State object at runtime.
        """

        print(f"RobotPlayer: Transition to {type(state).__name__}")
        self._state = state
        self._state._robotPlayer = self

    """
    The RobotPlayer delegates part of its behavior to the current State object.
    """

    def updateKill(self):
        self._state.handleKill()

    def updateRest(self):
        self._state.handleRest()
    
    def updateStaticControl(self):
        self._state.handleStatic()

    def updateDynamicControl(self):
        self._state.handleDynamic()


    def setRobotStateVariables(self, robot_desired_states):
        self.body.orientation = robot_desired_states['com_orientation']
        self.body.position = robot_desired_states['com_position']
        self.body.linear_velocity = robot_desired_states['linear_velocity']
        self.body.linear_angle = robot_desired_states['linear_angle']
        self.body.angular_velocity = robot_desired_states['angular_velocity']


    def killProgram(self):
        print("Reseting microcontroller and closing...")
        time.sleep(0.4)

        angles , transformedBodytoFeet = self.body.kinematics.solve(self.orientation0, 
                                                                    self.position0, 
                                                                    self.body.to_feet)
        pulsesCommand = angleToPulse.convert(angles)

        sys.exit()


    def robotResting(self):
        self.body_to_feet = self.body_to_feet_rest.copy()

        angles , transformedBodytoFeet = self.body.kinematics.solve(self.orientation0, 
                                                                    self.position0, 
                                                                    self.body.to_feet)
        pulsesCommand = angleToPulse.convert(angles)


    def standMove(self):
        ## STAND UP ROUTINE
        h = 0.06 # step height in m.

        print('Standing up')
        moveFeetZ(self.loop_latency , self.body , self.body_to_feet0[0,2] , 1.0)

        print('Init pose 0')
        moveFeetTo(self.loop_latency , self.body , self.body_to_feet0 , h , 0.6)


    def layDownMove(self):
        ##  RESTING ROUTINE
        h = 0.06 # step height in m.

        print('Init REST')
        moveFeetTo(self.loop_latency , self.body , self.body_to_feet_rest , h , 0.6)

        print('Going rest')
        moveFeetZ(self.loop_latency , self.body , self.body_to_feet_rest[0,2] , 1.0)


    def staticControl(self):
        self.body_to_feet = self.body_to_feet0.copy() # Static position.
        angles , transformedBodytoFeet = self.body.kinematics.solve(self.orientation0 + self.body.orientation, 
                                                                    self.position0 + self.body.position, 
                                                                    self.body.to_feet)
        pulsesCommand = angleToPulse.convert(angles)


    def dynamicControl(self):
        self.body_to_feet  = self.trot.loop(self.body.linear_velocity , self.body.linear_angle , 
                                            self.body.angular_velocity , self.step_period0 , 
                                            self.gait_offset , self.body_to_feet0)
        angles , transformedBodytoFeet = self.body.kinematics.solve(self.orientation0 + self.body.orientation, 
                                                                    self.position0 + self.body.position, 
                                                                    self.body.to_feet)
        pulsesCommand = angleToPulse.convert(angles)