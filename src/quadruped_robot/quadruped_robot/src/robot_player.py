from __future__ import annotations
from typing import TYPE_CHECKING
if TYPE_CHECKING:
    from .state import State
import time
import numpy as np
from sys import exit

from .states_manager import StatesManager
from .kinematic_model import RobotKinematics
from .gait_planner import TrotGait
from . import angleToPulse
from .move_feet import moveFeetZ , moveFeetTo 


class RobotStateVariables:
    def __init__(self, body_to_feet_rest, orientation0, position0):
        self.to_feet = body_to_feet_rest
        self.orientation = orientation0
        self.position = position0
        self.linear_velocity = 0.
        self.linear_angle = 0.
        self.angular_velocity = 0.
        self.kinematics = RobotKinematics('><')




class RobotPlayer(StatesManager):
    def __init__(self, state: State, parameters: dict):
        super().__init__(state)
        self.loop_latency = parameters['loop_latency']
        self.orientation0 = np.array(parameters['com_orientation0'])
        self.position0 = np.array(parameters['com_position0'])
        self.body_to_feet0 = np.array([parameters['body_to_feet0'][i:i + 3] for i in range(0, len(parameters['body_to_feet0']), 3)])
        self.body_to_feet_rest = np.array([parameters['body_to_feet_rest'][i:i + 3] for i in range(0, len(parameters['body_to_feet_rest']), 3)])
        self.gait_offset = np.array(parameters['gait_offset0'])
        self.step_period0 = parameters['step_period0']

        self.trot = TrotGait()

        self.body = RobotStateVariables(self.body_to_feet_rest, 
                                        self.orientation0, 
                                        self.position0)


    def setRobotStateVariables(self, robot_desired_states: dict):
        self.body.orientation = robot_desired_states['com_orientation']
        self.body.position = robot_desired_states['com_position']
        self.body.linear_velocity = robot_desired_states['linear_velocity']
        self.body.linear_angle = robot_desired_states['linear_angle']
        self.body.angular_velocity = robot_desired_states['angular_velocity']


    def killProgram(self):
        print("Reseting microcontroller and closing...")
        time.sleep(0.4)
        print(f'foot: {self.body.to_feet[0,:]} | pos: {self.body.position} | orn: {self.body.orientation}')
        angles = self.body.kinematics.solve(self.orientation0, 
                                            self.position0, 
                                            self.body.to_feet)
        pulsesCommand = angleToPulse.convert(angles)

        exit()


    def robotResting(self):
        self.body.to_feet = self.body_to_feet_rest.copy()

        angles = self.body.kinematics.solve(self.orientation0, 
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
        print(f'foot: {self.body.to_feet[0,:]} | pos: {self.body.position} | orn: {self.body.orientation}')
        self.body_to_feet = self.body_to_feet0.copy() # Static position.
        angles = self.body.kinematics.solve(self.orientation0 + self.body.orientation, 
                                            self.position0 + self.body.position, 
                                            self.body.to_feet)
        pulsesCommand = angleToPulse.convert(angles)


    def dynamicControl(self):
        self.body_to_feet  = self.trot.loop(self.body.linear_velocity , 
                                            self.body.linear_angle , 
                                            self.body.angular_velocity , 
                                            self.step_period0 , 
                                            self.gait_offset , 
                                            self.body_to_feet0)
        
        angles = self.body.kinematics.solve(self.orientation0 + self.body.orientation, 
                                            self.position0 + self.body.position, 
                                            self.body.to_feet)
        pulsesCommand = angleToPulse.convert(angles)