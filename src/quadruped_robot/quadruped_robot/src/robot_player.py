from __future__ import annotations
from typing import TYPE_CHECKING
if TYPE_CHECKING:
    from .state import State
import time
import numpy as np
from sys import exit

from . import move_controller
from . import kinematic_model
from . import gait_planner
from .states_manager import StatesManager

class RobotStateVariables:
    def __init__(self, body_to_feet_rest, orientation0, position0):
        self.to_feet = body_to_feet_rest.copy()
        self.orientation = orientation0.copy()
        self.position = position0.copy()
        self.linear_velocity = 0.
        self.linear_angle = 0.
        self.angular_velocity = 0.
        self.kinematics = kinematic_model.RobotKinematics('><')
        angles , self.to_feet = self.kinematics.solve(orientation0, 
                                       position0, 
                                       body_to_feet_rest)
        self.joint_angles = angles.copy()
        



class RobotPlayer(StatesManager):
    # Init all robot variables and instanciate quadruped model
    # Defines the differentes states the robot can be
    def __init__(self, state: State, parameters: dict):
        super().__init__(state)
        self.loop_latency = parameters['loop_latency']
        self.orientation0 = np.array(parameters['com_orientation0'])
        self.position0 = np.array(parameters['com_position0'])
        self.body_to_feet0 = np.array([parameters['body_to_feet0'][0:3],
                                       parameters['body_to_feet0'][3:6],
                                       parameters['body_to_feet0'][6:9],
                                       parameters['body_to_feet0'][9:12]])
        self.body_to_feet_rest = np.array([parameters['body_to_feet_rest'][0:3],
                                           parameters['body_to_feet_rest'][3:6],
                                           parameters['body_to_feet_rest'][6:9],
                                           parameters['body_to_feet_rest'][9:12]])
        self.gait_offset = np.array(parameters['gait_offset0'])
        self.step_period0 = parameters['step_period0']

        self.trot = gait_planner.TrotGait()

        self.body = RobotStateVariables(self.body_to_feet_rest, 
                                        self.orientation0, 
                                        self.position0)
        
        self.controller = move_controller.MoveController(self.body, self.loop_latency)

    def killProgram(self):
        print("Reseting microcontroller and closing...")
        time.sleep(0.4)
        self.body.joint_angles, self.body.to_feet = self.body.kinematics.solve(self.orientation0, 
                                                                          self.position0, 
                                                                          self.body.to_feet)
        exit()


    def robotResting(self):
        self.body.to_feet = self.body_to_feet_rest.copy()
        self.body.joint_angles, self.body.to_feet = self.body.kinematics.solve(self.orientation0, 
                                                                          self.position0, 
                                                                          self.body.to_feet)


    def standUpMove(self):
        move_done = False
        self.body.to_feet, move_done = self.controller.updateStandUp(self.body_to_feet0)
        self.body.joint_angles, self.body.to_feet = self.body.kinematics.solve(self.orientation0, 
                                                                               self.position0, 
                                                                               self.body.to_feet)
        return move_done    

    def layDownMove(self):
        move_done = False
        self.body.to_feet, move_done = self.controller.updateLayDown(self.body_to_feet_rest.copy())
        self.body.joint_angles, self.body.to_feet = self.body.kinematics.solve(self.orientation0, 
                                                                          self.position0, 
                                                                          self.body.to_feet)
        return move_done

    def staticControl(self):
        self.body.to_feet = self.body_to_feet0.copy() # Static position.
        self.body.joint_angles, self.body.to_feet = self.body.kinematics.solve(self.orientation0 + self.body.orientation, 
                                                                          self.position0 + self.body.position, 
                                                                          self.body.to_feet)


    def dynamicControl(self):
        self.body.to_feet  = self.trot.loop(self.body.linear_velocity , 
                                            self.body.linear_angle , 
                                            self.body.angular_velocity , 
                                            self.step_period0 , 
                                            self.gait_offset , 
                                            self.body_to_feet0)
        self.body.joint_angles, self.body.to_feet = self.body.kinematics.solve(self.orientation0 + self.body.orientation, 
                                                                          self.position0 + self.body.position, 
                                                                          self.body.to_feet)
