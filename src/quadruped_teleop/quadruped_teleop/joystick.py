#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Mon Mar  9 14:05:02 2020

@author: miguel-asd
"""

import numpy as np

from os import system, name
from evdev import InputDevice, categorize, ecodes
from select import select


def Controller():
    #code values for PS5 controller
    return {
            'X': 304,
            'triangle': 307,
            'circle': 305,
            'square': 308,
            'start': 315,
            'ps': 316,
            'r1': 311,
            'l1': 310,
            'r2': "ABS_RZ",
            'l2': "ABS_Z",
            'right_joy_x': "ABS_RX",
            'right_joy_y': "ABS_RY",
            'left_joy_x': "ABS_X",
            'left_joy_y': "ABS_Y",
            'arrow_x': "ABS_HAT0X",
            'arrow_y': "ABS_HAT0Y"
    }

    
class RobotInputs:
    def __init__(self):
        self.step_period = 0.4
        self.linear_velocity = 0.
        self.linear_angle = 0.
        self.angular_velocity = 0.
        self.compliant_mode = False
        self.pose_mode = True
        self.kill = False
        self.rest_mode = True
        self.com_pos = np.zeros(3)
        self.com_orn = np.zeros(3)
        self.calibration = 0
        self.heigth_increment = 0
    
class RawValues:
    def __init__(self):
        self.l_joy = np.array([0. , 0.])
        self.r_joy = np.array([0. , 0.])
        self.oL3 = np.array([0. , 0.])
        self.oR3 = np.array([0. , 0.])
        self.L3data = np.array([0. , 0.])
        self.R3data = np.array([0. , 0.])
        
        self.L2data = 0.
        self.L2 = 0.
        self.oL2 = 0.
        self.R2data = 0.
        self.R2 = 0.
        self.oR2 = 0.
        self.L1data = 0.
        self.L1 = 0.
        self.oL1 = 0.
        self.R1data = 0.
        self.R1 = 0.
        self.oR1 = 0.
        
        self.x=0
        self.triangle=0
        self.circle=0
        self.square=0


class Joystick:

    def __init__(self, robotInputs: RobotInputs, rawValues: RawValues):    
        self.inputs = robotInputs
        self.raw = rawValues


    def clear(self):
        # for windows
        if name == 'nt':
            _ = system('cls')
        # for mac and linux(here, os.name is 'posix')
        else:
            _ = system('clear')


    def conect(self , event):
        try:
            # python3 /usr/local/lib/python3.10/dist-packages/evdev/evtest.py 
            # or sudo jstest /dev/input/js0 for identify event
            self.gamepad = InputDevice(event)
            return True
        except:
            return False
            
            
    
    def read(self):
        last_r_joy = self.raw.r_joy
        last_l_joy = self.raw.l_joy
        last_L2 = self.raw.L2
        last_R2 = self.raw.R2

        try:
            r,w,x = select([self.gamepad.fd], [], [], 0.)
            
            if r:
                for event in self.gamepad.read():
                    if event.type == ecodes.EV_KEY:
                        if event.value == 1:                            
    #                         if event.code == 307:#triangle
    #                             if self.compliantMode == True:
    #                                 self.compliantMode = False
    #                             elif self.compliantMode == False:
    #                                 self.compliantMode = True  
                            if event.code == Controller()['square']:
                                if self.inputs.pose_mode == True:
                                    self.inputs.pose_mode = False
                                else:
                                    self.inputs.pose_mode = True
                            elif event.code == Controller()['start']:
                                if self.inputs.kill == False:
                                    self.inputs.kill = True
                                else:
                                    self.inputs.kill = False
                            elif event.code == Controller()['ps']:
                                if self.inputs.rest_mode == False:
                                    self.inputs.rest_mode = True                                
                                elif self.inputs.rest_mode == True:
                                    self.inputs.rest_mode = False
                            elif event.code == Controller()['r1']:#R1
                                self.raw.R1data += 0.0005
                                
                            elif event.code == Controller()['l1']:#L1
                                self.raw.L1data -= 0.0005
                        else:
                            pass
                    ########################################  for my own joystick
                    #      ^           #     ^            #
                    #    ABS_Y         #    ABS_RY        #
                    #  ←─────→ ABS_X #  ←─────→ ABS_RX   #
                    #     ↓           #     ↓            #  
                    #######################################
                    elif event.type == ecodes.EV_ABS:
                        absevent = categorize(event)
                        if ecodes.bytype[absevent.event.type][absevent.event.code] == Controller()['left_joy_x']: #Lx
                            self.raw.L3data[0] = absevent.event.value - 127.0
                        elif ecodes.bytype[absevent.event.type][absevent.event.code] == Controller()['left_joy_y']: #Ly
                            self.raw.L3data[1] = absevent.event.value - 127.0
                        elif ecodes.bytype[absevent.event.type][absevent.event.code] == Controller()['r2']: #R2
                            self.raw.R2data = absevent.event.value - 127.0
                        elif ecodes.bytype[absevent.event.type][absevent.event.code] == Controller()['l2']: #L2
                            self.raw.L2data = absevent.event.value - 127.0
                        elif ecodes.bytype[absevent.event.type][absevent.event.code] == Controller()['right_joy_x']: #Rx
                            self.raw.R3data[0] = absevent.event.value - 127.0
                        elif ecodes.bytype[absevent.event.type][absevent.event.code] == Controller()['right_joy_y']: #Ry
                            self.raw.R3data[1] = absevent.event.value - 127.0
            
                        elif ecodes.bytype[absevent.event.type][absevent.event.code] == Controller()['arrow_x']:
                            if absevent.event.value == 1:#right arrow 
                                self.inputs.step_period += 0.01
                            elif absevent.event.value == -1:#left arrow 
                                self.inputs.step_period -= 0.01 
                        elif ecodes.bytype[absevent.event.type][absevent.event.code] == Controller()['arrow_y']:
                            if absevent.event.value == -1:#up arrow
                                self.inputs.heigth_increment += 0.002
                            elif absevent.event.value == 1:#down arrow
                                self.inputs.heigth_increment -= 0.002
                         
            
            self.raw.l_joy[0] = self.raw.L3data[0]*0.1 + last_l_joy[0]*0.9
            self.raw.l_joy[1] = self.raw.L3data[1]*0.1 + last_l_joy[1]*0.9
            self.raw.r_joy[0] = self.raw.R3data[0]*0.1 + last_r_joy[0]*0.9
            self.raw.r_joy[1] = self.raw.R3data[1]#*0.51 + self.oR3[1]*0.49
            self.raw.R2 = self.raw.R2data*0.1 + last_R2*0.9
            self.raw.L2 = self.raw.L2data*0.1 + last_L2*0.9

            if self.inputs.pose_mode == False:           
                self.inputs.linear_velocity = np.sqrt(self.raw.l_joy[1]**2 + self.raw.l_joy[0]**2)/300.
                self.inputs.linear_angle = np.rad2deg(np.arctan2(-self.raw.l_joy[0] , -self.raw.l_joy[1]))
                self.inputs.angular_velocity = -self.raw.r_joy[0]/1500.
                if self.inputs.linear_velocity <= 0.005:
                    self.inputs.linear_velocity = 0.
                if self.inputs.angular_velocity <= 0.005 and self.inputs.angular_velocity >= -0.005:
                    self.inputs.angular_velocity = 0.
            else:
                self.inputs.com_pos[0] = self.raw.R1 + self.raw.L1  # self.r_joy[1]/10000
                self.inputs.com_pos[2] = self.raw.r_joy[1]/5000 + self.inputs.heigth_increment # self.r_joy[0]/5000 + 
                                
                self.inputs.com_orn[0] = np.deg2rad(self.raw.l_joy[0]/5)
                self.inputs.com_orn[1] = np.deg2rad(self.raw.l_joy[1]/5)
                self.inputs.com_orn[2] = - np.deg2rad(self.raw.R2/2000) + np.deg2rad(self.raw.L2/2000)
                
        except:
            pass


        return self.inputs, self.raw