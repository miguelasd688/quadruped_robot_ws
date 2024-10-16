#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Mon Mar  9 14:05:02 2020

@author: miguel-asd
"""

from evdev import InputDevice, categorize, ecodes
from select import select
import numpy as np
import sys

class Joystick:
    def __init__(self):

        self.L3 = np.array([0. , 0.])
        self.R3 = np.array([0. , 0.])
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
        
        self.T = 0.4
        self.V = 0.
        self.angle = 0.
        self.Wrot = 0.
        self.compliantMode = False
        self.poseMode = True
        self.Kill = False
        self.Rest = True
        self.bodyAngle = 0.
        self.CoM_pos = np.zeros(3)
        self.CoM_orn = np.zeros(3)
        self.calibration = 0
        self.increment = 0
        
    def conect(self , event):
        try:
            # python3 /usr/local/lib/python3.10/dist-packages/evdev/evtest.py 
            # or sudo jstest /dev/input/js0 for identify event
            self.gamepad = InputDevice(event)
            print('Bluetooth joystick conected at: ', event)
        except:
            print('Bluetooth joystick not conect. Try: python3 /usr/local/lib/python3.10/dist-packages/evdev/evtest.py')
            sys.exit()
            pass
    
    def read(self):
        try:
            r,w,x = select([self.gamepad.fd], [], [], 0.)
            
            if r:
                for event in self.gamepad.read():
    #                print(event)
                    if event.type == ecodes.EV_KEY:
                        if event.value == 1:                            
    #                         if event.code == 307:#triangle
    #                             if self.compliantMode == True:
    #                                 self.compliantMode = False
    #                             elif self.compliantMode == False:
    #                                 self.compliantMode = True  
                            if event.code == 305:#X
                                if self.poseMode == True:
                                    self.poseMode = False
                                elif self.poseMode == False:
                                    self.poseMode = True
                            if event.code == 313:#start
                                if self.Kill == False:
                                    self.Kill = True
                                elif self.Kill == True:
                                    self.Kill = False
                            if event.code == 316: #PS      ( 307:#Y )
                                if self.Rest == False:
                                    self.Rest = True                                
                                elif self.Rest == True:
                                    self.Rest = False
                            if event.code == 309:#R1
                                self.R1data += 0.0005
                            #if event.code == 297:#R2
                            #    self.R2data += 0.0005
                                
                            if event.code == 308:#L1
                                self.L1data -= 0.0005
                            #if event.code == 296:#L2
                            #    self.L2data -= 0.0005
                        else:
                            print("button released")
                            self.L2data = 0.
                            self.R2data = 0.
                            self.L1data = 0.
                            self.R1data = 0.
                    ########################################  for my own joystick
                    #      ^           #     ^            #
                    #    ABS_Y         #    ABS_RY        #
                    #  ←─────→ ABS_X #  ←─────→ ABS_RX   #
                    #     ↓           #     ↓            #  
                    #######################################
                    elif event.type == ecodes.EV_ABS:
                        absevent = categorize(event)
                        if ecodes.bytype[absevent.event.type][absevent.event.code] == "ABS_X": #Lx
                            self.L3data[0] = absevent.event.value - 127.0
                        elif ecodes.bytype[absevent.event.type][absevent.event.code] == "ABS_Y": #Ly
                            self.L3data[1] = absevent.event.value - 127.0
                        elif ecodes.bytype[absevent.event.type][absevent.event.code] == "ABS_RY": #R2
                            self.R2data = absevent.event.value + 32767
                        elif ecodes.bytype[absevent.event.type][absevent.event.code] == "ABS_RX": #L2
                            self.L2data = absevent.event.value + 32767
                        elif ecodes.bytype[absevent.event.type][absevent.event.code] == "ABS_Z": #Rx
                            self.R3data[0] = absevent.event.value - 127.0
                        elif ecodes.bytype[absevent.event.type][absevent.event.code] == "ABS_RZ": #Ry
                            self.R3data[1] = absevent.event.value - 127.0
                        
                        elif ecodes.bytype[absevent.event.type][absevent.event.code] == "ABS_HAT0X":
                            if absevent.event.value == 1:#right arrow 
                                self.T += 0.01
                            elif absevent.event.value == -1:#left arrow 
                                self.T -= 0.01 
                        elif ecodes.bytype[absevent.event.type][absevent.event.code] == "ABS_HAT0Y":
                            if absevent.event.value == -1:#up arrow
                                self.increment += 0.002
                            elif absevent.event.value == 1:#down arrow
                                self.increment -= 0.002
            
            self.L3[0] = self.L3data[0]*0.1 + self.oL3[0]*0.9
            self.L3[1] = self.L3data[1]*0.1 + self.oL3[1]*0.9
            self.R3[0] = self.R3data[0]*0.1 + self.oR3[0]*0.9
            self.R3[1] = self.R3data[1]#*0.51 + self.oR3[1]*0.49
            self.R2 = self.R2data*0.1 + self.oR2*0.9
            self.L2 = self.L2data*0.1 + self.oL2*0.9

#             if sumR2 == True
#                 self.R2data += 0.0005
#                 
#             if sumL2 == True
#                 self.L2data += 0.0005
#                 
#             self.L2 = self.L2data*0.05 + self.oL2*0.95
#             self.R2 = self.R2data*0.05 + self.oR2*0.95
#             if sumR2 == True
#                 self.R2data += 0.0005
#             elif sumL2 == True
#                 self.L2data += 0.0005
#                 
#             self.L1 = self.L1data*0.05 + self.oL1*0.95
#             self.R1 = self.R1data*0.05 + self.oR1*0.95
            
            
            self.oL3[0] = self.L3[0]
            self.oL3[1] = self.L3[1]
            self.oR3[0] = self.R3[0]
            self.oR3[1] = self.R3[1]
            
            self.oL2 = self.L2
            self.oR2 = self.R2
            self.oL1 = self.L1
            self.oR1 = self.R1
            
            if self.poseMode == False:           
                self.V = np.sqrt(self.L3[1]**2 + self.L3[0]**2)/300.
                self.angle = np.rad2deg(np.arctan2(-self.L3[0] , -self.L3[1]))
                self.Wrot = -self.R3[0]/1500.
        #        Lrot = 0.
                if self.V <= 0.005:
                    self.V = 0.
                if self.Wrot <= 0.005 and self.Wrot >= -0.005:
                    self.Wrot = 0.
            else:
                self.CoM_pos[0] = self.R1 + self.L1  # self.R3[1]/10000
                #self.CoM_pos[1] = -self.R3[0]/10000 
                self.CoM_pos[2] = self.R3[1]/5000 + self.increment # self.R3[0]/5000 + 
                                
                self.CoM_orn[0] = np.deg2rad(self.L3[0]/5)
                self.CoM_orn[1] = np.deg2rad(self.L3[1]/5)
                self.CoM_orn[2] = - np.deg2rad(self.R2/2000) + np.deg2rad(self.L2/2000)
                
        except:
            pass
        return self.bodyAngle , self.CoM_pos , self.CoM_orn , self.V ,-self.angle , -self.Wrot , self.T , self.compliantMode , self.Kill , self.Rest , self.poseMode 
