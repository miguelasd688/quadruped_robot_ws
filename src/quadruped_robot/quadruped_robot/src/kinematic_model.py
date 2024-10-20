#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Thu Feb 27 15:21:52 2020

@author: miguel-asd
"""
import numpy as np
from . import IK_solver
from . import geometrics


""""""""""""""""""""""""""""""""""""""""""""""""
"                   used frame                 "
"  z                                           "
"    |                                         "
"    |                                         "
"    |    /  y                                 "
"    |   /                                     "
"    |  /                                      "
"    | /                                       "
"    |/_____________  x  (front of the robot)  "
""""""""""""""""""""""""""""""""""""""""""""""""
class RobotKinematics:
    #   kinematics Model: Input body orientation, deviation and foot position    
    #   and get the angles, neccesary to reach that position, for every joint 
    #   It can move a joint in the geometric center of the body, zero by default.  
       
    def __init__(self , kneeConfig = "><"):
        self.kneeConfig = kneeConfig
        """in meter """
        self.L = 0.248 #length of robot joints
        self.W = 0.090 #width of robot joints
        self.coxa = 0.035 #coxa length
        self.femur = 0.09 #femur length
        self.tibia = 0.09 #tibia length
        
        """initial foot position"""
        self.Ydist = 11.
        self.Xdist = self.L
        self.height = 15.
        
        self.orn_f = np.zeros([3])
        self.orn_b = np.zeros([3])
        #body frame to coxa frame vector
        self.bodytoFR0 = np.array([ self.L/2, -self.W/2 , 0])
        self.bodytoFL0 = np.array([ self.L/2,  self.W/2 , 0])
        self.bodytoBR0 = np.array([-self.L/2, -self.W/2 , 0])
        self.bodytoBL0 = np.array([-self.L/2,  self.W/2 , 0])
        #body frame to foot frame vector
        self.bodytoFR4 = np.array([ self.Xdist/2 , -self.Ydist/2 , -self.height])
        self.bodytoFL4 = np.array([ self.Xdist/2 ,  self.Ydist/2 , -self.height])
        self.bodytoBR4 = np.array([-self.Xdist/2 , -self.Ydist/2 , -self.height])
        self.bodytoBL4 = np.array([-self.Xdist/2 ,  self.Ydist/2 , -self.height])

    def solve(self, orn , pos , bodytoFeet , body_ang = 0 ):
        bodytoFR4 = np.asarray([bodytoFeet[0,0],bodytoFeet[0,1],bodytoFeet[0,2]])
        bodytoFL4 = np.asarray([bodytoFeet[1,0],bodytoFeet[1,1],bodytoFeet[1,2]])
        bodytoBR4 = np.asarray([bodytoFeet[2,0],bodytoFeet[2,1],bodytoFeet[2,2]])
        bodytoBL4 = np.asarray([bodytoFeet[3,0],bodytoFeet[3,1],bodytoFeet[3,2]])

        
        self.orn_f[0] = orn[0]
        self.orn_f[1] = orn[1]
        self.orn_b[0] = orn[0]
        self.orn_b[1] = orn[1]

        bend = body_ang/2.
        self.orn_f[2] = orn[2] + bend
        self.orn_b[2] = orn[2] - bend

        """defines 4 vertices which rotates with the body"""
        _bodytoFR0 = geometrics.transform(self.bodytoFR0 , self.orn_f, pos)
        _bodytoFL0 = geometrics.transform(self.bodytoFL0 , self.orn_f, pos)
        _bodytoBR0 = geometrics.transform(self.bodytoBR0 , self.orn_b, pos)
        _bodytoBL0 = geometrics.transform(self.bodytoBL0 , self.orn_b, pos)
        """defines coxa_frame to foot_frame leg vector neccesary for IK"""
        FRcoord = bodytoFR4 - _bodytoFR0
        FLcoord = bodytoFL4 - _bodytoFL0
        BRcoord = bodytoBR4 - _bodytoBR0
        BLcoord = bodytoBL4 - _bodytoBL0
        """undo transformation of leg vector to keep feet still"""

        undoOrn_f = -self.orn_f
        undoOrn_b = -self.orn_b
        undoPos = -pos

        _FRcoord = geometrics.transform(FRcoord , undoOrn_f, undoPos)
        _FLcoord = geometrics.transform(FLcoord , undoOrn_f, undoPos)
        _BRcoord = geometrics.transform(BRcoord , undoOrn_b, undoPos)
        _BLcoord = geometrics.transform(BLcoord , undoOrn_b, undoPos)
        
        angles = IK_solver.legs_IK(_FRcoord , _FLcoord , _BRcoord , _BLcoord , 
                                   self.coxa , self.femur , self.tibia , self.kneeConfig)

        
        _bodytofeetFR = _bodytoFR0 + _FRcoord
        _bodytofeetFL = _bodytoFL0 + _FLcoord
        _bodytofeetBR = _bodytoBR0 + _BRcoord
        _bodytofeetBL = _bodytoBL0 + _BLcoord
        _bodytofeet = np.array([[_bodytofeetFR[0] , _bodytofeetFR[1] , _bodytofeetFR[2]],
                                 [_bodytofeetFL[0] , _bodytofeetFL[1] , _bodytofeetFL[2]],
                                 [_bodytofeetBR[0] , _bodytofeetBR[1] , _bodytofeetBR[2]],
                                 [_bodytofeetBL[0] , _bodytofeetBL[1] , _bodytofeetBL[2]]])
        
        return angles, _bodytofeet
