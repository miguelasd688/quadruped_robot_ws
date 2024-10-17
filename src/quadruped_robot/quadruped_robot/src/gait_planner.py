#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Created on Wed Mar 11 16:38:15 2020

@author: miguel-asd
"""
import time
import numpy as np
import sys

#function neccesary to build a parametrized bezier curve 
def f(n,k): #calculates binomial factor (n k)
    return np.math.factorial(n)/(np.math.factorial(k)*np.math.factorial(n-k))

def b(t,k,point):
    n = 9 #10 points bezier curve
    return point*f(n,k)*np.power(t,k)*np.power(1-t,n-k)


#gait planner in order to move all feet
class TrotGait:
    def __init__(self):
        self.bodytoFeet = np.zeros([4,3])  
        self.phi = 99999.
        self.phiStance = 0.
        self.lastTime = 0.
        self.alpha = 0.
        self.s = False
    
    """This trajectory planning is mostly based on: 
    https://www.researchgate.net/publication/332374021_Leg_Trajectory_Planning_for_Quadruped_Robots_with_High-Speed_Trot_Gait"""
    def calculateStance(self , phi_st , V , Tst , angle):#phi_st between [0,1), angle in degrees
        c = np.cos(np.deg2rad(angle))#cylindrical coordinates
        s = np.sin(np.deg2rad(angle))
        
        A = 0.002
        L = np.abs(V)*Tst
        halfL = L/2.
        p_stance=L*(1-2*phi_st)/2.
        
        stanceX = -c*L*(phi_st-0.5)
        stanceY = s*L*(phi_st-0.5)
        if (L != 0):
            stanceZ = -A*np.cos(np.pi*p_stance/L)
        else:
            stanceZ = 0.
            
        return stanceX, stanceY , stanceZ
    
    
    def calculateBezier_swing(self , phi_sw , V , angle):#phi between [0,1), angle in degrees
    #curve generator https://www.desmos.com/calculator/xlpbe9bgll
        c = np.cos(np.deg2rad(angle))#cylindrical coordinates
        s = np.sin(np.deg2rad(angle))
#        if (phi >= 0.75 or phi < 0.25):
#            self.s = True
##            print('foot DOWN' , self.s , phi)
#            
#        elif (phi <= 0.75 and phi > 0.25):
#            self.s = False
##            print('foot UP', self.s , phi)
            
        
        X = np.abs(V)*c*np.array([-0.05 ,
                                  -0.06 ,
                                  -0.07 , 
                                  -0.07 ,
                                  0. ,
                                  0. , 
                                  0.07 ,
                                  0.07 ,
                                  0.06 ,
                                  0.05 ])
    
        Y = np.abs(V)*s*np.array([ 0.05 ,
                                   0.06 ,
                                   0.07 , 
                                   0.07 ,
                                   0. ,
                                   -0. , 
                                   -0.07 ,
                                   -0.07 ,
                                   -0.06 ,
                                   -0.05 ])
    
        Z = np.abs(V)*np.array([0. ,
                                0. ,
                                0.05 , 
                                0.05 ,
                                0.05 ,
                                0.06 , 
                                0.06 ,
                                0.06 ,
                                0. ,
                                0. ])
        swingX = 0.
        swingY = 0.
        swingZ = 0.
        for i in range(10): #sum all terms of the curve
            swingX = swingX + b(phi_sw,i,X[i]) 
            swingY = swingY + b(phi_sw,i,Y[i])
            swingZ = swingZ + b(phi_sw,i,Z[i])
            
        return swingX, swingY , swingZ


    def BSpline(self, t, degree, points, knots):

        n = len(points)    #points count
        d = len(points[0]) #point dimensionality
        weights = np.zeros(n)
        results = np.zeros(d)
        s = 1
        v = np.zeros((n, d+1))
        if(degree < 1):
            print('degree must be at least 1 (linear)')
            sys.exit()
        if(degree > (n-1)):
            print('degree must be less than or equal to point count - 1 ', n)
            sys.exit()

        if(len(knots) != n+degree+1 ):
            print('bad knot vector length')
            sys.exit()
        for i in range(n):
            weights[i] = 1.

        domain = [degree,
                len(knots) - 1 - degree]

        # remap t to the domain where the spline is defined
        low = knots[domain[0]]
        high = knots[domain[1]]
        t = t * (high - low) + low

        if(t < low or t > high):
            print('out of bounds')
            sys.exit()
        # find s (the spline segment) for the [t] value provided
        for si in range(domain[0],domain[1]):
            if(t >= knots[si] and t <= knots[si+1]):
                s = si
                break

        # convert points to homogeneous coordinates
        
        for i in range(n):
            for j in range(d):
                v[i , j] = points[i , j] * weights[i]
            v[i , d] = weights[i]
  

        # l (level) goes from 1 to the curve degree + 1
        #alpha
        for l in range(1,degree+1+1):
            # build level l of the pyramid
            for i in range(s , s-degree+l-1,-1):
                alpha = (t - knots[i]) / (knots[i+degree+1-l] - knots[i])

                # interpolate each component
                for j in range(d+1): 
                    v[i,j] = (1 - alpha) * v[i-1,j] + alpha * v[i,j]


        # convert back to cartesian and return
        for i in range(d):
            results[i] = v[s,i] / v[s,d]

        return results
        
        
    def calculateSwing(self , phi_sw , V , Tsw, angle):#phi between [0,1), angle in degrees
        c = np.cos(np.deg2rad(angle))#cylindrical coordinates
        s = np.sin(np.deg2rad(angle))
                              
        degree = 5
        #Tst = 0.225 #sec
        l = np.abs(V)*Tsw/2.
        h = 0.02
        pointsAcc = np.array([[- l/2. ,  0],
                              [- l/2. -  l , 0],
                              [- l/2. - 2. * l , 0],
                              [- 2. * l , h],
                              [- l , h],
                              [0 , h]])
        pointsDec = np.array([[0 ,  h],
                              [l , h],
                              [2. * l , h],
                              [l/2. + 2. * l , 0],
                              [l/2. + l , 0],
                              [l/2. , 0]])

        knots = np.array([0,0,0,0,0,0,1,1,1,1,1,1])
        
        if (phi_sw < 0.5):
            sw = phi_sw*2
            result = self.BSpline(sw, degree, pointsAcc, knots)
        elif (phi_sw >= 0.5):
            sw = (phi_sw - 0.5)*2
            result = self.BSpline(sw, degree, pointsDec, knots)
        
        swingX = c * result[0]
        swingY = - s * result[0]
        swingZ = result[1]

        return swingX, swingY , swingZ
    
        
    def stepTrajectory(self , phi , V , T , angle , Wrot , centerToFoot , stepOffset=0.5): #phi belong [0,1), angles in degrees
        """  Step offset defines the time each phase long. 0.5 means stance and swing phases take half the period each.
             0.25 means stance fase takes 25% of the total period.  """

        #calculate Period of Stance and Swing phases
        Tst = T*stepOffset
        Tsw = T*(1.-stepOffset)


        if (phi >= 1):
            phi = phi - 1.
        #step describes a circuference in order to rotate
        r = np.sqrt(centerToFoot[0]**2 + centerToFoot[1]**2) #radius of the ciscunscribed circle
        footAngle = np.arctan2(centerToFoot[1],centerToFoot[0]) 
        
        if Wrot >= 0.:#As it is defined inside cylindrical coordinates, when Wrot < 0, this is the same as rotate it 180ª
            circleTrayectory = 90. - np.rad2deg(footAngle - self.alpha)
        else:
            circleTrayectory = 270. - np.rad2deg(footAngle - self.alpha)
        
        
        if phi <= stepOffset: #stance phase
            phiStance = phi/stepOffset
            stepX_long , stepY_long , stepZ_long = self.calculateStance(phiStance , V  , Tst, angle)#longitudinal step
            stepX_rot , stepY_rot , stepZ_rot = self.calculateStance(phiStance , Wrot , Tst , circleTrayectory)#rotational step
#            print(phi,phiStance, stepX_long)
        else: #swing phase
            phiSwing = (phi-stepOffset)/(1-stepOffset)
            stepX_long , stepY_long , stepZ_long = self.calculateSwing(phiSwing , V , Tsw , angle)#longitudinal step
            stepX_rot , stepY_rot , stepZ_rot = self.calculateSwing(phiSwing , Wrot , Tsw , circleTrayectory)#rotational step

        #alpha refers to the angle tangential to step trajectory plane
        if (centerToFoot[1] > 0):#define the sign for every quadrant 
            if (stepX_rot < 0):
                self.alpha = -np.arctan2(np.sqrt(stepX_rot**2 + stepY_rot**2) , r)
            else:
                self.alpha = np.arctan2(np.sqrt(stepX_rot**2 + stepY_rot**2) , r)
        else:
            if (stepX_rot < 0):
                self.alpha = np.arctan2(np.sqrt(stepX_rot**2 + stepY_rot**2) , r)
            else:
                self.alpha = -np.arctan2(np.sqrt(stepX_rot**2 + stepY_rot**2) , r)

        coord = np.empty(3)        
        coord[0] = stepX_long + stepX_rot
        coord[1] = stepY_long + stepY_rot
        coord[2] = stepZ_long + stepZ_rot
        
        return coord
    
    #computes step trajectory for every foot, defining L which is like velocity command, its angle, 
    #offset between each foot, period of time of each step and the initial vector from center of robot to feet.
    def loop(self , V , angle , Wrot , T , offset , bodytoFeet_ ):
        
        if T <= 0.01: 
            T = 0.01
        
        if ((time.time()-self.lastTime)/T >= 0.99):
            self.lastTime = time.time()
        self.phi = (time.time()-self.lastTime)/T
#        print(self.phi)
        #now it calculates step trajectory for every foot
        step_coord = self.stepTrajectory(self.phi + offset[0] , V , T , angle , Wrot , np.squeeze(np.asarray(bodytoFeet_[0,:]))) #FR
        self.bodytoFeet[0,0] =  bodytoFeet_[0,0] + step_coord[0]
        self.bodytoFeet[0,1] =  bodytoFeet_[0,1] + step_coord[1] 
        self.bodytoFeet[0,2] =  bodytoFeet_[0,2] - step_coord[2]
    
        step_coord = self.stepTrajectory(self.phi + offset[1] , V , T , angle , Wrot , np.squeeze(np.asarray(bodytoFeet_[1,:])))#FL
        self.bodytoFeet[1,0] =  bodytoFeet_[1,0] + step_coord[0]
        self.bodytoFeet[1,1] =  bodytoFeet_[1,1] + step_coord[1]
        self.bodytoFeet[1,2] =  bodytoFeet_[1,2] - step_coord[2]
        
        step_coord = self.stepTrajectory(self.phi + offset[2] , V , T , angle , Wrot , np.squeeze(np.asarray(bodytoFeet_[2,:])))#BR
        self.bodytoFeet[2,0] =  bodytoFeet_[2,0] + step_coord[0]
        self.bodytoFeet[2,1] =  bodytoFeet_[2,1] + step_coord[1]
        self.bodytoFeet[2,2] =  bodytoFeet_[2,2] - step_coord[2]

        step_coord = self.stepTrajectory(self.phi + offset[3] , V , T , angle , Wrot , np.squeeze(np.asarray(bodytoFeet_[3,:])))#BL
        self.bodytoFeet[3,0] =  bodytoFeet_[3,0] + step_coord[0]
        self.bodytoFeet[3,1] =  bodytoFeet_[3,1] + step_coord[1]
        self.bodytoFeet[3,2] =  bodytoFeet_[3,2] - step_coord[2]
#            

        return self.bodytoFeet
    
