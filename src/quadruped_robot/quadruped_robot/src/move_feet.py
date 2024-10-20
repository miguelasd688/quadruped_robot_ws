import time
import numpy as np

from . import angleToPulse

def moveFeetTo(start_body_to_feet, new_body_to_feet , h , nit, it):
    # Move feet position to desired new position. It moves 2 diagonal legs at a time
    # time in seconds to make the movement
    # h: step height in m.
    l1 = np.array([0,1])
    l2 = np.array([3,2])
    body_to_feet = start_body_to_feet.copy()
    for k in range(2):
        moveX1 = np.linspace(start_body_to_feet[l1[k],0] , new_body_to_feet[l1[k],0] , nit)
        moveY1 = np.linspace(start_body_to_feet[l1[k],1] , new_body_to_feet[l1[k],1] , nit)
        moveX2 = np.linspace(start_body_to_feet[l2[k],0] , new_body_to_feet[l2[k],0] , nit)
        moveY2 = np.linspace(start_body_to_feet[l2[k],1] , new_body_to_feet[l2[k],1] , nit)
        
        body_to_feet[l1[k],0] = moveX1[it]
        body_to_feet[l1[k],1] = moveY1[it]
        body_to_feet[l1[k],2] = start_body_to_feet[l1[k],2] + ( 4*h*(it/nit-0.5)**2 - h )
        body_to_feet[l2[k],0] = moveX2[it]
        body_to_feet[l2[k],1] = moveY2[it]
        body_to_feet[l2[k],2] = start_body_to_feet[l2[k],2] + ( 4*h*(it/nit-0.5)**2 - h )

    return body_to_feet



def moveFeetZ(start_body_to_feet , new_body_to_feet , nit, it):
    l1 = np.array([0,1])
    l2 = np.array([3,2])
    body_to_feet = start_body_to_feet.copy()

    moveZ1 = np.linspace(start_body_to_feet[0,2] , new_body_to_feet[0,2] , nit)
    moveZ2 = np.linspace(start_body_to_feet[1,2] , new_body_to_feet[1,2] , nit)
    moveZ3 = np.linspace(start_body_to_feet[2,2] , new_body_to_feet[2,2] , nit)
    moveZ4 = np.linspace(start_body_to_feet[3,2] , new_body_to_feet[3,2] , nit)

    body_to_feet[0,2] = moveZ1[it]
    body_to_feet[1,2] = moveZ2[it]
    body_to_feet[2,2] = moveZ3[it]
    body_to_feet[3,2] = moveZ4[it]
    
    return body_to_feet