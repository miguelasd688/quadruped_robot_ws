import time
import numpy as np

from . import angleToPulse

def moveFeetTo(body_to_feet, start_body_to_feet, new_body_to_feet , h , nit, it):
    # Move feet position to desired new position over a number of iterations. It make 2 diagonal legs steps at a time
    # h: step height in m.
    l1 = np.array([0,1])
    l2 = np.array([3,2])
   

    k = 0
    if (it < int(nit/2)):
        k = 0
    elif (it >= int(nit/2)):
        k = 1

    nnit = int(nit/2)
    iit = it - k*nnit

    moveX1 = np.linspace(start_body_to_feet[l1[k],0] , new_body_to_feet[l1[k],0] , nnit + 1)
    moveY1 = np.linspace(start_body_to_feet[l1[k],1] , new_body_to_feet[l1[k],1] , nnit + 1)
    moveX2 = np.linspace(start_body_to_feet[l2[k],0] , new_body_to_feet[l2[k],0] , nnit + 1)
    moveY2 = np.linspace(start_body_to_feet[l2[k],1] , new_body_to_feet[l2[k],1] , nnit + 1)
    
    body_to_feet[l1[k],0] = moveX1[iit]
    body_to_feet[l1[k],1] = moveY1[iit]
    body_to_feet[l1[k],2] = start_body_to_feet[l1[k],2] + ( 4*h*((iit)/nnit-0.5)**2 - h )
    body_to_feet[l2[k],0] = moveX2[iit]
    body_to_feet[l2[k],1] = moveY2[iit]
    body_to_feet[l2[k],2] = start_body_to_feet[l2[k],2] + ( 4*h*((iit)/nnit-0.5)**2 - h )

    return body_to_feet



def moveFeetZ(body_to_feet, start_body_to_feet , new_body_to_feet , nit, it):
    # Move feet position to desired new position over a number of iterations. 
    l1 = np.array([0,1])
    l2 = np.array([3,2])

    moveZ1 = np.linspace(start_body_to_feet[0,2] , new_body_to_feet[0,2] , nit)
    moveZ2 = np.linspace(start_body_to_feet[1,2] , new_body_to_feet[1,2] , nit)
    moveZ3 = np.linspace(start_body_to_feet[2,2] , new_body_to_feet[2,2] , nit)
    moveZ4 = np.linspace(start_body_to_feet[3,2] , new_body_to_feet[3,2] , nit)

    body_to_feet[0,2] = moveZ1[it]
    body_to_feet[1,2] = moveZ2[it]
    body_to_feet[2,2] = moveZ3[it]
    body_to_feet[3,2] = moveZ4[it]
    
    return body_to_feet