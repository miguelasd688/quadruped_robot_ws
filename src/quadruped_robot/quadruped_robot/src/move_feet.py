import time
import numpy as np

from . import angleToPulse

def moveFeetTo(dt, body , new_body_to_feet , h , moveTime):
    # Move feet position to desired new position. It moves 2 diagonal legs at a time
    # time in seconds to make the movement
    # h: step height in m.
    l1 = np.array([0,1])
    l2 = np.array([3,2])
    moveTime = moveTime/2.
    nit = int(moveTime/dt)
    for k in range(2):
        moveX1 = np.linspace(body.to_feet[l1[k],0] , new_body_to_feet[l1[k],0] , nit)
        moveY1 = np.linspace(body.to_feet[l1[k],1] , new_body_to_feet[l1[k],1] , nit)
        moveX2 = np.linspace(body.to_feet[l2[k],0] , new_body_to_feet[l2[k],0] , nit)
        moveY2 = np.linspace(body.to_feet[l2[k],1] , new_body_to_feet[l2[k],1] , nit)
        start_body_to_feet = body.to_feet
        it = int(0)
        MVlastTime=0
        while it < nit:
            if (time.time()-MVlastTime >= dt):
                MVloopTime = time.time() - MVlastTime
                MVlastTime = time.time()
                body.to_feet[l1[k],0] = moveX1[it]
                body.to_feet[l1[k],1] = moveY1[it]
                body.to_feet[l1[k],2] = start_body_to_feet[l1[k],2] + ( 4*h*(it/nit-0.5)**2 - h )
                body.to_feet[l2[k],0] = moveX2[it]
                body.to_feet[l2[k],1] = moveY2[it]
                body.to_feet[l2[k],2] = start_body_to_feet[l2[k],2] + ( 4*h*(it/nit-0.5)**2 - h )
                
                angles , transformedBodytoFeet = body.kinematics.solve(body.orientation, 
                                                                            body.position, 
                                                                            body.to_feet)
                pulsesCommand = angleToPulse.convert(angles)
                it = it + 1


def moveFeetZ(dt , body , to_height , move_Time):
    nit = int(move_Time/dt) #time in sec to change feet pos.
    l1 = np.array([0,1])
    l2 = np.array([3,2])
    moveZ1 = np.linspace(body.to_feet[0,2] , to_height , nit)
    moveZ2 = np.linspace(body.to_feet[1,2] , to_height , nit)
    moveZ3 = np.linspace(body.to_feet[2,2] , to_height , nit)
    moveZ4 = np.linspace(body.to_feet[3,2] , to_height , nit)
    print('Standing up')
    it = int(0)
    MVlastTime=0
    while it < nit:
        if (time.time()-MVlastTime >= dt):
            MVloopTime = time.time() - MVlastTime
            MVlastTime = time.time()
            body.to_feet[0,2] = moveZ1[it]
            body.to_feet[1,2] = moveZ2[it]
            body.to_feet[2,2] = moveZ3[it]
            body.to_feet[3,2] = moveZ4[it]
            angles , transformedBodytoFeet = body.kinematics.solve(body.orientation, 
                                                                        body.position, 
                                                                        body.to_feet)
            pulsesCommand = angleToPulse.convert(angles)
            it = it + 1