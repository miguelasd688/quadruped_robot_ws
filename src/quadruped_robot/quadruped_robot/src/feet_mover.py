import time
import numpy as np

class FeetMover():

    def __init__ (self, start_body_to_feet, target_body_to_feet , nit):
        # define index for simultaneously move diagonal legs
        self.l1 = np.array([0,1])
        self.l2 = np.array([3,2])

        self.nnit = int(nit/2)
        self.moveX1 = np.zeros((2 , self.nnit + 1))
        self.moveY1 = np.zeros((2 , self.nnit + 1))
        self.moveX2 = np.zeros((2 , self.nnit + 1))
        self.moveY2 = np.zeros((2 , self.nnit + 1))

        for i in range(2):
            self.moveX1[i,:] = np.linspace(start_body_to_feet[self.l1[i],0] , target_body_to_feet[self.l1[i],0] , self.nnit + 1)
            self.moveY1[i,:] = np.linspace(start_body_to_feet[self.l1[i],1] , target_body_to_feet[self.l1[i],1] , self.nnit + 1)
            self.moveX2[i,:] = np.linspace(start_body_to_feet[self.l2[i],0] , target_body_to_feet[self.l2[i],0] , self.nnit + 1)
            self.moveY2[i,:] = np.linspace(start_body_to_feet[self.l2[i],1] , target_body_to_feet[self.l2[i],1] , self.nnit + 1)

        self.moveZ1 = np.linspace(start_body_to_feet[0,2] , target_body_to_feet[0,2] , nit)
        self.moveZ2 = np.linspace(start_body_to_feet[1,2] , target_body_to_feet[1,2] , nit)
        self.moveZ3 = np.linspace(start_body_to_feet[2,2] , target_body_to_feet[2,2] , nit)
        self.moveZ4 = np.linspace(start_body_to_feet[3,2] , target_body_to_feet[3,2] , nit)

    def moveFeetTo(self, body_to_feet, start_body_to_feet , h , nit, it):
        # Move feet position to desired new position over a number of iterations. It make 2 diagonal legs steps at a time
        # h: step height in m.
    
        if (it < int(nit/2)):
            k = 0
        else:
            k = 1

        iit = it - k*self.nnit
        body_to_feet[self.l1[k],0] = self.moveX1[k, iit]
        body_to_feet[self.l1[k],1] = self.moveY1[k, iit]
        body_to_feet[self.l1[k],2] = start_body_to_feet[self.l1[k],2] + ( 4*h*((iit)/self.nnit-0.5)**2 - h )
        body_to_feet[self.l2[k],0] = self.moveX2[k, iit]
        body_to_feet[self.l2[k],1] = self.moveY2[k, iit]
        body_to_feet[self.l2[k],2] = start_body_to_feet[self.l2[k],2] + ( 4*h*((iit)/self.nnit-0.5)**2 - h )

        return body_to_feet



    def moveFeetZ(self, body_to_feet, it):
        # Move feet position to desired new position over a number of iterations. 

        body_to_feet[0,2] = self.moveZ1[it]
        body_to_feet[1,2] = self.moveZ2[it]
        body_to_feet[2,2] = self.moveZ3[it]
        body_to_feet[3,2] = self.moveZ4[it]

        return body_to_feet