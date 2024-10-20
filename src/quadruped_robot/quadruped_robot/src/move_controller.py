
from .move_feet import moveFeetZ , moveFeetTo 

class MoveController():
    #robot Controller to generate secuence of movements
    def __init__(self, body, loop_latency):
        self.body = body
        self.loop_latency = loop_latency
        self.actions = {'ready': 0, 'heigh_set': 1, 'feet_place': 2, 'end': 3}
        self.action_now = self.actions['ready']
        self.movement_iterations = int(0)
        self.start_body_to_feet = self.body.to_feet.copy()
    
    #set start_body_to_feet
    def initAction(self):
        if(self.movement_iterations < 1):
            print('    |__|__init new secuence')
            self.start_body_to_feet = self.body.to_feet.copy()

    # reset iteractor and set next action
    def nextAction(self, action, n_iterations):
        if (self.movement_iterations >= n_iterations):
            self.movement_iterations = int(0)
            self.action_now = action

    # unpdate self.body.to_feet depending on the action
    def updateMovement(self, end_body_to_feet, action, n_iterations):
        if (action == self.actions['heigh_set']):
            if (self.movement_iterations < n_iterations):
                self.body.to_feet = moveFeetZ(self.body.to_feet, self.start_body_to_feet , end_body_to_feet , n_iterations, self.movement_iterations)
                #print('    |__|__|__Setting robot height')
                #print(f'foot: {self.body.to_feet[0,:]} start: {self.start_body_to_feet[0,:]} | it: {self.movement_iterations} of {n_iterations}')
                self.movement_iterations += 1
        elif (action == self.actions['feet_place']):
            h = 0.06
            if (self.movement_iterations < n_iterations):
                self.body.to_feet = moveFeetTo(self.body.to_feet, self.start_body_to_feet , end_body_to_feet , h , n_iterations, self.movement_iterations)
                #print('    |__|__|__Placing feet to new position')
                #print(f'foot: {self.body.to_feet[0,:]} start: {self.start_body_to_feet[0,:]} | it: {self.movement_iterations} of {n_iterations}')
                self.movement_iterations += 1


    ## STAND UP ROUTINE
    def updateStandUp(self, desired_body_to_feet):
        move_done = False
        print("    |__Standing up move in progress...")
        if (self.action_now == self.actions['end']):
            move_done = True
            self.action_now = self.actions['ready']
        elif (self.action_now == self.actions['ready']):
            self.action_now = self.actions['heigh_set']

        if (self.action_now == self.actions['heigh_set']):
            move_time = 0.6
            n_iterations = int(move_time/self.loop_latency)
            self.initAction()
            self.updateMovement(desired_body_to_feet, self.action_now, n_iterations)
            self.nextAction(self.actions['feet_place'], n_iterations)
        
        if (self.action_now == self.actions['feet_place']):          
            move_time = 1.3
            n_iterations = int(move_time/self.loop_latency)
            self.initAction()
            self.updateMovement(desired_body_to_feet, self.action_now, n_iterations)
            self.nextAction(self.actions['end'], n_iterations)

        return self.body.to_feet, move_done

    ## LAY DOWN ROUTINE
    def updateLayDown(self, desired_body_to_feet):
        print("    |__Laying down move in progress...")
        move_done = False
        if (self.action_now == self.actions['end']):
            move_done = True
            self.action_now = self.actions['ready']
        elif (self.action_now == self.actions['ready']):
            self.action_now = self.actions['feet_place']

        if (self.action_now == self.actions['feet_place']):
            move_time = 1.3
            n_iterations = int(move_time/self.loop_latency)
            self.initAction()
            self.updateMovement(desired_body_to_feet, self.action_now, n_iterations)
            self.nextAction(self.actions['heigh_set'], n_iterations)
        
        elif (self.action_now == self.actions['heigh_set']):
            move_time = 0.6
            n_iterations = int(move_time/self.loop_latency)
            self.initAction()
            self.updateMovement(desired_body_to_feet, self.action_now, n_iterations)
            self.nextAction(self.actions['end'], n_iterations)
            
        return self.body.to_feet, move_done