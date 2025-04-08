import numpy as np

class CalibrationController:
    def __init__(self, loop_latency, body_to_feet, calibration_body_to_feet):
        self.is_routine_active = False
        self.loop_latency = loop_latency
        self.move_time = 0.6
        self.n_iterations = int(self.move_time/self.loop_latency)
        self.sweep_move = np.zeros((self.n_iterations+1,3))
        self.nit = 0

    def executeRoutine(self, body_to_feet, desired_body_to_feet, confirm):
        output_body_to_feet = desired_body_to_feet

        if (confirm and not self.is_routine_active):
            self.is_routine_active = True
            
            
            
            for i in range(3):
                self.sweep_move[:,i] = np.linspace(desired_body_to_feet[i], body_to_feet[i], self.n_iterations + 1)
            self.nit = 0
            print(f'Starting sweep to make interpolation')   
            
        elif (self.is_routine_active):
            for i in range(3):
                output_body_to_feet[i] = self.sweep_move[self.nit,i] 
            print(f'Output: {output_body_to_feet}') 
            self.nit += 1

            if (self.nit >= len(self.sweep_move[:,0])):
                self.is_routine_active = False
                print(f'Interpolation finished') 


        return output_body_to_feet

