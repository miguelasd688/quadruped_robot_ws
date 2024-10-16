import time
import csv
import sys
import numpy as np

import rclpy
from rclpy.node import Node
from robot_interfaces.msg import Commands
from robot_interfaces.msg import RobotStatus

from simple_pid import PID
from quadruped_robot.src.kinematic_model import RobotKinematics
from quadruped_robot.src.gait_planner import trotGait
from quadruped_robot.src import angleToPulse

fieldnames = ["t",
            "FRc","FRf","FRt",
            "FLc","FLf","FLt",
            "BRc","BRf","BRt",
            "BLc","BLf","BLt",
            "bodyAngle"]

"initial foot position"
#angles
targetAngs = np.array([0 , np.pi/4 , -np.pi/2, 0 ,#BR
                                0 , np.pi/4 , -np.pi/2, 0 ,#BL
                                0 , np.pi/4 , -np.pi/2, 0 ,#FL
                                0 , np.pi/4 , -np.pi/2, 0 ])#FR
#foot separation (0.182 -> tetta=0) and distance to floor
Xdist = 0.22
Ydist = 0.14
height = 0.12
#body frame to foot frame vector (0.08/-0.11 , -0.07 , -height)
bodytoFeet0 = np.matrix([[ Xdist/2. , -Ydist/2. , height],
                                [ Xdist/2. ,  Ydist/2. , height],
                                [-Xdist/2. , -Ydist/2. , height],
                                [-Xdist/2. ,  Ydist/2. , height]])

bodytoFeet_REST = np.matrix([[ 0.124 , -0.1 , 0.018],
                                    [ 0.124 ,  0.1 , 0.018],
                                    [-0.124 , -0.1 , 0.018],
                                    [-0.124 ,  0.1 , 0.018]])#0.018

orn = np.array([0. , 0. , 0.])
pos = np.array([0. , 0. , 0.])
commandBodyAngle = 0.
Upid_yorn = [0.]
Upid_y = [0.]
Upid_xorn = [0.]
Upid_x = [0.]
startTime = time.time()
lastTime = startTime
t = []
en = 0
T = 0.4 #period of time (in seconds) of every step
offset = np.array([0.5 , 0 , 0.0 , 0.5]) #defines the offset between each foot step in this order (FR,FL,BR,BL)
                                                # [0. , 0.25 , 0.75 , 0.5] creep gait
interval = 0.015 #loop time.
bodyAngle = 0.0
kill = False
rest = True
CTRL1 = False
CTRL2 = False
lr = True
bodytoFeet = bodytoFeet_REST.copy()
pid = PID(-0.05, 0.5, 0.0, setpoint=0.0)
pid.sample_time = interval
pid.output_limits = (np.deg2rad(-20) , np.deg2rad(20))
poseMode=False


class RobotRun(Node):
    
    def __init__(self):
        super().__init__('robot_run_node')
        self.timer = self.create_timer(interval, self.robot_main_loop)
        self.get_logger().info('Robot Run node has started.')
        self.cmd = self.create_subscription(
            Commands,
            'commands_topic',
            self.listener_cmd_callback,
            10)
        self.status = self.create_subscription(
            RobotStatus,
            'status_topic',
            self.listener_status_callback,
            10)
        self.subscriptions  # prevent unused variable warning

        robotKinematics = RobotKinematics('><')
        trot = trotGait()

        
        print('Loop t: ', interval , ', Feet pos: (X = 0.248 Y = 0.2 Z = 0.018) m , REST = ',rest)
        #pdb.set_trace()
        n=0


        angles , transformedBodytoFeet = robotKinematics.solve(commandBodyAngle , orn, pos, bodytoFeet)
        pulsesCommand = angleToPulse.convert(angles, commandBodyAngle)
        time.sleep(0.02)



    def listener_cmd_callback(self, msg):
        self.linear_velocity = msg.linear_velocity
        self.linear_angle = msg.linear_angle
        self.angular_velocity = msg.angular_velocity
        self.com_position = msg.com_position  
        self.com_orientation = msg.com_orientation 

        self.get_logger().info(f"Received Command:")
        self.get_logger().info(f"Linear Velocity: {self.linear_velocity}")
        self.get_logger().info(f"Linear Angle: {self.linear_angle}")
        self.get_logger().info(f"Angular Velocity: {self.angular_velocity}")
        self.get_logger().info(f"CoM Position: {self.com_position}")
        self.get_logger().info(f"CoM Orientation: {self.com_orientation}")

    def listener_status_callback(self, msg):
        self.kill_flag = msg.kill_flag
        self.rest_flag = msg.rest_flag
        self.compliant_mode = msg.compliant_mode
        self.pose_mode = msg.pose_mode  

        self.get_logger().info(f"Received Command:")
        self.get_logger().info(f"Linear Velocity: {self.kill_flag}")
        self.get_logger().info(f"Linear Angle: {self.rest_flag}")
        self.get_logger().info(f"Angular Velocity: {self.compliant_mode}")
        self.get_logger().info(f"CoM Position: {self.pose_mode}")

    with open('src/quadruped_robot/quadruped_robot/telemetry/data.csv','w') as csv_file:
        csv_writer = csv.DictWriter(csv_file,fieldnames = fieldnames)
        csv_writer.writeheader()



    
    def update_data(self, bodyAngle , angles):
        with open('telemetry/data.csv','a') as csv_file:
            csv_writer = csv.DictWriter(csv_file, fieldnames = fieldnames)
            info = {"t" :  round(t,2),
                    "FRc" : round(angles[0,0],1),"FRf" : round(angles[0,1],1),"FRt" : round(angles[0,2],1),
                    "FLc" : round(angles[1,0],1),"FLf" : round(angles[1,1],1),"FLt" : round(angles[1,2],1),
                    "BRc" : round(angles[2,0],1),"BRf" : round(angles[2,1],1),"BRt" : round(angles[2,2],1),
                    "BLc" : round(angles[3,0],1),"BLf" : round(angles[3,1],1),"BLt" : round(angles[3,2],1),
                    "bodyAngle" : round(bodyAngle,1)}
            csv_writer.writerow(info)

    def moveFeetZ(self, dt , fromHeight , toHeight , moveTime):
        nit = int(moveTime/dt) #time in sec to change feet pos.
        l1 = np.array([0,1])
        l2 = np.array([3,2])
        moveZ1 = np.linspace(fromHeight , toHeight , nit)
        moveZ2 = np.linspace(fromHeight , toHeight , nit)
        moveZ3 = np.linspace(fromHeight , toHeight , nit)
        moveZ4 = np.linspace(fromHeight , toHeight , nit)
        print('Standing up')
        it = int(0)
        MVlastTime=0
        while it < nit:
            if (time.time()-MVlastTime >= dt):
                MVloopTime = time.time() - MVloopTime
                MVlastTime = time.time()
                #commandBodyAngle , commandPose , commandOrn , V , angle , Wrot , T , compliantMode , kill , rest , poseMode = joystick.read()

                bodytoFeet[0,2] = moveZ1[it]
                bodytoFeet[1,2] = moveZ2[it]
                bodytoFeet[2,2] = moveZ3[it]
                bodytoFeet[3,2] = moveZ4[it]
                #####################################################################################
                #####   kinematics Model: Input body orientation, deviation and foot position    ####
                #####   and get the angles, neccesary to reach that position, for every joint    ####
                angles , transformedBodytoFeet = robotKinematics.solve(bodyAngle, orn + self.com_orientation , pos + self.com_position , bodytoFeet)
                pulsesCommand = angleToPulse.convert(angles, bodyAngle)
                update_data(bodyAngle , angles )
                it = it + 1

    def layDownMove(self, dt , bodyAngle , bodytoFeet , bodytoFeet0):
    ######            RESTING ROUTINE           ##################
        h = 0.06 # step height in m.

        print('Init REST')
        moveFeetTo(dt , bodytoFeet , bodytoFeet_REST , 0.3 , h)

        print('Going rest')
        moveFeetZ(dt , bodytoFeet[0,2] , 0.018 , 1.0)

    def robotResting():
        bodytoFeet = bodytoFeet_REST.copy()

        angles , transformedBodytoFeet = robotKinematics.solve(bodyAngle, orn , pos , bodytoFeet)
        pulsesCommand = angleToPulse.convert(angles, bodyAngle)
        update_data(bodyAngle , angles )

    def standMove(self, dt , bodyAngle , bodytoFeet , bodytoFeet0):
        ######            STAND UP ROUTINE           ########################
        h = 0.06 # step height in m.

        print('Standing up')
        moveFeetZ(dt , 0.018 , bodytoFeet0[0,2] , 1.0)

        print('Init pose 0')
        moveFeetTo(dt , bodytoFeet , bodytoFeet0 , 0.3 , h)



    def exitProgram(self, bodyAngle , bodytoFeet):
        print("Reseting microcontroller and closing...")
        time.sleep(0.4)
        #commandBodyAngle , commandPose , commandOrn , V , angle , Wrot , T , compliantMode , kill , rest , poseMode = joystick.read()

        en = 1
        ####################################################################################
        #####   kinematics Model: Input body orientation, deviation and foot position    ####
        #####   and get the angles, neccesary to reach that position, for every joint    ####
        angles , transformedBodytoFeet = robotKinematics.solve(bodyAngle, orn, pos, bodytoFeet)
        pulsesCommand = angleToPulse.convert(angles, bodyAngle)   #            time.sleep(8)
    #            arduino.connect()
    #            bodyAngle , commandPose , commandOrn , V , angle , Wrot , T , compliantMode , kill , poseMode = joystick.read()
    #            kill = False
    #            en = 0




    def robot_main_loop(self):
        self.get_logger().info('Robot Run node is running.')
        """loopTime = time.time() - lastTime
        lastTime = time.time()
        t = time.time() - startTime"""
        #commandBodyAngle , commandPose , commandOrn , V , angle , Wrot , T , compliantMode , kill , rest , poseMode = joystick.read()
        if (kill == False):
            en = 0
            #arduinoLoopTime , encoderAng = arduino.serialRecive()#recive serial message    |  , linearAcc , eulerAngles , angularVelocity
            if (rest == True and lr == True):
                robotResting()

            elif (rest == False and lr == False):
                #calculates the feet coord for gait, defining length of the step and direction (0º -> forward; 180º -> backward)
                if (poseMode == False):
                    bodytoFeet  = trot.loop( self.linear_velocity , self.linear_angle , self.angular_velocity , T , offset , bodytoFeet0)
                else:
                    bodytoFeet = bodytoFeet0.copy() # Static position.
#                     pos[0] = 0.005*np.sin(2*np.pi*t/2.)
#                     pos[1] = 0.005*np.cos(2*np.pi*t/2.)
                    #commandBodyAngle = - np.deg2rad(10)*np.cos(2*np.pi*t/T)
#                     commandBodyAngle = pid(eulerAngles[1])
#                    print(loopTime )
                    #commandPose[1] = -commandBodyAngle/20.
                    #####################################################################################
                    #####   kinematics Model: Input body orientation, deviation and foot position    ####
                    #####   and get the angles, neccesary to reach that position, for every joint    ####
                angles , transformedBodytoFeet = robotKinematics.solve(commandBodyAngle , orn + com_orientation, pos + com_position, bodytoFeet)
                pulsesCommand = angleToPulse.convert(angles, commandBodyAngle)         
                update_data(commandBodyAngle , angles)

            elif (rest == True and lr == False):
                layDownMove(interval , bodyAngle , bodytoFeet , bodytoFeet0)
                print('Loop t: ', interval , ', Feet pos: (X = 0.248 Y = 0.2 Z = 0.018) m , REST == True')

            elif (rest == False and lr == True):
                standMove(interval , bodyAngle , bodytoFeet , bodytoFeet0)
                print('Loop t: ', interval , ', Feet pos: (X = 0.248 Y = 0.1 Z = 0.120) m , REST == False')

            lr = rest
        else:
            exitProgram(bodyAngle , bodytoFeet)



def main(args=None):
    rclpy.init(args=args)
    node = RobotRun()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()









