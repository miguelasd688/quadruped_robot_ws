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


class RobotRun(Node):
    
    def __init__(self):
        super().__init__('robot_run')
        self.get_parameters()

        self.timer = self.create_timer(self.loop_latency, self.robot_main_loop)
        self.get_logger().info('Robot Run node has started.')
        self.cmd = self.create_subscription(
            Commands,
            'controller_command',
            self.listener_cmd_callback,
            100)
        self.status = self.create_subscription(
            RobotStatus,
            'controller_status',
            self.listener_status_callback,
            100)
        self.subscriptions  # prevent unused variable warning
        self.status_received = False
        self.cmd_received = False
        
#        with open('src/quadruped_robot/quadruped_robot/telemetry/data.csv','w') as csv_file:
#            csv_writer = csv.DictWriter(csv_file,fieldnames = self.fieldnames)
#            csv_writer.writeheader()


        self.start_time = time.time()
        self.t = time.time() - self.start_time
        self.last_time = self.t
        self.rest_flag = True
        self.lr = self.rest_flag
        self.armed = False

        pid = PID(-0.05, 0.5, 0.0, setpoint=0.0)
        pid.sample_time = self.loop_latency
        pid.output_limits = (np.deg2rad(-20) , np.deg2rad(20))

        self.body_to_feet = self.body_to_feet_rest.copy()
        self.robotKinematics = RobotKinematics('><')
        self.trot = trotGait()
        angles , transformedBodytoFeet = self.robotKinematics.solve(self.orientation0, 
                                                                    self.position0, 
                                                                    self.body_to_feet)
        pulsesCommand = angleToPulse.convert(angles)
        print('Loop t: ', self.loop_latency , ', Feet pos: ', self.body_to_feet[0] ,' REST = ', self.rest_flag)
        #pdb.set_trace()


    def get_parameters(self) :
        self.declare_parameter('fieldnames', ['','','','',
                                                 '','','',
                                                 '','','',
                                                 '','',''])
        self.declare_parameter('body_to_feet0', [0.0 , 0.0 , 0.0,
                                                 0.0 , 0.0 , 0.0,
                                                 0.0 , 0.0 , 0.0,
                                                 0.0 , 0.0 , 0.0])
        self.declare_parameter('body_to_feet_rest', [0.0 , 0.0 , 0.0,
                                                     0.0 , 0.0 , 0.0,
                                                     0.0 , 0.0 , 0.0,
                                                     0.0 , 0.0 , 0.0])
        self.declare_parameter('com_orientation0', [0.0 , 0.0 , 0.0])
        self.declare_parameter('com_position0', [0.0 , 0.0 , 0.0])
        self.declare_parameter('gait_offset0', [0.0 , 0.0 , 0.0])
        self.declare_parameter('step_period0', 0.0)
        self.declare_parameter('loop_latency', 0.0)

        self.fieldnames = self.get_parameter('fieldnames').get_parameter_value().string_array_value
        body_to_feet0_flat = self.get_parameter('body_to_feet0').get_parameter_value().double_array_value
        self.body_to_feet0 = np.array([body_to_feet0_flat[i:i + 3] for i in range(0, len(body_to_feet0_flat), 3)])
        body_to_feet_rest_flat = self.get_parameter('body_to_feet_rest').get_parameter_value().double_array_value
        self.body_to_feet_rest = np.array([body_to_feet_rest_flat[i:i + 3] for i in range(0, len(body_to_feet_rest_flat), 3)])
        self.orientation0 = np.array(self.get_parameter('com_orientation0').get_parameter_value().double_array_value)
        self.position0 = np.array(self.get_parameter('com_position0').get_parameter_value().double_array_value)
        self.gait_offset = np.array(self.get_parameter('gait_offset0').get_parameter_value().double_array_value)
        self.step_period0 = self.get_parameter('step_period0').get_parameter_value().double_value
        self.loop_latency = self.get_parameter('loop_latency').get_parameter_value().double_value

        self.get_logger().info(f"loop_latency: {self.loop_latency}, loop_latency: {self.loop_latency}")
    
    
    def listener_cmd_callback(self, msg):
        self.linear_velocity = msg.linear_velocity
        self.linear_angle = msg.linear_angle
        self.angular_velocity = msg.angular_velocity
        self.com_position = msg.com_position  
        self.com_orientation = msg.com_orientation 

        self.cmd_received = True
        """
        self.get_logger().info(f"Received Command:")
        self.get_logger().info(f"Linear Velocity: {self.linear_velocity}")
        self.get_logger().info(f"Linear Angle: {self.linear_angle}")
        self.get_logger().info(f"Angular Velocity: {self.angular_velocity}")
        self.get_logger().info(f"CoM Position: {self.com_position}")
        self.get_logger().info(f"CoM Orientation: {self.com_orientation}")
        """

    def listener_status_callback(self, msg):
        self.kill_flag = msg.kill_flag
        self.rest_flag = msg.rest_flag
        self.compliant_mode = msg.compliant_mode
        self.pose_mode = msg.pose_mode  

        self.status_received = True

        """
        self.get_logger().info(f"Received Command:")
        self.get_logger().info(f"kill: {self.kill_flag}")
        self.get_logger().info(f"rest: {self.rest_flag}")
        self.get_logger().info(f"compliant mode: {self.compliant_mode}")
        self.get_logger().info(f"pose mode: {self.pose_mode}")
        """

    def moveFeetTo(self , new_body_to_feet , h , moveTime):
        # Move feet position to desired new position. It moves 2 diagonal legs at a time
        # time in seconds to make the movement
        # h: step height in m.

        l1 = np.array([0,1])
        l2 = np.array([3,2])
        moveTime = moveTime/2.
        nit = int(moveTime/self.loop_latency)
        for k in range(2):
            moveX1 = np.linspace(self.body_to_feet[l1[k],0] , new_body_to_feet[l1[k],0] , nit)
            moveY1 = np.linspace(self.body_to_feet[l1[k],1] , new_body_to_feet[l1[k],1] , nit)
            moveX2 = np.linspace(self.body_to_feet[l2[k],0] , new_body_to_feet[l2[k],0] , nit)
            moveY2 = np.linspace(self.body_to_feet[l2[k],1] , new_body_to_feet[l2[k],1] , nit)

            it = int(0)
            MVlastTime=0
            while it < nit:
                if (time.time()-MVlastTime >= self.loop_latency):
                    MVloopTime = time.time() - MVlastTime
                    MVlastTime = time.time()

                    self.body_to_feet[l1[k],0] = moveX1[it]
                    self.body_to_feet[l1[k],1] = moveY1[it]
                    self.body_to_feet[l1[k],2] = self.body_to_feet0[l1[k],2] + ( 4*h*(it/nit-0.5)**2 - h )

                    self.body_to_feet[l2[k],0] = moveX2[it]
                    self.body_to_feet[l2[k],1] = moveY2[it]
                    self.body_to_feet[l2[k],2] = self.body_to_feet0[l2[k],2] + ( 4*h*(it/nit-0.5)**2 - h )

                    angles , transformedBodytoFeet = self.robotKinematics.solve(self.orientation0, 
                                                                                self.position0, 
                                                                                self.body_to_feet)
                    pulsesCommand = angleToPulse.convert(angles)
                    it = it + 1


    def moveFeetZ(self , fromHeight , toHeight , moveTime):
        nit = int(moveTime/self.loop_latency) #time in sec to change feet pos.
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
            if (time.time()-MVlastTime >= self.loop_latency):
                MVloopTime = time.time() - MVlastTime
                MVlastTime = time.time()

                self.body_to_feet[0,2] = moveZ1[it]
                self.body_to_feet[1,2] = moveZ2[it]
                self.body_to_feet[2,2] = moveZ3[it]
                self.body_to_feet[3,2] = moveZ4[it]

                angles , transformedBodytoFeet = self.robotKinematics.solve(self.orientation0, 
                                                                            self.position0, 
                                                                            self.body_to_feet)
                pulsesCommand = angleToPulse.convert(angles)
                it = it + 1


    def robotResting(self):
        self.body_to_feet = self.body_to_feet_rest.copy()

        angles , transformedBodytoFeet = self.robotKinematics.solve(self.orientation0, 
                                                                    self.position0, 
                                                                    self.body_to_feet)
        pulsesCommand = angleToPulse.convert(angles)


    def layDownMove(self):
    ######            RESTING ROUTINE           ##################
        h = 0.06 # step height in m.

        print('Init REST')
        self.moveFeetTo(self.body_to_feet_rest , h , 0.3)

        print('Going rest')
        self.moveFeetZ(self.body_to_feet[0,2] , 0.018 , 1.0)


    def standMove(self):
    ######            STAND UP ROUTINE           ########################
        h = 0.06 # step height in m.

        print('Standing up')
        self.moveFeetZ(0.018 , self.body_to_feet0[0,2] , 1.0)

        print('Init pose 0')
        self.moveFeetTo(self.body_to_feet0 , h , 0.3)


    def exitProgram(self):
        print("Reseting microcontroller and closing...")
        time.sleep(0.4)

        angles , transformedBodytoFeet = self.robotKinematics.solve(self.orientation0, 
                                                                    self.position0, 
                                                                    self.body_to_feet)
        pulsesCommand = angleToPulse.convert(angles) 


    def robot_main_loop(self):
        if not (self.cmd_received and self.status_received):
            self.get_logger().info("Waiting for command and status messages...")
            return
        if not (self.armed):
            self.armed = True
            self.get_logger().info("Robot armed")

        loop_time = time.time() - self.last_time
        self.last_time = time.time()
        self.t = time.time() - self.start_time
        if (self.kill_flag == False):
            if (self.rest_flag == True and self.lr == True):
                self.robotResting()
            elif (self.rest_flag == False and self.lr == False):
                if (self.pose_mode == False):
                    self.body_to_feet  = self.trot.loop(self.linear_velocity , self.linear_angle , 
                                                        self.angular_velocity , self.step_period0 , 
                                                        self.gait_offset , self.body_to_feet0)
                else:
                    self.body_to_feet = self.body_to_feet0.copy() # Static position.
                angles , transformedBodytoFeet = self.robotKinematics.solve(self.orientation0 + self.com_orientation, 
                                                                            self.position0 + self.com_position, 
                                                                            self.body_to_feet)
                pulsesCommand = angleToPulse.convert(angles)         
            elif (self.rest_flag == True and self.lr == False):
                self.layDownMove()
                print('Loop t: ', loop_time , ', Feet pos: (X = 0.248 Y = 0.2 Z = 0.018) m , REST == True')
            elif (self.rest_flag == False and self.lr == True):
                self.standMove()
                print('Loop t: ', loop_time , ', Feet pos: (X = 0.248 Y = 0.1 Z = 0.120) m , REST == False')
            self.lr = self.rest_flag
        else:
            self.exitProgram()


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









