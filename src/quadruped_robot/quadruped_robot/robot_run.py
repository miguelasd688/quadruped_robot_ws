import time
import csv
import sys
import numpy as np
from os import system, name

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Int8MultiArray
from std_msgs.msg import Float32MultiArray

from .src.robot_player import RobotPlayer
from .src.state_rest import RestState




class RobotRun(Node):
    
    def __init__(self):
        super().__init__('robot_run')
        parameters = self.get_parameters()
        self.loop_latency = parameters['loop_latency']
        self.status_received = False
        self.cmd_received = False

        self.robot_player = RobotPlayer(RestState(), parameters)
        self.robot_player.updateRest()

        self.start_time = time.time()
        self.t = time.time() - self.start_time
        self.last_time = self.t
        self.rest_flag = True
        self.armed = False

        self.timer = self.create_timer(self.loop_latency, self.robot_main_loop)
        self.last_time_print = time.time()
        self.get_logger().info('Robot Run node has started.')
        
        self.feet_publisher = self.create_publisher(
            Float32MultiArray,
            'target_angle_msg',
            10)

        self.cmd = self.create_subscription(
            Twist,
            'cmd_vel',
            self.listener_cmd_callback,
            100)
        self.cmd = self.create_subscription(
            Twist,
            'com_state',
            self.listener_com_callback,
            100)
        self.status = self.create_subscription(
            Int8MultiArray,
            'controller_status',
            self.listener_status_callback,
            100)
        self.subscriptions  # prevent unused variable warning

    def listener_com_callback(self, msg):
        self.robot_player.body.position[0] = msg.linear.x
        self.robot_player.body.position[1] = msg.linear.y
        self.robot_player.body.position[2] = msg.linear.z
        self.robot_player.body.orientation[0] = msg.angular.x
        self.robot_player.body.orientation[1] = msg.angular.y
        self.robot_player.body.orientation[2] = msg.angular.z

        if not (self.cmd_received):
            self.cmd_received = True

    def listener_cmd_callback(self, msg):
        self.robot_player.body.linear_velocity = msg.linear.x
        self.robot_player.body.linear_angle = msg.linear.y
        self.robot_player.body.angular_velocity = msg.angular.z

        if not (self.cmd_received):
            self.cmd_received = True


    def listener_status_callback(self, msg):
        self.kill_flag = msg.data[0]
        self.rest_flag = msg.data[1]
        self.pose_mode = msg.data[2]
        self.compliant_mode = msg.data[3]
        if not (self.status_received):
            self.status_received = True

    def get_parameters(self) :
        self.declare_parameter('fieldnames', ['','','','', '','','', '','','', '','',''])
        self.declare_parameter('body_to_feet0', [0.0 , 0.0 , 0.0, 0.0 , 0.0 , 0.0, 0.0 , 0.0 , 0.0, 0.0 , 0.0 , 0.0])
        self.declare_parameter('body_to_feet_rest', [0.0 , 0.0 , 0.0, 0.0 , 0.0 , 0.0, 0.0 , 0.0 , 0.0, 0.0 , 0.0 , 0.0])
        self.declare_parameter('com_orientation0', [0.0 , 0.0 , 0.0])
        self.declare_parameter('com_position0', [0.0 , 0.0 , 0.0])
        self.declare_parameter('gait_offset0', [0.0 , 0.0 , 0.0])
        self.declare_parameter('step_period0', 0.0)
        self.declare_parameter('loop_latency', 0.0)


        return {
            'fieldnames': self.get_parameter('fieldnames').get_parameter_value().string_array_value,
            'body_to_feet0': self.get_parameter('body_to_feet0').get_parameter_value().double_array_value,
            'body_to_feet_rest': self.get_parameter('body_to_feet_rest').get_parameter_value().double_array_value,
            'com_orientation0': self.get_parameter('com_orientation0').get_parameter_value().double_array_value,
            'com_position0': self.get_parameter('com_position0').get_parameter_value().double_array_value,
            'gait_offset0': self.get_parameter('gait_offset0').get_parameter_value().double_array_value,
            'step_period0': self.get_parameter('step_period0').get_parameter_value().double_value,
            'loop_latency': self.get_parameter('loop_latency').get_parameter_value().double_value
        }   

    def publishAngleTopic(self):    
        fr_angles = np.array([0.0 , -0.785 , 1.57])
        fl_angles = np.array([0.0 , -0.785 , 1.57])
        br_angles = np.array([0.0 ,  0.785 , -1.57])
        bl_angles = np.array([0.0 ,  0.785 , -1.57])
        angles = np.concatenate((fr_angles, fl_angles, br_angles, bl_angles))

        #self.get_logger().info('angle type:' + str(type(self.robot_player.body.joint_angles)))
        for i in range(len(fr_angles)):
            angles[i] = self.robot_player.body.joint_angles[0,i]* 360.0 / (2 * 3.1415)
            angles[3+i] = self.robot_player.body.joint_angles[1,i]* 360.0 / (2 * 3.1415)
            angles[6+i] = self.robot_player.body.joint_angles[2,i]* 360.0 / (2 * 3.1415)
            angles[9+i] = self.robot_player.body.joint_angles[3,i]* 360.0 / (2 * 3.1415)

        msg = Float32MultiArray()
        msg.data = angles.astype(np.float32).tolist()
        self.feet_publisher.publish(msg)

    def clear(self):
        # for windows
        if name == 'nt':
            _ = system('cls')
        # for mac and linux(here, os.name is 'posix')
        else:
            _ = system('clear')

    def printInformation(self, latency):
        if (time.time() - self.last_time_print >= 0.1):
            self.last_time_print = time.time()
            self.clear()
            self.get_logger().info('')
            self.get_logger().info('robot run (Robot armed)____________fps: '+ str(1.0/latency))
            self.get_logger().info('')
            self.get_logger().info('fr: ' + str(self.robot_player.body.to_feet[0,:]))
                                   
    def robot_main_loop(self):
        if not (self.cmd_received and self.status_received):
            self.get_logger().info("Waiting for robot teleop input controller...")
            return
        if not (self.armed):
            self.armed = True
            
        loop_time = time.time() - self.last_time
        self.last_time = time.time()
        self.t = time.time() - self.start_time
        self.printInformation(loop_time)
        
        if (self.kill_flag):
            self.robot_player.updateKill()
        else:
            if (self.rest_flag):
                self.robot_player.updateRest()
            else:
                if (self.pose_mode):
                    self.robot_player.updateStaticControl()
                else:
                    self.robot_player.updateDynamicControl()

        self.publishAngleTopic()
        


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









