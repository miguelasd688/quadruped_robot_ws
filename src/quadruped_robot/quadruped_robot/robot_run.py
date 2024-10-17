import time
import csv
import sys
import numpy as np

import rclpy
from rclpy.node import Node
from robot_interfaces.msg import Commands
from robot_interfaces.msg import RobotStatus

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


    def listener_cmd_callback(self, msg):
        self.linear_velocity = msg.linear_velocity
        self.linear_angle = msg.linear_angle
        self.angular_velocity = msg.angular_velocity
        self.com_position = msg.com_position  
        self.com_orientation = msg.com_orientation 
        print()
        if not (self.cmd_received):
            self.cmd_received = True


    def listener_status_callback(self, msg):
        self.kill_flag = msg.kill_flag
        self.rest_flag = msg.rest_flag
        self.compliant_mode = msg.compliant_mode
        self.pose_mode = msg.pose_mode  
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
    
    



    def robotDesiredStates(self):
        return {
            'com_position': self.com_position,
            'com_orientation': self.com_orientation,
            'linear_velocity': self.linear_velocity,
            'linear_angle': self.linear_angle,
            'angular_velocity': self.angular_velocity,
        }


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
        self.robot_player.setRobotStateVariables(self.robotDesiredStates())
        
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








