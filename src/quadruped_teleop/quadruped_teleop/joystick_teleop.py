import rclpy
from rclpy.node import Node

import time
from sys import exit
from os import system, name

from . import joystick as joy

from robot_interfaces.msg import Commands
from robot_interfaces.msg import RobotStatus




class PS5Controller(Node):

    def __init__(self):
        super().__init__('joystick_teleop')
        self.cmd_publisher = self.create_publisher(Commands, 'controller_command', 10)
        self.status_publisher = self.create_publisher(RobotStatus, 'controller_status', 10)
        self.timer = self.create_timer(0.01, self.read_joystick)
        
        self.declare_parameter('device_events', '')
        event_path = self.get_parameter('device_events').get_parameter_value().string_value
        self.inputs = joy.RobotInputs()
        self.raw = joy.RawValues()
        self.connectController(event_path)

        self.last_time = time.time()
        self.last_time_print = self.last_time

    def connectController(self, event_path):
        self.get_logger().info(f"Trying to conect to: {event_path}")
        self.joystick = joy.Joystick(self.inputs, self.raw)
        if (self.joystick.conect(event_path)):
            self.get_logger().info(f"Joystick controller connected")
        else:
            self.get_logger().error('Cannot hear from' + event_path)
            self.get_logger().warning('Try: python3 /usr/local/lib/python3.10/dist-packages/evdev/evtest.py')
            exit()


    def publishCommands(self):
        cmd = Commands()
        cmd.linear_velocity = self.inputs.linear_velocity
        cmd.linear_angle = self.inputs.linear_angle
        cmd.angular_velocity = self.inputs.angular_velocity
        cmd.com_position = self.inputs.com_pos
        cmd.com_orientation = self.inputs.com_orn
        self.cmd_publisher.publish(cmd)


    def publishStatus(self):
        status = RobotStatus()
        status.rest_flag = self.inputs.rest_mode
        status.kill_flag = self.inputs.kill
        status.pose_mode = self.inputs.pose_mode
        status.compliant_mode = self.inputs.compliant_mode
        self.status_publisher.publish(status)


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
            self.get_logger().info('JoyStick______________________ fps: ' + str(round(1./latency,1)))
            self.get_logger().info('')
            self.get_logger().info(f'    · R_JOY_X: {int(self.raw_values.r_joy[0],)}, R_JOY_Y: {int(self.raw_values.r_joy[1])}, L_POT: {int(self.raw_values.L2)}')
            self.get_logger().info(f'    · L_JOY_X: {int(self.raw_values.l_joy[0])}, L_JOY_Y: {int(self.raw_values.l_joy[1])}, L_POT: {int(self.raw_values.R2)}')
            self.get_logger().info('')
            self.get_logger().info(f'    · pose_mode: {self.inputs.pose_mode}, rest_flag: {self.inputs.rest_mode}, kill_flag:{self.inputs.kill}')
            self.get_logger().info('  Body input:')
            self.get_logger().info(f'    · CoM_pos: {self.inputs.com_pos}, CoM_orn: {self.inputs.com_orn}')
            self.get_logger().info('  Step input:')
            self.get_logger().info(f'    · Vel: {round(self.inputs.linear_velocity,3)} m/sec, angle: {round(self.inputs.linear_angle,3)} deg, Wrot: {round(self.inputs.angular_velocity)}rad/sec')



    def read_joystick(self):
        time_now = time.time()
        latency = time_now - self.last_time
        self.last_time = time_now
        # Call read() from joystick
        #com_pos, com_orn, linear_velocity, linear_angle, w_rot, T, compliant_mode, kill, rest, pose_mode
        self.inputs , self.raw_values = self.joystick.read()
        # publish to topic /cmd_vel
        self.publishCommands()
        self.publishStatus()

        self.printInformation(latency)


def main(args=None):
    rclpy.init(args=args)
    joystick_teleop = PS5Controller()
    rclpy.spin(joystick_teleop)
    joystick_teleop.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()