import rclpy
from rclpy.node import Node


from quadruped_teleop.joystick import Joystick

from robot_interfaces.msg import Commands
from robot_interfaces.msg import RobotStatus




class PS5Controller(Node):

    def __init__(self):
        super().__init__('joystick_teleop')
        self.cmd_publisher = self.create_publisher(Commands, 'controller_command', 10)
        self.status_publisher = self.create_publisher(RobotStatus, 'controller_status', 10)
        self.timer = self.create_timer(0.02, self.read_joystick)

        self.declare_parameter('device_events', '')
        event_path = self.get_parameter('device_events').get_parameter_value().string_value
        self.joystick = Joystick()
        self.joystick.conect(event_path)


    def read_joystick(self):
        # Call read() from joystick
        bodyAngle, com_pos, com_orn, V, angle, w_rot, T, compliant_mode, kill, rest, pose_mode = self.joystick.read()

        # publish to topic /cmd_vel
        cmd = Commands()
        cmd.linear_velocity = V
        cmd.linear_angle = angle
        cmd.angular_velocity = w_rot
        cmd.com_position = com_pos
        cmd.com_orientation = com_orn

        status = RobotStatus()
        status.rest_flag = rest
        status.kill_flag = kill
        status.pose_mode = pose_mode
        status.compliant_mode = compliant_mode

        self.cmd_publisher.publish(cmd)
        self.status_publisher.publish(status)



def main(args=None):
    rclpy.init(args=args)
    joystick_teleop = PS5Controller()
    rclpy.spin(joystick_teleop)
    joystick_teleop.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()