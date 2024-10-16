from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='quadruped_teleop',
            executable='joystick_teleop',
            name='joystick_teleop',
            #prefix=["sudo -E env \"PYTHONPATH=$PYTHONPATH\" \"LD_LIBRARY_PATH=$LD_LIBRARY_PATH\" \"PATH=$PATH\" \"USER=$USER\"  bash -c "],
            shell=True
        )
        #Node(
        #    package='quadruped_robot',
        #    executable='sensor_node',
        #    name='sensor_node'
        #),
    ])