import os
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    config_file = os.path.join(
        get_package_share_directory('quadruped_teleop'),
        'config',
        'params.yaml'
    )

    declare_param_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=config_file,
        description='Path to the parameter file'
    )

    node = Node(
        package='quadruped_teleop',
        executable='joystick_teleop',
        name='joystick_teleop',
        parameters=[LaunchConfiguration('params_file')],
        #prefix=["sudo -E env \"PYTHONPATH=$PYTHONPATH\" \"LD_LIBRARY_PATH=$LD_LIBRARY_PATH\" \"PATH=$PATH\" \"USER=$USER\"  bash -c "],
        shell=True
    )

    return LaunchDescription([
        declare_param_file_arg,
        node
    ])