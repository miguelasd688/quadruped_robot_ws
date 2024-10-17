import os
from ament_index_python.packages import get_package_share_directory
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    config_file = os.path.join(
        get_package_share_directory('quadruped_robot'),
        'config',
        'params.yaml'
    )

    declare_param_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=config_file,
        description='Path to the parameter file'
    )

    node = Node(
        package='quadruped_robot',
        executable='robot_run',
        name='robot_run',
        #prefix=["sudo -E env \"PYTHONPATH=$PYTHONPATH\" \"LD_LIBRARY_PATH=$LD_LIBRARY_PATH\" \"PATH=$PATH\" \"USER=$USER\"  bash -c "],
        parameters=[LaunchConfiguration('params_file')],
        shell=True
    )

    return LaunchDescription([
        declare_param_file_arg,
        node
    ])