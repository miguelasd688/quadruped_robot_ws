# From root workspace ROS 2 folder:
    source /opt/ros/humble/setup.bash
    colcon build
    source install/local_setup.bash

# Run robot_run node with parameters
    ros2 run quadruped_robot robot_run --ros-args --params-file src/quadruped_robot/config/params.yaml

# Paring bluetooth controller with bluetoothctl:
    sudo bluetoothctl

    agent on
    discoverable on
    default-agent
    scan on

    connect CONTROLLER_MAC_ADDRESS
    trust CONTROLLER_MAC_ADDRESS
    quit
