# Description

This repo contains the code running on a quadruped robot.
Ececuted with Raspberry Pi 4, Ubuntu 24

It is implemented with ROS2

# From root workspace ROS 2 folder:

source /opt/ros/humble/setup.bash
colcon build
source install/local_setup.bash

# Packages:

* quadruped_robot: Where robot main loop is, it is:
  * ros2_node: _robot_run.py_
    Subscribed to Commands and RobotStatus. These are the inputs basically
    Publish joint angle to command it to the motors
  * states_manager: _robot_player.py
    It is a states manager, instanciates all the robot model variables
    state.py_ defines each state
  * movement controller: _move_controller.py_
    Implements some methods to allow secuences of actions on the robot.
* quadruped_teleop
  * ros2_node: _joystic_teleop.py_
    From _joystick.py_, it reads wireless gamepad
  * ros2_node: _keyboard_teleop.py_
    TODO: to control robot with any keyboard
* robot_interfaces
  * msg/Commasds.msg
  * msg/RobotStatus.msg
  * msg/JointAngles.msg
    custom topics to communicate between the nodes of the robot.

# Run robot_run node with parameters

```
ros2 run quadruped_robot robot_run --ros-args --params-file src/quadruped_robot/config/params.yaml
```

# Paring bluetooth controller with bluetoothctl:

```
sudo bluetoothctl

agent on
discoverable on
default-agent
scan on

connect CONTROLLER_MAC_ADDRESS
trust CONTROLLER_MAC_ADDRESS
quit
```