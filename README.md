# Description

This repo contains the code running on a quadruped robot.
Executed with Raspberry Pi 4, Ubuntu 20

It is implemented with ROS2

# Open de workspace

* [Install Docker and VSCode.](https://docs.ros.org/en/humble/How-To-Guides/Setup-ROS-2-with-VSCode-and-Docker-Container.html)
* 
* Use `View->Command Palette...` or `Ctrl+Shift+P` to open the command palette. Search for the command `Dev Containers: Reopen in Container` and execute it. This will build your development docker container for your. 

# From root workspace ROS 2 folder:

```
source /opt/ros/humble/setup.bash
colcon build
source install/local_setup.bash
```

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
* micro-ros w/ Teensy 4.1
  * https://micro.ros.org/docs/tutorials/core/teensy_with_arduino/


# Run robot_run node with parameters

```
ros2 run quadruped_robot robot_run --ros-args --params-file src/quadruped_robot/config/params.yaml
```

# Launch robot and teleop

* PS5 controller teleop:
```
ros2 launch quadruped_teleop ps5_controller.launch.py
```
* Robot node: 
```
ros2 launch quadruped_robot robot_launch.py
```


#### Connect agent to enable microros transports:

```
ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0
```

# Run Foxgloves server:

```
ros2 launch foxglove_bridge foxglove_bridge_launch.xml
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
