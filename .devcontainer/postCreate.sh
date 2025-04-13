#!/bin/bash
git clone -b $ROS_DISTRO https://github.com/micro-ROS/micro_ros_setup.git src/micro_ros_setup
mkdir -p src
sudo apt update && rosdep update
sudo rosdep install --from-paths /home/ws/src --ignore-src -y
sudo chown -R $(whoami) /home/ws/

colcon build --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
#colcon build --packages-select quadruped_teleop --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo -DCMAKE_EXPORT_COMPILE_COMMANDS=ON

sudo chmod 777 /dev/input/event*
#sudo touch /etc/udev/rules.d/99-input.rules
#sudo echo 'KERNEL=="event*", NAME="input/%k", MODE="660", GROUP="sgx"' >> /etc/udev/rules.d/99-input.rules

source /home/ws/install/setup.bash
ros2 run micro_ros_setup create_agent_ws.sh
ros2 run micro_ros_setup build_agent.sh
source /home/ws/install/setup.bash

#ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0