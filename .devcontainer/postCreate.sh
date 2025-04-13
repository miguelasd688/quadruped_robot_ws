#!/bin/bash
mkdir -p src
sudo rosdep update
sudo rosdep install --from-paths /home/ws/src --ignore-src -y
sudo chown -R $(whoami) /home/ws/
colcon build --packages-select quadruped_teleop --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
sudo chmod 777 /dev/input/event*
#sudo touch /etc/udev/rules.d/99-input.rules
#sudo echo 'KERNEL=="event*", NAME="input/%k", MODE="660", GROUP="sgx"' >> /etc/udev/rules.d/99-input.rules