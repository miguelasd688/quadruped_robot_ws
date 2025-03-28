FROM ros:humble
ARG USERNAME=miguelasd
ARG USER_UID=1000
ARG USER_GID=$USER_UID

# Delete user if it exists in container (e.g Ubuntu Noble: ubuntu)
RUN if id -u $USER_UID ; then userdel `id -un $USER_UID` ; fi

# Create the user
RUN groupadd --gid $USER_GID $USERNAME \
    && useradd --uid $USER_UID --gid $USER_GID -m $USERNAME \
    #
    # [Optional] Add sudo support. Omit if you don't need to install software after connecting.
    && apt-get update \
    && apt-get install -y sudo \
    && echo $USERNAME ALL=\(root\) NOPASSWD:ALL > /etc/sudoers.d/$USERNAME \
    && chmod 0440 /etc/sudoers.d/$USERNAME
RUN apt-get update && apt-get upgrade -y
RUN apt-get install -y python3-pip
#ENV SHELL /bin/bash

# ********************************************************
# * Anything else you want to do like clean up goes here *
# ********************************************************

#RUN source /opt/ros/humble/setup.bash
WORKDIR quadruped_ws/
COPY . .
RUN ls
#RUN cd quadruped_ws/

# Build basic packages of the program
RUN sudo colcon build --packages-select quadruped_teleop 
#RUN source install/local_setup.bash

RUN apt-get update && rosdep update
RUN sudo rosdep install --from-paths src --ignore-src -y --rosdistro humble
RUN sudo apt-get install python3-pip
# Build micro-ROS tools and source them
#RUN sudo colcon build
#RUN source install/local_setup.bash

# Download micro-ROS agent packages
#RUN ros2 run micro_ros_setup create_agent_ws.sh
#RUN ros2 run micro_ros_setup build_agent.sh
#RUN source install/local_setup.bash
#RUN ros2 run micro_ros_agent micro_ros_agent serial --dev /dev/ttyACM0


# [Optional] Set the default user. Omit if you want to keep the default as root.
USER $USERNAME
CMD ["/bin/bash"]