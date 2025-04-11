#!/bin/bash
#sudo apt install npm

#sudo npm install -g @devcontainers/cli

#devcontainer up --workspace-folder . 

#devcontainer exec --workspace-folder . /bin/bash

#devcontainer build --workspace-folder . --image-name quadruped-devcontainer
devcontainer build --image-name quadruped-devcontainer --log-level debug --workspace-folder .  

sudo docker run -t quadrupedcontainer --name containerrrrrr --network host \
    --mount /tmp/.X11-unix,target=/tmp/.X11-unix,type=bind,consistency=cached \
    /dev/dri,target=/dev/dri,type=bind,consistency=cached \
    ${localEnv:HOME}/.ssh,target=/home/${localEnv:USER}/.ssh,type=bind quadruped-devcontainer
