#!/bin/bash

DIRNAME=$(dirname "$0")
# $DIRNAME
gnome-terminal --tab --title="docker_up" -e "bash -c \"docker-compose -f docker-compose.yml up; exec bash\"" --tab -e "bash -c \"sleep 5; sudo xhost +local:root; docker exec -it --privileged strirus_gait_planning_ros-master_1 bash; exec bash\"" --tab -e "bash -c \"git status; exec bash\" ";
clear
