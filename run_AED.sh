#!/bin/bash

RED='\033[0;31m'
GREEN='\033[0;32m'
NC='\033[0m' # No Color

team=$1
team=${team,,}

if [[ "$#" -ne 1 ]]; then
echo -e "${RED}Usage: $0 [control|perception|localization|planning]${NC}" >&2
exit 1
fi

if [[ $team = "all" ]]; then
gnome-terminal --tab -e " rosrun path_planner_v2 path_planner_node_v2" &
echo -e "${GREEN}Plannig Activated${NC}"
sleep 2
gnome-terminal --tab -e " roslaunch usb_cam usb_cam-test.launch " &
sleep 2
gnome-terminal --tab -e " roslaunch yolov5_ros yolov5.launch " &
sleep 2
gnome-terminal --tab -e " roslaunch rplidar_ros view_rplidar_a3.launch " &
echo -e "${GREEN}Perception Activated${NC}"
sleep 2
gnome-terminal --tab -e " rosrun main_control control " &
echo -e "${GREEN}Control Activated${NC}"
sleep 3
exit 0
fi

if [[ $team = "control" ]]; then
gnome-terminal --tab -e " rosrun main_control control " &
echo -e "${GREEN}Control Activated${NC}"
elif [[ $team = "perception" ]]; then
gnome-terminal --tab -e " roslaunch usb_cam usb_cam-test.launch " &
sleep 2
gnome-terminal --tab -e " roslaunch yolov5_ros yolov5.launch " &
sleep 2
gnome-terminal --tab -e " roslaunch rplidar_ros view_rplidar_a3.launch " &
echo -e "${GREEN}Perception Activated${NC}"
elif [[ $team = "localization" ]]; then
gnome-terminal --tab -e " roslaunch gps_imu_ekf ekf.launch " &
echo -e "${GREEN}Localization Activated${NC}"
elif [[ $team = "planning" ]]; then
gnome-terminal --tab -e " rosrun path_planner_v2 path_planner_node_v2" &
echo -e "${GREEN}Plannig Activated${NC}"
else
echo -e "${RED}Please check team name${NC}"
echo -e "${RED}Usage: $0 [control|perception|localization|planning]${NC}" >&2
fi
