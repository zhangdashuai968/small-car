#!/bin/bash

echo "Starting complete ABOT navigation and grasp system..."

# Start all services in background
gnome-terminal --tab --title="Bringup" -- bash -c "roslaunch abot bringup.launch; exec bash" &
sleep 5
gnome-terminal --tab --title="Navigation" -- bash -c "roslaunch abot navigate.launch; exec bash" &
sleep 5
gnome-terminal --tab --title="VL_Locate" -- bash -c "conda activate 39 && roslaunch vl_locate vl_locate.launch; exec bash" &
sleep 5
gnome-terminal --tab --title="Grasp" -- bash -c "roslaunch ZachLab_grasp grasp.launch; exec bash" &
sleep 10

# Set initial servo position
echo "Setting initial servo position..."
rostopic pub -1 servo riki_msgs/Servo "Servo1: 90 
Servo2: 20"
sleep 2

echo "All services started. Running automation task..."
rosrun abot_grasp_race auto_navigation_grasp.py
