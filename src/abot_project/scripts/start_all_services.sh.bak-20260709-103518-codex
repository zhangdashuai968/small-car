#!/bin/bash

# One-click startup script for abot navigation and grasp system

echo "Starting ABOT Navigation and Grasp System..."

# Start bringup in background
echo "Starting bringup..."
gnome-terminal --tab --title="Bringup" -- bash -c "roslaunch abot bringup.launch; exec bash"
sleep 5

# Start navigation in background  
echo "Starting navigation..."
gnome-terminal --tab --title="Navigation" -- bash -c "roslaunch abot navigate.launch; exec bash"
sleep 5

# Start VL locate with conda environment
echo "Starting VL locate..."
gnome-terminal --tab --title="VL_Locate" -- bash -c "conda activate 39 && roslaunch vl_locate vl_locate.launch; exec bash"
sleep 5

# Start grasp system
echo "Starting grasp system..."
gnome-terminal --tab --title="Grasp" -- bash -c "roslaunch ZachLab_grasp grasp.launch; exec bash"
sleep 5

echo "All services started. You can now run the automation script:"
echo "rosrun abot_project auto_navigation_grasp.py"