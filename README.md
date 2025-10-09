# MTRX3760 Project 1 Project Overview
This project simulate a Turtlebot 3 in RViz and Gazebo, including simulated laser and camera in 2 different maze setting layouts (an opened maze and a closed maze). When launched, turtlebot autonomously navigate through one of the mazes, using the right wall following logic, until it reaches the maze exit goal as indicated by a green square box.

# Launch Gazebo and drive node
This interface environment requires installation of ROS2 jazzy and Gazebo ignition.\
In Terminal 1, Launch Gazebo:  \
Kill any leftover Gazebo processes
```bash
pkill -9 -f gazebo
pkill -9 -f gzserver
pkill -9 -f gzclient
```
Set TurtleBot3 model:
```bash
export TURTLEBOT3_MODEL=burger
```
Go to workspace:
```bash
cd ~/Documents/GitHub/MTRX3760-Project-1/turtlebot3_ws
```
Build only turtlebot3_gazebo with override (in case of underlay conflict):
```bash
colcon build --packages-select turtlebot3_gazebo --allow-overriding turtlebot3_gazebo
```
Source workspace:
```bash
source install/setup.bash
```
Launch Gazebo with your custom launch file:
```bash
ros2 launch turtlebot3_gazebo turtlebot3_dqn_stage1_custom.launch.py
```
Terminal 2: Build and run the modified code  \
Go to workspace
```bash
cd ~/Documents/GitHub/MTRX3760-Project-1/turtlebot3_ws
```
Make sure workspace is sourced:
```bash
source install/setup.bash
```
Run the drive node:
```bash
ros2 run turtlebot3_gazebo turtlebot3_drive
```
# Turtlebot setup Testing

# Maze Testing
