# MTRX3760-Project-1 🤖
## Group - class aMazing **

### **Group members ✍️:**  
- **Shiyao Lin**: 540745159
- **Michael Delano**: 530371788
- **Kaia Feng**: 530289769

## Member Responsibilities 👷‍♂️

| Name            | Responsibilities                      |  
|----------------|--------------------------------------|  
| Mike | Camera (refactored), Maze Setup, Maze Testing |  
| Kaia      | ROS Node Diagram, UML Diagram, ReadMe file |  
| Shiyao | Right Wall-Following Logic, Code Refactor |  

## Project Overview 📜
This repository is developed as part of MTRX3760 Project 1 at the University of Sydney.\
The project simulates a TurtleBot 3 robot navigating through two distinct maze environments, including an open maze and a closed maze, within RViz and Gazebo.\
When launched, the TurtleBot autonomously explores the maze using a right wall following navigation logic. Its goal is to reach the maze exit, visually marked by a green square box, while using simulated LiDAR and camera sensors for perception of surroundings.
### How to Download the Repo ⬇️
Clone the repository in your terminal using:
   ```bash
   git clone https://github.com/kaia-F/MTRX3760-Project-1
   ```

## Requirements
### Software Requirements
- **OS:**
  - Ubuntu 24.04 LTS (Noble), amd64 or arm64.
- **ROS 2 distribution:**
  - ROS 2 Jazzy Jalisco (desktop or ros-base profile). Install from deb packages on Ubuntu 24.04. RViz2 is included inside.
- **Gazebo Ignition:**
  - Gazebo Sim “Harmonic” (current recommended Gazebo for ROS 2). On Jazzy, Gazebo is provided via ROS vendor packages.
- **TurtleBot3 Simulation**
-   turtlebot3_gazebo, turtlebot3_msgs, turtlebot3_description packages for Jazzy.

### Installation & Setup ⚙️

### 1. TurtleBot3 PC Setup  
Follow the **PC Setup** steps from the official TurtleBot3 Quick Start Guide (Jazzy version):  
🔗 [TurtleBot3 Quick Start – PC Setup](https://emanual.robotis.com/docs/en/platform/turtlebot3/quick-start/#pc-setup)

This includes:
- Installing **ROS 2 Jazzy Jalisco** on Ubuntu 24.04.
- Setting up TurtleBot3 packages (`turtlebot3`, `turtlebot3_msgs`, `turtlebot3_gazebo`).
- Configuring the TurtleBot3 model environment variable (`burger`, `waffle`, or `waffle_pi`).

---

### 2. Launching Gazebo (Ignition / Gazebo Sim)
This simulation requires **ROS 2 Jazzy** and **Gazebo (Ignition/Harmonic)**.

#### **Terminal 1 — Start the Gazebo Simulation**

**Kill any leftover Gazebo processes:**
```bash
pkill -9 -f gazebo
pkill -9 -f gzserver
pkill -9 -f gzclient
```
**Set TurtleBot3 model:**
```bash
export TURTLEBOT3_MODEL=burger
```
**Go to workspace:**
```bash
cd ~/Documents/GitHub/MTRX3760-Project-1/turtlebot3_ws
```
**Build only turtlebot3_gazebo with override (in case of underlay conflict):**
```bash
colcon build --packages-select turtlebot3_gazebo --allow-overriding turtlebot3_gazebo
```
**Source workspace:**
```bash
source install/setup.bash
```
**Launch Gazebo with your custom launch file:**
```bash
ros2 launch turtlebot3_gazebo turtlebot3_dqn_stage1_custom.launch.py
```
**Terminal 2: Build and run the modified code  **\
**Go to workspace**
```bash
cd ~/Documents/GitHub/MTRX3760-Project-1/turtlebot3_ws
```
Run the drive node:
```bash
source install/setup.bash
ros2 run turtlebot3_gazebo turtlebot3_drive
```



## Testing and Troubleshooting 🧪
### Turtlebot robot setup testing
Confirm packages are built and discoverable
```bash
ros2 pkg prefix turtlebot3_description
ros2 pkg prefix turtlebot3_gazebo
ros2 pkg prefix turtlebot3_msgs
```
Check if robot has spawned:  \
In a new terminal, check for topics including /cmd_vel, /scan, /odom, /tf to confirm the robot is publishing control ready.
```bash
ros2 node list | grep -E 'gazebo|spawn|robot_state_publisher'
ros2 topic list | grep -E '/cmd_vel|/scan|/odom|/tf'
```
Publish a test velocity to make the robot roll forward for 3 seconds:
```bash
ros2 topic pub -r 10 /cmd_vel geometry_msgs/Twist '{linear: {x: 0.2}, angular: {z: 0.0}}' --qos-durability transient_local
```
Or manually control the robot with teleop:  \
In a new terminal, run ros2 teleop
```bash
export TURTLEBOT3_MODEL=burger
ros2 run turtlebot3_teleop teleop_keyboard
```

### Maze Testing




## Minutes and Meeting Notes 📁

All meeting notes and agendas are stored in the `/minutes/` directory of this repository.


