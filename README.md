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

## File Descriptions

### Source files overview (turtlebot3_ws/src/turtlebot3_simulations/turtlebot3_gazebo/src)
- **turtlebot3_drive_node.cpp** — ROS 2 executable node that wires up publishers/subscribers (Lidar, cmd_vel, odom) and runs the main control loop for driving.
- **turtlebot3_drive.cpp** — Core driving logic (velocity commands, simple obstacle checks, state updates) used by the node.
- **wall_follower_state_machine.cpp** — Right/left-wall following finite-state machine (follow/turn/realign, gap/corner handling).
- **wall_follower_config.cpp** — Tunable parameters for wall following (target distance, PID gains, angular/linear limits).
- **utilities.cpp** — Small helpers (angle wrapping, range filtering, scan windowing, safety clamping, timing).
### Headers files overview (turtlebot3_ws/src/turtlebot3_simulations/turtlebot3_gazebo/include/turtlebot3_gazebo)
- **turtlebot3_drive.hpp** — Declares the Turtlebot3Drive class (ROS 2 node interface): publishers/subscribers, parameter loading, and main control callbacks.
- **wall_follower_state_machine.hpp** — Finite-state machine API for wall following (state enum, transition logic, update()/reset() signatures).
- **wall_follower_config.hpp** — Parameter struct for wall-following behavior (target distance, PID gains, speed limits) plus helpers to read from ROS 2 parameters.
- **utilities.hpp** — small helper functions (angle wrapping, clamping, scan windowing/filtering, conversions) shared across components.
- **sensor_data.hpp** — Normalized sensor bundle (LaserScan slices, min/avg ranges per sector, bumper/Cliff IR flags if used, odom snapshot); helpers to derive obstacles, gaps, and wall distance from raw topics
- **robot_state.hpp**- Tracks the robott class of the robot’s current state (pose, twist, heading, goal flags, last command, timers); data + small utility methods to update or integrate state
### Launch files overview (turtlebot3_ws/src/turtlebot3_simulations/turtlebot3_gazebo/launch)
- **openmaze.launch.py** — Spawns TurtleBot3 in the open maze world and starts the core nodes (Gazebo sim, robot_state_publisher, and drive node) for the wall-following logic in a designed open maze layout.
- **closedmaze.launch.py** — Spawns TurtleBot3 in the closed maze world and brings up the same core stack, focusing on tighter corridors and dead-ends to demonstrate turning logic, gap detection, and recovery behaviors.
- **turtlebot3_dqn_stage1_custom.launch.py** — Fast-start launch for testing robot logic (FSM, parameter tuning, logging); loads the robot inside a square map and run the drive node to test parameters.

## File Setup
### Folder layout
```bash
turtlebot3_gazebo/
├─ include/
│  └─ turtlebot3_gazebo/
│     ├─ turtlebot3_drive.hpp
│     ├─ wall_follower_state_machine.hpp
│     ├─ wall_follower_config.hpp
│     ├─ utilities.hpp
│     ├─ robot_state.hpp
│     └─ sensor_data.hpp
├─ src/
│  ├─ turtlebot3_drive_node.cpp     # main() entrypoint (ROS 2 node)
│  ├─ turtlebot3_drive.cpp          # implementation for turtlebot3_drive.hpp
│  ├─ wall_follower_state_machine.cpp
│  ├─ wall_follower_config.cpp
│  ├─ utilities.cpp
│  ├─ robot_state.cpp
│  └─ sensor_data.cpp
├─ launch/
│  ├─ openmaze.launch.py
│  ├─ closedmaze.launch.py
│  └─ turtlebot3_dqn_stage1_custom.launch.py
├─ CMakeLists.txt
└─ package.xml
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


