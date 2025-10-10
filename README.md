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
The project simulates a TurtleBot 3 robot navigating through two distinct maze environments—an open maze and a closed maze—within RViz and Gazebo.\
When launched, the TurtleBot autonomously explores the maze using a right wall following navigation logic. Its goal is to reach the maze exit, visually marked by a green square box, while using simulated LiDAR and camera sensors for perception of surroundings.\
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

## Installation & Setup
### TurtleBot3 PC Setup
Completed the “PC setup” steps of the turtlebot quick-start guide at https://emanual.robotis.com/docs/en/platform/turtlebot3/quick-start/#pc-setup, under Jazzy version.  \
Install TurtleBot3 Simulation Packages by following the guide at https://emanual.robotis.com/docs/en/platform/turtlebot3/quick-start/#pc-setup. \

### Launch Gazebo
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




<p align="center">
  <img src="images/backdrop.png" alt="MTRX2700 Assignment 2 Banner" width="100%"/>
</p>

# MTRX2700-Assignment-2 🤖
## Group Number - NAME **

### **Authors ✍️:**  
- **Amelia Chan**: 530839244  
- **Dylan George**: 530839244  
- **Shiyao Lin**: 540745159

## Roles and Responsibilities 👷‍♂️

| Name            | Role                  | Responsibilities                      |  
|----------------|----------------------|--------------------------------------|  
| Amelia  | Engineer    | Digital IO, Intergation Parsing |  
| Dylan      | Engineer   | Serial Communication, Integration |  
| Shiyao | Engineer    | Hardware Timers, Integration Documentation |  

## Project Overview 📜
This repository was made to meet requirements for the ASM Lab (Assignment 2) as part of coursework in MTRX2700 (Mechatronics 2) at the **University of Sydney**. The code is mostly in the **C** coding language and was written for, and tested on, **STM32F3 Discovery** micro-controllers.  

The code is broken into separate modules, with sub-tasks for each module, each entailing their own respective files. Descriptions of each task, and how to run the code are explained in the drop-downs in the **Module Overview** section.  

It is recommended to create a new project in **STM32CubeIDE** using the downloaded Git repository (see details below). You can then run each sub-project by navigating into its respective directory or by manually copying individual files into a new project.

### How to Download the Repo ⬇️
Clone the repository in your terminal using:
   ```bash
   git clone https://github.com/dylangeorg3/MTRX2700-Assignment-2.git
   ```
Note that the repository is not publically available, for reasons pertaining to academic integrity.
## Module Overview 📂

## Requirements
### Software Requirements
- **STM32CubeIDE** (Required for building, debugging, and uploading code to the STM32F3Discovery board).
- **Serial Monitor Tool**
  - **Windows:** [PuTTY](https://www.putty.org/)
  - **MacOS:** [CuteCom](https://cutecom.sourceforge.io/)

### Hardware Requirements
- **One or Two STM32F3 Discovery Boards**
  - Two boards are required for **Exercise 3E** and **Integration**, connected via serial communication.
- **3 x 30AWG female to female jumper cables**
  - Required to connect the two boards in the aforementioned tasks
  - They are relatively cheap, and available at [Core Electronics](https://core-electronics.com.au/female-to-female-dupont-line-40-pin-10cm-24awg.html?gad_source=1&gclid=Cj0KCQjw4v6-BhDuARIsALprm32sz4oCAe0GOuz8QdB3mVvDaUouCLKruWdyOYjAz_SCJl4C5ngxbRAaAuPGEALw_wcB)


## Installation & Setup
### STM32CubeIDE Installation
1. Download STM32CubeIDE from [ST’s official website](https://www.st.com/en/development-tools/stm32cubeide.html).
2. Follow the installation instructions provided on the website for your operating system.

### Serial Monitor Installation
- **Windows (PuTTY):**
  1. Download from [PuTTY’s official website](https://www.putty.org/).
  2. Install using the provided installer.
- **MacOS (CuteCom):**
  1. Install via Homebrew (Recommended):
     ```bash
     brew install cutecom
     ```
  2. Or download from the [CuteCom website](https://cutecom.sourceforge.io/).


<details>
<summary><strong>Exercise 1 - Digital I/O 💡</strong></summary>

#### **Description**
This module provides an encapsulated interface for managing a digital button input and LED output. The interface includes callback-based interrupt handling for the button and LED control using hardware timers. The LED state is encapsulated and can only be accessed through dedicated get and set functions exposed via the header file, ensuring modularity and abstraction.

Key features:

Initialization of button and LED pins.

Function pointer callback triggered on button press (via EXTI interrupt).

Encapsulated LED state accessible through get_led_state() and set_led_state().

Timer-based LED update rate limiting to prevent rapid toggling.

#### **Usage**
This module can be used in a variety of applications to handle interrupts. This specific module uses the user button PA0 as the interrupt trigger and the function chase_led as the interrupt request handler. However, this can easily be extended to various applications by changing the interrupt trigger or changing the callback function. To view this function, the user must press the button to change the led. 

### **Testing**

Button Interrupt Test

Verified that the callback function is triggered exactly once on each valid button press using edge-triggered EXTI.

Tested with hardware debounce (if applicable) or confirmed via signal trace.

LED State Management Test: Called set_led_state(1) and set_led_state(0) and verified LED output matches expected behavior. 
Verified that get_led_state() returns accurate and updated status.

Rate Limiting Test : Attempted to toggle the LED rapidly via button input, and verified that LED state changes were restricted to the timer-controlled rate limit (e.g., 200ms minimum delay). ALso confirmed that set_led_state() is non-blocking and returns immediately.

Integration Test: Confirmed that the module runs without interfering with other tasks in the main loop.


</details>

---

<details>
<summary><strong>Exercise 2 - Serial Interface 📡</strong></summary>

#### **Description**
This module implements a modular, interrupt-driven UART communication interface for the STM32F3Discovery board. It supports multiple input modes (static, dynamic, and double-buffered), callback functions for RX/TX completion, and non-blocking background processing of received messages.

Key features:

- **USART1 Setup**: Fully initialised with custom baud rate configuration and alternate function GPIO setup.
- **Static Input Mode**: Reads into a fixed buffer up to `BUFFER_SIZE`, terminates on newline.
- **Dynamic Input Mode**: Uses `malloc` and `realloc` to support arbitrarily long inputs.
- **Double Buffer Mode**: Uses two alternating buffers and interrupt-based reception to allow simultaneous input and processing.
- **Callback Functions**: RX and TX complete functions can be set during `SerialInitialise`.
- **Interrupt-Driven TX/RX**: TXE and RXNE handlers enable non-blocking, efficient I/O.
- **Buffer Swap and Processing**: Automatically swaps RX buffers once a message is complete, and processes the string in the background (e.g., controlling LEDs based on message length).

Modularity is emphasized through clear separation of:
- Peripheral configuration (`initialise.c`)
- Serial functionality (`serial.c/.h`)
- Demo/control flow (`serial_demo.c/.h`)
- Application entry point (`main.c`)



#### **Usage**
1. Connect the STM32F3Discovery board to your computer via USB.
2. Open the project in STM32CubeIDE and ensure all files related to Exercise 2 are included.
3. Navigate to `serial_demo.c` and uncomment the mode you'd like to test:
   - `SerialStaticMode();` — basic blocking input using a fixed buffer
   - `SerialDynamicMode();` — dynamic memory allocation with long input support
   - `DoubleBufferMode();` — full-duplex input/output with interrupt-based RX and TX
4. Debug and run the project.
5. Open a serial monitor (e.g., PuTTY or CuteCom) at **115200 baud**.
6. Type and send a message, you should see it echoed back over the serial interface

### **Testing**
**Static Mode Test**
- Sent input strings under 64 bytes via serial monitor and verified correct echo and LED behavior.
- Confirmed that messages terminate correctly on `\n` and buffer overflow is avoided.

**Dynamic Mode Test**
- Sent input strings over 100+ characters to confirm dynamic reallocation.
- Verified that echo and RX callback still function correctly with longer input.

**Double Buffer Mode Test**
- Sent input strings while observing LED response for message length.
- Confirmed real-time responsiveness, non-blocking processing, and seamless RX/TX.
- Verified that processed buffer contents match transmitted content.

**Interrupt Handling Test**
- Verified that both TXE and RXNE interrupts were handled correctly.
- Confirmed the system could receive and echo multiple messages back-to-back without crashing.

**Failure Cases Tested**
- Buffer overflow: ensured the string was safely terminated and old data was cleared.
- Reentrant access: confirmed proper separation between `active_buffer` and `processing_buffer`.

</details>

---

<details>
<summary><strong>Exercise 3 - Timer Interface ⏳</strong></summary>

#### **Description**
This module provides an encapsulated interface for a timer software module that can trigger a callback function at regular intervals, or trigger a one-shot event. The period of the timer and the callback function can be defined by the user. Get/set functions are implemented to allow outside modules to change the period.

#### **Usage**
1. Connect the STM32F3Discovery board to your computer via USB.
2. Open the project in STM32CubeIDE and ensure all files related to Exercise 3 are included.
3. Navigate to `main.c` and uncomment the mode you'd like to test:
   The function inputs are:
     1. desired timer interval in ms
     2. callback function
  
    // --- Choose (uncomment) ONE of the following ---
    // Option 1: Regular timer (e.g. blinking every 1000ms)
    //regular_timer(1000, &chase_led);

    // Option 2: One-shot timer (e.g. delay once for 5000ms)
    //oneshot_timer(5000, &chase_led);

5. Run the project.
6. Watch the leds flash at the given time period on your STM32 board.

### **Testing**
Different time periods were tested and the time interval was measured with a stopwatch to confirm accurate timing.

</details>

---

<details>
<summary><strong>Exercise 4 - Integration 🔄</strong></summary>

#### **Description**
This module combines all previously developed modules — UART serial communication, digital I/O, and timer control — into a single integrated application. It uses a custom command parser to interpret strings sent over the serial port and dispatch appropriate functions based on the command.

The system continuously listens for serial input using a double-buffered, interrupt-driven UART handler. Upon receiving a complete message, it parses the command and executes one of the following operations:

- **`led <bitmask>`** — Sets the state of GPIOE LEDs based on the binary string operand (e.g., `led 10101010`).
- **`serial <message>`** — Echoes the operand string back through the serial port.
- **`oneshot <delay_ms>`** — Starts a one-shot timer that executes a callback (e.g., toggles LEDs) once after the specified delay.
- **`timer <period_ms>`** — Starts a continuous timer that calls a callback (e.g., a "chase" LED pattern) at the specified period.
- **`stop`** — Stops the continuous timer (TIM2).

All functionality is modularised and integrated non-invasively, preserving software design principles such as encapsulation, reusability, and interrupt-driven architecture.

#### **Usage**
1. Connect your STM32F3Discovery board via USB and open a serial terminal at **115200 baud**.
2. Flash the integrated firmware using STM32CubeIDE.
3. In your serial terminal (e.g., PuTTY or CuteCom), send one of the following commands:
   - `led 10101010` — Sets GPIOE LEDs in the specified binary pattern.
   - `serial Hello World!` — Sends back "Hello World!" to your terminal.
   - `oneshot 1000` — Turns on a one-shot timer to execute a callback after 1000 ms.
   - `timer 500` — Starts a chase LED animation every 500 ms.
   - `stop` — Stops the periodic chase LED timer.

4. Observe LED behavior and echoed responses to validate execution.

#### **Testing**

**Command Parser Test**
- Verified that all command types (`led`, `serial`, `oneshot`, `timer`, `stop`) are correctly parsed and routed.
- Ensured fallback message `"Unknown command"` is displayed for invalid commands.

**LED Output Test**
- Tested various bitmasks (e.g., `led 11111111`, `led 00000000`) and confirmed correct GPIOE output.
- Confirmed internal LED state is preserved with `set_led_state()` and `get_led_state()`.

**Serial Echo Test**
- Sent multiple `serial <text>` messages and confirmed clean round-trip transmission through `SerialInterruptOutputString()`.

**One-Shot Timer Test**
- Sent `oneshot 2000` and verified callback fired once after 2 seconds.
- Confirmed that timer stopped automatically after execution.

**Continuous Timer Test**
- Sent `timer 1000` and verified chase LED rotated at 1-second intervals.
- Adjusted delay with new values (e.g., `timer 500`, `timer 250`) to test dynamic reconfiguration.
- Sent `stop` to verify timer halt and interrupt disablement.

**Interrupt Handling**
- Verified seamless concurrent UART RX/TX and TIM2 ISR execution.
- Ensured no race conditions when swapping UART buffers while a timer is running.

**Failure Modes Tested**
- Sent invalid input (e.g., `led banana`) and confirmed fallback handling.
- Flooded input rapidly to test buffer boundaries and interrupt resilience.
- Sent overlapping valid commands to confirm UART ISR handled state changes gracefully.



</details>

---




## Testing and Troubleshooting 🧪
### Project Setup ⚙️
1. Move all files from each folder in this project into your STM32CubeIDE workspace.
2. Open STM32CubeIDE and import the project files as necessary.
3. Build the project and ensure there are no errors.


### Debugging 🐞
- To debug your program in STM32CubeIDE:
  1. Click on **Debug** to start the debugging process.
  2. Once debugging starts, click **Resume** to run the program.
  3. Transmit your data via serial communication using your chosen serial monitor tool.
  4. Click **Pause** in STM32CubeIDE to inspect what was received.


- Make sure all dependencies are installed and properly configured.
- If the IDE does not detect the STM32F3Discovery board, try restarting your IDE or reconnecting the board.
- If using a MAC, the dongle connectors can be temperamental. Try unplugging and retrying
- Ensure serial connections are properly established between the boards if required

  
---

## Minutes and Meeting Notes 📁

All meeting notes and agendas are stored in the `/minutes/` directory of this repository.


