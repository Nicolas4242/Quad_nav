# Fennec Quadruped Robot Control System

This repository contains the electrical and low-level control stack for a quadruped robot[cite: 1]. It handles real-time actuator control on an Arduino Mega, trajectory validation and motor benchmarking using MATLAB, and serial communication with an onboard Raspberry Pi[cite: 1, 2, 6, 9].

---

## Hardware Architecture

The control architecture is split across two main processing units:

* **Microcontroller:** An Arduino Mega 2560 fitted with a ROBOTIS DYNAMIXEL Shield runs the primary 100 Hz control loop, real-time inverse/forward kinematics, and motor bus communications[cite: 1].
* **Companion Computer:** A Raspberry Pi sends operational commands and monitors telemetry via serial communication (UART2: TX2 on Pin 16, RX2 on Pin 17)[cite: 1].
* **Actuators:**
  * **8 × ROBOTIS Dynamixel XM430-W350-R:** Drive thigh and crank joints for all four legs using Protocol 2.0 at 1 Mbps[cite: 1].
  * **4 × ROBOTIS Dynamixel AX-12A:** Drive Z-axis abduction/adduction alignment using Protocol 1.0 at 1 Mbps[cite: 1].

---

## Repository Structure

| File / Component | Description |
| :--- | :--- |
| `arduino_uart2_quad_standard_gait.ino` | Main Arduino firmware implementing 100 Hz cycloid gait generation, kinematics, motor safety limits, and UART command parsing[cite: 1]. |
| `read_serial.txt` | Python script for the Raspberry Pi establishing threaded bidirectional UART communication (`/dev/ttyUSB0` at 115200 baud)[cite: 9]. |
| `fennec_foot_tracking.m` | MATLAB utility for tracking and extracting front-paw coordinate points from reference gait video footage[cite: 4]. |
| `fennec_complete_cycle.m` | MATLAB script that interpolates, smooths, and completes the full stance/swing foot trajectory cycle[cite: 3]. |
| `fennec_foot_points.csv` | Reference coordinates extracted from tracked foot-tip positions[cite: 3, 4]. |
| `ax12a_*.m` | MATLAB scripts for characterization, speed/position testing, and closed-loop position correction of AX-12A servos[cite: 2, 5]. |
| `xm430_*.m` | MATLAB benchmark scripts analyzing XM430-W350-R velocity and position profiles (step, rectangular, and trapezoidal)[cite: 6, 7, 8]. |

---

## Low-Level Serial Protocol

The Arduino listens on UART2 at 115200 baud for newline-terminated ASCII commands[cite: 1]:

* `WALK` / `W` / `START` — Transitions into the cyclic walking gait[cite: 1].
* `STAND` / `S` — Executes a smooth ramp into the neutral standing configuration[cite: 1].
* `TOGGLE` / `T` / `SPACE` — Toggles between walking and standing modes[cite: 1].
* `STATUS` / `PING` / `SCAN` — Requests telemetry including current state, phase index, target foot coordinates, and raw motor angles[cite: 1].
* `QUIT` / `Q` / `STOP` / `DISABLE` — Disables motor torque across all IDs and halts the control loop[cite: 1].

---

## ROS Integration

> **Note:** This section is reserved for the high-level ROS workspace and package documentation once implemented.

### Architecture Overview
<!--
Briefly explain the high-level ROS architecture (e.g., ROS 1 Noetic or ROS 2 Humble/Iron), 
how nodes interact with the low-level serial bridge, and the role of navigation/odometry.
-->

### Nodes & Interfaces
<!--
List the planned or implemented nodes, topics, and services:

- `serial_bridge_node`: Interfaces with /dev/ttyUSB0 and exposes state/cmd topics.
- Published Topics:
  - `/robot/joint_states` (sensor_msgs/JointState)
  - `/robot/telemetry`
- Subscribed Topics:
  - `/cmd_vel` (geometry_msgs/Twist)
  - `/robot/gait_command` (std_msgs/String)
-->

### Launch & Execution
<!--
Provide instructions for building the workspace and running the launch files:

```bash
# Example workspace build
cd ~/catkin_ws  # or ros2_ws
colcon build --symlink-install
source install/setup.bash

# Example launch command
ros2 launch fennec_control bringup.launch.py
