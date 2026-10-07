# CSable / Fennec — Quadruped Robot Control & Electrical System

This repository contains the electrical and control framework for a quadruped robot featuring a **2-DOF parallel-link leg mechanism** driven by ROBOTIS Dynamixel smart actuators. 

The system operates across two main environments:
* **Embedded System:** Real-time 100 Hz gait and inverse kinematics execution on an Arduino Mega 2560 supervised by a Raspberry Pi via UART[cite: 1, 9].
* **Control & Simulation (MATLAB):** Kinematic analysis, workspace validation, open-loop trajectory tracking, and Virtual Model Control (impedance control).

---

## Hardware & Electrical Overview

* **Microcontroller:** Arduino Mega 2560 with ROBOTIS DYNAMIXEL Shield[cite: 1].
  * DYNAMIXEL Bus: Hardware `Serial` at 1 Mbps[cite: 1].
  * Raspberry Pi Interface: `Serial2` (TX2 pin 16, RX2 pin 17) at 115,200 baud[cite: 1].
* **Companion Computer:** Raspberry Pi running high-level commands over USB/UART (`/dev/ttyUSB0`)[cite: 9].
* **Host PC Interface (MATLAB):** ROBOTIS U2D2 USB-to-RS485/TTL adapter[cite: 10].
* **Actuators (12 motors total):**
  * **8 × Dynamixel XM430-W350-R:** Control thigh and crank parallel linkages (Protocol 2.0, 1 Mbps).
  * **4 × Dynamixel AX-12A:** Hold lateral Z-axis hip abduction/adduction angles (Protocol 1.0, 1 Mbps).
* **Power Supply:** 12 V DC (≥ 5 A recommended)[cite: 10].

---

## Control Architecture

### Embedded Arduino Loop (100 Hz)
* **Gait Generation:** Parameterized cycloid trajectories generating diagonal trot patterns[cite: 1].
* **Kinematics Engine:** Onboard analytical inverse and forward kinematics for the 2-DOF parallel linkage[cite: 1].
* **Safety Limiting:** Dynamic joint rate limiting clamped at 240°/s to prevent servo strain[cite: 1].

### MATLAB Control Stack
* **Kinematics (`+kinematics`):** Forward/inverse kinematics, 2D numerical Jacobian computation, and workspace sweep validation[cite: 10].
* **Controllers (`+control`):**
  * `OpenLoopControl`: Kinematic trajectory tracking with velocity limits[cite: 10].
  * `VirtualModelControl` (VMC): Cartesian compliance via virtual spring-dampers ($\boldsymbol{\tau} = \mathbf{J}^T \mathbf{F}$) under current/torque control[cite: 10].
* **Gait & Motor Testing:** Tools for tracking paw trajectories from reference video (`fennec_*.m`)[cite: 3, 4] and motor characterization scripts (`xm430_*.m`, `ax12a_*.m`)[cite: 2, 6].

---

## Serial Communication

The Arduino listens on UART2 at 115,200 baud for newline-terminated ASCII commands from the companion computer[cite: 1]:

* `WALK` / `W` — Starts the walking trot gait[cite: 1].
* `STAND` / `S` — Transitions to the neutral standing pose[cite: 1].
* `TOGGLE` / `T` / `SPACE` — Toggles between walking and standing modes[cite: 1].
* `STATUS` / `PING` — Reports operating state, phase index, and motor positions[cite: 1].
* `QUIT` / `Q` — Disables motor torque and safely stops the system[cite: 1].

---

## ROS Integration

> **Note:** Reserved for high-level ROS package documentation once implemented.

### Architecture Overview
<!-- Outline high-level nodes, serial bridging to the Arduino Mega, and navigation stack integration. -->

### Nodes & Interfaces
<!--
Planned topics and nodes:
- `serial_bridge_node`: Interfaces with /dev/ttyUSB0
- Published: `/robot/joint_states`, `/robot/telemetry`
- Subscribed: `/cmd_vel`, `/robot/mode`
-->

### Build & Bringup
<!--
```bash
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash
ros2 launch fennec_bringup robot.launch.py
