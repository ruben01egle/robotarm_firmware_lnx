# Distributed ROS 2 Robot Control Architecture

This repository document outlines the professional, modular, and real-time capable architecture designed for a distributed ROS 2 robot system. The architecture separates concerns by isolating interfaces, high-level orchestration, and low-level real-time hardware execution across multiple machines.

---

## 1. Repository Split (The 4-Repo Strategy)

To maximize modularity, prevent circular dependencies, and isolate compiling environments, the codebase is split into four distinct repositories. Two act as "Data/Interface Contracts" and two act as "Application Logic".
```
[ INTERFACES / CONTRACTS ]
 ├── robotarm_description     (URDF, Xacro, STLs)
 └── robotarm_interfaces      (Custom Messages/Services/Actions)
         ▲                 ▲
         │ (Submodule)     │ (Submodule)
 [ APPLICATIONS / LOGIK ]
 ├── robotarm_core            (High-Level Navigation/GUI) ──> Runs on PC
 └── robotarm_firmware_lnx    (Low-Level Hardware Driver) ──> Runs on RT PC
```

### 📁 1. `robot_description` (Interface)
* **Purpose:** The single source of truth for physical robot geometry and kinematics. Contains **no executable code**.
* **Contents:** URDF/Xacro files, 3D Mesh files (`.stl`, `.dae`), and joint limit configurations.
* **Distribution:** Included as a Git submodule or local package on **both** PCs.

### 📁 2. `robot_interfaces` (Interface)
* **Purpose:** Defines how components talk to each other. Contains **no application logic**.
* **Contents:** Custom ROS 2 messages (`.msg`), services (`.srv`, e.g., `TunnelCanHardware.srv`), and actions (`.action`).
* **Distribution:** Included as a Git submodule on **both** PCs.

### 📁 3. `robot_core` (Application)
* **Purpose:** Handles cognitive task planning, user interaction, and visualization.
* **Contents:** High-level State Machine, GUI Nodes, MoveIt 2 integration, and central launch files.
* **Distribution:** Compiles and runs exclusively on **PC 1** (High-Level PC).

### 📁 4. `robot_firmware` (Application)
* **Purpose:** Handles fast, deterministic hardware interfacing and motor control loops.
* **Contents:** Custom `ros2_control::SystemInterface` written in C++ wrapping the CAN-FD driver stack.
* **Distribution:** Compiles and runs exclusively on **PC 2** (Real-Time/Robot PC).

---

## 2. Hardware Distributed Setup (Network Topology)

The system is deployed over a local network fabric using ROS 2 DDS communication.

* **PC 1: High-Level / User Space (Non-RT):** Responsible for running the GUI, processing heavy algorithms, and hosting the `robot_state_publisher`. It parses the URDF + STLs from its local copy of `robot_description` and publishes the text structure to the network over `/robot_description` while rendering the 3D meshes locally in RViz.
* **PC 2: Embedded / Real-Time (RT Linux / Preempt-RT):** Connected directly to the physical robot via CAN-FD. It loads the text-only kinematics directly from its local copy of the Xacro file at startup, ensuring it can boot **independently** of PC 1 network availability. It completely ignores heavy 3D STL visual meshes to conserve RAM and CPU cycles.

---

## 3. Orchestration & The "Single Master" Principle

To avoid conflicting commands ("two masters"), control authority follows a strict hierarchy:

* **The High-Level State Machine (PC 1) is the ultimate Master.** It orchestrates overall system states (e.g., `IDLE`, `PLANNING`, `MOVING`, `CONFIG_MODE`, `ERROR`).
* **The Low-Level Controllers (PC 2) are reactive execution tools.** They mathematically compute inputs or stream states but have no contextual awareness of *why* they are running.
* **The Telemetry Stream:** Raw high-frequency feedback (1 kHz) is gathered natively on PC 2. The `joint_state_broadcaster` downsamples this data to a network-friendly frequency (e.g., 50 Hz) and streams standard `/joint_states` back to PC 1 for the GUI and visualization. Custom trajectory or telemetry packaging is obsolete, as native ROS 2 actions (`FollowJointTrajectory`) and messages handle this transparently.

---

## 4. Inside `ros2_control`: Execution & Lifecycle

The Real-Time PC executes a strict 1 kHz execution loop managed by the `controller_manager`. The loop is fundamentally split from the active controllers, ensuring deterministic scheduling.

```
             ┌──────────────────────────────────┐
             │   1-kHz Real-Time Loop (PC 2)    │
             └─────────────────┬────────────────┘
                               │
READ PHASE                     ▼
┌──────────────────────────────────────────────────────────────────────┐
│ Custom Hardware Interface: read()                                    │
│ -> Fetches encoder/sensor packets from CAN-FD into RAM               │
└──────────────────────────────┬───────────────────────────────────────┘
                               │
                               │
UPDATE PHASE                   ▼
┌──────────────────────────────────────────────────────────────────────┐
│ Controller Manager routes data through ACTIVE Controllers:           │
│ - JointStateBroadcaster    -> Prepares /joint_states network message │
│ - JointTrajectoryController -> Calculates next cyclic math setpoints │
└──────────────────────────────┬───────────────────────────────────────┘
                               │
                               │
WRITE PHASE                    ▼
┌──────────────────────────────────────────────────────────────────────┐
│ Custom Hardware Interface: write()                                   │
│ -> Flushes computed setpoints/gripper tokens down the CAN-FD bus     │
└──────────────────────────────────────────────────────────────────────┘
```

### Arming & Disarming via Hardware Lifecycle
Hardware state progression utilizes the built-in `ros2_control` component state machine:

1. **Boot / `INACTIVE` (Disarmed):** On startup, the hardware component is `INACTIVE`. However, `read()` runs immediately at 1 kHz. The `JointStateBroadcaster` can be turned on, meaning PC 1 instantly sees the true, safe position of the robot in RViz before any power relays close.
2. **`on_activate()` Hook (Arming):** Triggered asynchronously by a service call from the High-Level State Machine. It fires the "Close Contactors / Power On" CAN frame and blocks inside `on_activate()` with a short timeout until the hardware confirms a healthy state. Returning `SUCCESS` promotes the component to `ACTIVE`.
3. **`ACTIVE` (Armed):** Only now does the High-Level State Machine activate the `JointTrajectoryController`. `write()` begins passing down real path planning setpoints.
4. **`on_deactivate()` Hook (Disarming):** Fires safe shutdown commands to drop motor power and engage physical brakes seamlessly.

---

## 5. Safe Configuration & CAN Tunneling Mode

For administrative tasks (e.g., flashing motor registers, tuning internal PIDs via the GUI), rove CAN messages need to be tunneled directly to the hardware interface. To prevent dual-master race conditions on the CAN bus, a hardware-enforced lockstep pattern is implemented:

1. **Initiation:** The configuration tool asks the High-Level State Machine to enter configuration mode.
2. **Deactivation:** The State Machine stops the `JointTrajectoryController` and commands the hardware component back to `INACTIVE` (Disarmed) via the Controller Manager service. 1 kHz cyclic motion profiles stop completely.
3. **Asynchronous Tunneling Service:** The hardware interface hosts a custom ROS 2 service (`~/tunnel_can_message`). To protect real-time constraints, this service callback **never blocks**. It safely writes payloads to a thread-safe Lock-Free `tx_queue`.
4. **Execution:** During the regular `write()` loop, if the hardware is `INACTIVE`, it checks the queue and schedules the raw configuration frames onto the CAN bus interleaved safely with idle state checking.
5. **Hardware Self-Protection:** If a glitch or bad script calls the tunneling service while the hardware lifecycle is still `ACTIVE` (Armed), the C++ code immediately rejects the request with a `FAILURE` response, keeping the robot safe from unvetted interventions.

---

## 6. High-Frequency Telemetry Logging

For advanced signal analysis, system identification, and controller tuning (e.g., calculating tracking errors or frequency responses), a full and jitter-free **1 kHz telemetry stream** (Position, Velocity, Effort) is required. 

To prevent network congestion and packet drops, the architecture avoids live-streaming this data over the network fabric. Instead, it utilizes **ROS 2 Shared Memory IPC (Inter-Process Communication)** coupled with local file backing.
```
[ PC 2: REAL-TIME CONTROLLER PC ]
┌────────────────────────────────────────────────────────┐
│  RAM (Real-Time Domain)                                │
│  ┌──────────────────────────────────────────────────┐  │
│  │ Shared Memory Pool                               │  │
│  │                                                  │  │
│  │  1. Custom Hardware Interface writes 1 kHz data  │  │
│  │     into local RAM via high-freq ROS 2 Topic.    │  │
│  │                                                  │  │
│  │  2. Local 'ros2 bag' node reads directly from    │  │
│  │     the identical RAM block (Zero Network Load)  │  │
│  │                                                  │  │
│  └────────────────────────┬─────────────────────────┘  │
│                           │                            │
│                           ▼                            │
│  3. Async C++ writer commits data to local NVMe SSD.   │
└────────────────────────────────────────────────────────┘
```
### Data Logging Flow

1. **High-Frequency Publication:** The Custom Hardware Interface or a specialized broadcaster in `robot_firmware` publishes full state-and-command metrics to an internal topic (e.g., `/joint_states_high_freq`) at exactly 1000 Hz.
2. **Zero-Copy Memory Transport:** Because the recording node runs locally on **PC 2**, ROS 2 DDS bypasses the network stack entirely. It passes pointers inside the RAM via Shared Memory, resulting in near-zero CPU overhead for serialization.
3. **Local Storage:** A lightweight, optimized asynchronous logging process (`ros2 bag record`) runs on PC 2, dumping the binary telemetry data directly onto the local SSD (ideally into `.mcap` files).
4. **Offline Analysis (The "Fetch" Request):** Once the robot trajectory or experimental run is complete, the user or high-level master can fetch the telemetry log file from PC 2 via standard file transfer protocols (e.g., SFTP, SCP) for native, deep-dive analysis in MATLAB, Python, or PlotJuggler on **PC 1**.

This setup guarantees hard real-time data integrity for system identification while keeping the distributed network line completely free for runtime high-level execution tasks.