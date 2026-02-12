# Multi-Robot Communication & Orchestration (ROS 2 Humble)

**GitHub:** https://github.com/AbiolaArowolo/task3_launcher

This repository contains the source code for **Task 3: Heterogeneous Multi-Robot Communication**, developed as part of ongoing research with Assistant Professor Doyun Lee. It demonstrates a bi-directional handshake between a **Clearpath Husky A200** and a **TurtleBot3 Burger** within a namespaced Gazebo simulation.

---

## Architecture Overview

| Component | Description |
|---|---|
| Orchestration | Custom `task3_launcher` package |
| Approach | System-Hybrid Binary (bypasses `gz_math_vendor` / GnuTLS dependency conflicts) |
| Namespacing | `/husky` and `/turtlebot3` — full isolation, zero sensor/transform collision |
| Communication | Bi-directional ROS 2 topic handshake via custom Python nodes |

---

## Prerequisites

- **OS:** Ubuntu 22.04
- **ROS 2:** Humble Hawksbill (system-installed)
- **Required system packages** (pre-installed via `/opt/ros/humble`):
  - `clearpath_gz`
  - `turtlebot3_gazebo`
  - `gazebo_ros`

> **Note:** This package uses system binaries. You do **not** need to build `clearpath_gz` or `turtlebot3` from source.

---

## Installation

### 1. Clone the repository

```bash
mkdir -p ~/multi_robot_ws/src
cd ~/multi_robot_ws/src
git clone https://github.com/AbiolaArowolo/task3_launcher.git
```

### 2. Build the workspace

```bash
cd ~/multi_robot_ws
colcon build --packages-select task3_launcher
```

### 3. Source the workspace

```bash
source ~/multi_robot_ws/install/setup.bash
```

---

## Running the Simulation

Open **three separate terminals** and run the following commands in order.

### Terminal 1 — Launch Simulation (World + Husky + TurtleBot3)

```bash
cd ~/multi_robot_ws
source install/setup.bash
ros2 launch task3_launcher dual_robot.launch.py
```

> A **10-second `TimerAction`** is built into the launch file. This ensures Gazebo is fully stable before the TurtleBot3 is spawned, preventing physics engine crashes.

### Terminal 2 — Start Husky Brain (Broadcaster)

```bash
source ~/multi_robot_ws/install/setup.bash
ros2 run task3_launcher husky_talker
```

### Terminal 3 — Start TurtleBot3 Brain (Listener + Replier)

```bash
source ~/multi_robot_ws/install/setup.bash
ros2 run task3_launcher turtle_talker
```

---

## Expected Output

Once all three terminals are running, you will see a live bi-directional handshake in the logs:

```
[husky_talker]: Husky sent: Hello Turtle from Husky!
[turtle_talker]: Turtle received: Hello Turtle from Husky!
[turtle_talker]: Turtle sent: Hello Husky from Turtle!
[husky_talker]: Husky received: Hello Husky from Turtle!
```

---

## File Structure

```
task3_launcher/
├── launch/
│   └── dual_robot.launch.py       # Orchestrator — starts world, spawns robots, runs nodes
├── task3_launcher/
│   ├── husky_talker.py            # Husky brain — publishes to /turtlebot3/incoming, subscribes to /husky/incoming
│   └── turtle_talker.py           # TurtleBot brain — subscribes to /turtlebot3/incoming, replies to /husky/incoming
├── setup.py                       # Package registry — maps executables for ROS 2
├── package.xml
└── resource/
    └── task3_launcher
```

---

## Communication Topology

```
Husky Node (/husky namespace)
  ├── PUBLISHES  →  /turtlebot3/incoming
  └── SUBSCRIBES ←  /husky/incoming

TurtleBot3 Node (/turtlebot3 namespace)
  ├── SUBSCRIBES ←  /turtlebot3/incoming
  └── PUBLISHES  →  /husky/incoming
```

---

## Troubleshooting

| Problem | Solution |
|---|---|
| `Package 'task3_launcher' not found` | Re-run `colcon build` and `source install/setup.bash` |
| Gazebo crashes on launch | Wait for the 10-second delay to complete; do not interrupt Terminal 1 |
| TurtleBot3 not spawning | Ensure `gazebo_ros` is installed: `sudo apt install ros-humble-gazebo-ros` |
| No messages in Terminal 2/3 | Confirm both nodes are sourced from the same workspace |

---

## Maintainer

**Abiola Arowolo**  
Email: abayo83@gmail.com  
Research Supervisor: Assistant Professor Doyun Lee
