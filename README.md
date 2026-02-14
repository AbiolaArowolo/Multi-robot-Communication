# Multi-Robot Communication & Orchestration (ROS 2 Humble)

**GitHub:** https://github.com/AbiolaArowolo/Multi-robot-Communication

This repository contains the source code for **Task 3: Heterogeneous Multi-Robot Communication**, developed as part of ongoing research with Assistant Professor Doyun Lee. It demonstrates a bi-directional handshake between a **Clearpath Husky A200** and a **TurtleBot3 Burger** within a shared simulation environment.

---

## Current Status (Last Updated: February 15, 2026)

| Feature | Status |
|---|---|
| Bi-directional communication (Husky ↔ TurtleBot3) |  Complete |
| Gazebo simulation (Husky) |  Complete |
| RViz auto-launch with pre-configured layout |  Complete |
| Husky TF / RobotModel in RViz |  Complete |
| TurtleBot3 TF frames in RViz |  Present |
| TurtleBot3 full Gazebo visualization |  See Technical Note below |

---

## Architecture Overview

| Component | Description |
|---|---|
| Orchestration | Custom `task3_launcher` package |
| Approach | System-Hybrid Binary — bypasses `gz_math_vendor` / GnuTLS dependency conflicts |
| Namespacing | `/husky` and `/turtlebot3` — full isolation, zero sensor/transform collision |
| Communication | Bi-directional ROS 2 topic handshake via custom Python nodes |
| TF Relay | `topic_tools/relay` bridges `/cpr_a200_0000/tf` → `/tf` for RViz |
| Visualization | RViz2 auto-launches at 5 seconds with pre-configured `dual_robot.rviz` |

---

## Technical Note — Ignition vs Gazebo Classic Incompatibility

During development, a fundamental simulator incompatibility was identified and documented:

**Finding:** The Clearpath Husky simulation runs on **Ignition Gazebo 6** (`ign gazebo`), while TurtleBot3's SDF plugins (`libgazebo_ros_diff_drive.so`, `libgazebo_ros_joint_state_publisher.so`) are compiled for **Gazebo Classic**.

**Impact:** TurtleBot3 can be spawned into the Ignition world (`/world/warehouse/create` service confirmed working) but its ROS 2 control plugins do not initialize, so `/turtlebot3/joint_states`, `/turtlebot3/odom`, and `/turtlebot3/cmd_vel` are not published.

**Result:** TurtleBot3 TF frames are visible in RViz via `robot_state_publisher`, but full Gazebo physics and joint animation require a unified simulator environment.

**Resolution path for Task 4:** Upgrade to a unified Ignition-compatible TurtleBot3 model or use a separate Gazebo Classic instance for TurtleBot3.

---

## Prerequisites

- **OS:** Ubuntu 22.04
- **ROS 2:** Humble Hawksbill (system-installed)
- **Required packages:**

```bash
sudo apt install ros-humble-topic-tools
sudo apt install ros-humble-gazebo-ros
sudo apt install ros-humble-robot-state-publisher
```

> **Note:** This package uses system binaries. You do **not** need to build `clearpath_gz` or `turtlebot3` from source.

---

## Installation

### 1. Clone the repository

```bash
mkdir -p ~/multi_robot_ws/src
cd ~/multi_robot_ws/src
git clone https://github.com/AbiolaArowolo/Multi-robot-Communication.git task3_launcher
```

### 2. Set TurtleBot3 model

```bash
echo "export TURTLEBOT3_MODEL=burger" >> ~/.bashrc
echo "export GAZEBO_MODEL_PATH=/opt/ros/humble/share/turtlebot3_gazebo/models:$GAZEBO_MODEL_PATH" >> ~/.bashrc
source ~/.bashrc
```

### 3. Build the workspace

```bash
cd ~/multi_robot_ws
colcon build --packages-select task3_launcher
```

### 4. Source the workspace

```bash
source ~/multi_robot_ws/install/setup.bash
```

### 5. Add kill alias (recommended)

```bash
echo "alias killros='killall -9 gzserver gzclient ros2 ruby rviz2'" >> ~/.bashrc
source ~/.bashrc
```

---

## Running the Simulation

### Terminal 1 — Launch everything

```bash
cd ~/multi_robot_ws
source install/setup.bash
ros2 launch task3_launcher dual_robot.launch.py
```

**Launch timeline:**
- **0s** — Husky simulation starts, TF relay begins
- **5s** — RViz opens automatically (pre-configured)
- **10s** — TurtleBot3 spawn attempt
- **12s** — TurtleBot3 robot_state_publisher starts

### Terminal 2 — Start Husky Brain

```bash
source ~/multi_robot_ws/install/setup.bash
ros2 run task3_launcher husky_talker
```

### Terminal 3 — Start TurtleBot3 Brain

```bash
source ~/multi_robot_ws/install/setup.bash
ros2 run task3_launcher turtle_talker
```

### Terminal 4 — Drive Husky (optional demo)

```bash
source ~/multi_robot_ws/install/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r cmd_vel:=/cpr_a200_0000/cmd_vel
```

---

## Expected Output

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
│   └── dual_robot.launch.py            # Orchestrator — world, robots, TF relay, RViz
├── task3_launcher/
│   ├── husky_talker.py                 # Husky brain — publisher + subscriber
│   └── turtle_talker.py               # TurtleBot brain — subscriber + replier
├── config/
│   └── dual_robot.rviz                 # Pre-configured RViz layout
├── models/
│   └── turtlebot3_burger/
│       └── model.sdf                   # Modified SDF with /turtlebot3 namespace
│   └── turtlebot3_common/
│       └── meshes/                     # Local mesh files for spawning
├── setup.py                            # Package registry
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

## TF Architecture

```
Ignition Gazebo (Clearpath)
  └── /cpr_a200_0000/tf  ──relay──►  /tf  ◄── RViz (Fixed Frame: base_link)
  └── /cpr_a200_0000/tf_static  ──relay──►  /tf_static

TurtleBot3 robot_state_publisher
  └── publishes to /tf with frame_prefix: turtlebot3/
  └── frames: turtlebot3/base_link, turtlebot3/base_footprint, etc.
```

---

## Session Log

### February 11, 2026 — Task 3 Complete
- Bi-directional communication established between Husky and TurtleBot3
- System-Hybrid Binary approach implemented to bypass dependency issues
- Custom `task3_launcher` package created

### February 14-15, 2026 — Visualization Work
- RViz auto-launch with pre-configured layout implemented
- Husky TF relay from `/cpr_a200_0000/tf` → `/tf` working
- Husky RobotModel showing **Status: Ok** in RViz
- TurtleBot3 `robot_state_publisher` added with `frame_prefix: turtlebot3`
- TurtleBot3 frames visible in RViz
- Identified Ignition vs Gazebo Classic incompatibility
- Modified TurtleBot3 `model.sdf` with `/turtlebot3` namespace
- Confirmed `/world/warehouse/create` as correct Ignition spawn service

### Next Steps — Task 4
1. Implement Ignition-compatible TurtleBot3 model
2. Upgrade messages from text to Odometry coordinates
3. Implement TurtleBot3 follow-the-leader logic
4. Record final demo video

---

## Troubleshooting

| Problem | Solution |
|---|---|
| `Package 'task3_launcher' not found` | Re-run `colcon build` and `source install/setup.bash` |
| Gazebo crashes / hangs on relaunch | Run `killros` before every relaunch |
| RViz Global Frame error | Confirm `base_link` is set as Fixed Frame in `dual_robot.rviz` |
| TurtleBot3 not spawning | Ensure `TURTLEBOT3_MODEL=burger` is exported |
| No messages in terminals | Confirm both nodes are sourced from the same workspace |
| `relay: command not found` | Run `sudo apt install ros-humble-topic-tools` |

---

## Maintainer

**Abiola Arowolo**
Email: abayo83@gmail.com
Research Supervisor: Assistant Professor Doyun Lee
