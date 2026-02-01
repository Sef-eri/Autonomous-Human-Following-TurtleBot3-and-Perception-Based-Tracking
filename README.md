# Autonomous-Human-Following-TurtleBot3-and-Perception-Based-Tracking

![Built with ROS 2 Humble](https://img.shields.io/badge/ROS%202-Humble%20Hawksbill-blue?logo=ros)
![Python 3.10+](https://img.shields.io/badge/Python-3.10%2B-3776AB?logo=python&logoColor=white)
![Ubuntu 22.04](https://img.shields.io/badge/Ubuntu-22.04%20Jammy-E95420?logo=ubuntu&logoColor=white)
![Gazebo Classic](https://img.shields.io/badge/Gazebo-Classic%2011-orange)

## Description

A **TurtleBot3 Waffle Pi** that autonomously follows a color-marked target through a simulated warehouse while avoiding obstacles with LiDAR.

This project began as a **MediaPipe-based person-following** system. In practice, MediaPipe's pose-estimation models — trained on photorealistic images — failed against Gazebo's synthetic meshes (jittery landmarks, frequent false negatives). The perception layer was pivoted to **HSV color-space tracking**: a neon-green cylinder stands in for the human operator, and a five-stage OpenCV pipeline (blur → HSV convert → threshold → morphology → contour) delivers deterministic, sub-5 ms detections. The control and safety layers were left unchanged, proving the system's modularity.


### System Architecture

```
/camera/image_raw
        │
  ┌─────▼──────────────┐
  │  Color Tracker Node │──► /target_vector
  │  (HSV detection)    │      (x, y, distance, detected)
  └────────────────────┘
                │
  ┌─────────────▼─────────────┐
  │     Follower Node         │──► /cmd_vel_raw
  │  (PID + Finite State      │
  │   Machine controller)     │
  └───────────────────────────┘
                │
  ┌─────────────▼─────────────┐
  │      Safety Node          │◄── /scan (360° LiDAR)
  │  (Layered obstacle        │
  │   avoidance filter)       │──► /cmd_vel → Robot
  └───────────────────────────┘
```
**Follower FSM States:** `IDLE` → `SEARCHING` → `FOLLOWING` → `STOPPING`

---
## Prerequisites

All commands below assume a **fresh Ubuntu 22.04** installation.

### 1 — ROS 2 Humble (Desktop)

Follow the official guide: [docs.ros.org/en/humble/Installation](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debs.html), or run:

### 2 — Gazebo Classic & ROS Integration

### 3 — TurtleBot3 Packages

```bash
sudo apt install -y ros-humble-turtlebot3 \
                    ros-humble-turtlebot3-gazebo \
                    ros-humble-turtlebot3-msgs \
                    ros-humble-turtlebot3-description \
                    ros-humble-turtlebot3-teleop
```

### 4 — Additional ROS 2 Packages

```bash
sudo apt install -y ros-humble-cv-bridge \
                    ros-humble-robot-state-publisher \
                    ros-humble-joint-state-publisher \
                    python3-colcon-common-extensions \
                    python3-rosdep
```

### 5 — Python Dependencies

```bash
pip install opencv-python numpy
```

### 6 — Environment Variable

The TurtleBot3 stack requires a model selector. Add to `~/.bashrc`:

```bash
echo 'export TURTLEBOT3_MODEL=waffle_pi' >> ~/.bashrc
source ~/.bashrc
```

---

## Installation

### Step 1 — Create the workspace

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
```

### Step 2 — Clone the repository

```bash
git clone https://github.com/Sef-eri/Autonomous-Human-Following-TurtleBot3-and-Perception-Based-Tracking.git

```

### Step 3 — Install dependencies with `rosdep`

```bash
cd ~/ros2_ws
sudo rosdep init          # skip if already initialized
rosdep update
rosdep install --from-paths src --ignore-src -r -y
```

### Step 4 — Build with `colcon`

```bash
cd ~/ros2_ws
colcon build
```

### Step 5 — Source the workspace overlay

```bash
source ~/ros2_ws/install/setup.bash
```
## Usage

### Quick Start (Single Command)

```bash
export TURTLEBOT3_MODEL=waffle_pi
ros2 launch tb3_warehouse color_follow.launch.py
```

This launches Gazebo → spawns the robot → starts the color tracker, follower, and safety nodes.

---
### To start moving the actor by keyboard

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard cmd_vel:=/target/cmd_vel
```

