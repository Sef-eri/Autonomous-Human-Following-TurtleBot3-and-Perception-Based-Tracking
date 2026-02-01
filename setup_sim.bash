#!/bin/bash
# TurtleBot3 Warehouse Simulation Setup Script
# Ubuntu 22.04 + ROS 2 Humble

set -e

echo "=== TurtleBot3 Warehouse Simulation Setup ==="

# Check ROS 2 installation
if [ -z "$ROS_DISTRO" ]; then
    echo "Sourcing ROS 2 Humble..."
    source /opt/ros/humble/setup.bash
fi

# Install dependencies
echo "Installing TurtleBot3 packages..."
sudo apt update
sudo apt install -y \
    ros-humble-turtlebot3* \
    ros-humble-gazebo-ros-pkgs \
    ros-humble-gazebo-ros2-control \
    ros-humble-robot-state-publisher \
    ros-humble-cv-bridge \
    ros-humble-image-transport \
    python3-pip

# Install MediaPipe for vision node (Step 2)
pip3 install mediapipe opencv-python numpy

# Set TurtleBot3 model
echo "export TURTLEBOT3_MODEL=waffle_pi" >> ~/.bashrc
export TURTLEBOT3_MODEL=waffle_pi

# Add Gazebo model path
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
echo "export GAZEBO_MODEL_PATH=\$GAZEBO_MODEL_PATH:${SCRIPT_DIR}/src/tb3_warehouse/models" >> ~/.bashrc
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:${SCRIPT_DIR}/src/tb3_warehouse/models

# Build workspace
echo "Building workspace..."
cd ~/tb3_warehouse_ws
colcon build --symlink-install

echo ""
echo "=== Setup Complete ==="
echo ""
echo "To run the simulation:"
echo "  1. source ~/tb3_warehouse_ws/install/setup.bash"
echo "  2. export TURTLEBOT3_MODEL=waffle_pi"
echo "  3. ros2 launch tb3_warehouse warehouse_sim.launch.py"
echo ""
