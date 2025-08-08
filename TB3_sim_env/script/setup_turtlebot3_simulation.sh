#!/bin/bash

# Turtlebot3 Simulation Build Script for ROS2 Jazzy
# This script builds Turtlebot3 packages for simulation

set -e

echo "=========================================="
echo "Turtlebot3 Simulation Build for ROS2 Jazzy"
echo "=========================================="

# Set workspace path
WORKSPACE_PATH="$HOME/ros2_ws"

# Check if workspace exists
if [ ! -d "$WORKSPACE_PATH" ]; then
    echo "Error: Workspace $WORKSPACE_PATH does not exist!"
    echo "Please ensure Turtlebot3 packages are installed via Docker build."
    exit 1
fi

echo ""
echo "Step 1: Setting up environment variables..."
echo "--------------------------------------------"

# Check if .bashrc already contains the export
if ! grep -q "export TURTLEBOT3_MODEL" ~/.bashrc; then
    echo "" >> ~/.bashrc
    echo "# Turtlebot3 Configuration" >> ~/.bashrc
    echo "export TURTLEBOT3_MODEL=burger" >> ~/.bashrc
    echo "Added TURTLEBOT3_MODEL=burger to ~/.bashrc"
else
    echo "TURTLEBOT3_MODEL already set in ~/.bashrc"
fi

# Also export for current session
export TURTLEBOT3_MODEL=burger

echo ""
echo "Step 2: Building the workspace..."
echo "----------------------------------"
cd "$WORKSPACE_PATH"

# Source ROS2 setup
source /opt/ros/jazzy/setup.bash

# Build only simulation-related packages (skip packages that require DynamixelSDK)
echo "Building simulation packages only..."
echo "Note: Skipping turtlebot3_node, turtlebot3_bringup (require DynamixelSDK for real hardware)"
echo "Note: Skipping turtlebot3_fake_node to avoid conflicts with Gazebo simulation"

# Build specific packages needed for simulation (excluding turtlebot3_fake_node to avoid conflicts)
colcon build --symlink-install --packages-select \
    turtlebot3_msgs \
    turtlebot3_description \
    turtlebot3_gazebo \
    turtlebot3_cartographer \
    turtlebot3_navigation2 \
    turtlebot3_teleop

echo ""
echo "=========================================="
echo "Build Complete!"
echo "=========================================="
echo ""
echo "To use the Turtlebot3 simulation environment:"
echo "1. Source the workspace: source $WORKSPACE_PATH/install/setup.bash"
echo "2. Launch integrated simulation with Nav2:"
echo "   ros2 launch tb3_ctrl_bringup tb3_nav2_simple.launch.py"
echo "3. Or launch individual components:"
echo "   ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py"
echo ""
echo "Note: Default robot model is set to 'burger'."
echo "You can change it by setting: export TURTLEBOT3_MODEL=waffle or waffle_pi"
echo ""