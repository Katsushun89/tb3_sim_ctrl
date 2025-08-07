#!/bin/bash

# Turtlebot3 Simulation Environment Setup Script for ROS2 Jazzy
# This script installs all necessary packages for running Turtlebot3 in Gazebo simulation

set -e

echo "=========================================="
echo "Turtlebot3 Simulation Setup for ROS2 Jazzy"
echo "=========================================="

# Function to check if a package is already cloned
check_and_clone() {
    local repo_url=$1
    local target_dir=$2
    local branch=$3
    
    if [ -d "$target_dir" ]; then
        echo "Directory $target_dir already exists. Skipping clone."
    else
        echo "Cloning $repo_url to $target_dir..."
        if [ -n "$branch" ]; then
            git clone -b "$branch" "$repo_url" "$target_dir"
        else
            git clone "$repo_url" "$target_dir"
        fi
    fi
}

# Set workspace path
WORKSPACE_PATH="$HOME/ros2_ws"
SRC_PATH="$WORKSPACE_PATH/src"

# Check if workspace exists
if [ ! -d "$WORKSPACE_PATH" ]; then
    echo "Creating ROS2 workspace at $WORKSPACE_PATH..."
    mkdir -p "$SRC_PATH"
fi

cd "$SRC_PATH"

echo ""
echo "Step 1: Installing Gazebo (formerly Ignition) packages..."
echo "-----------------------------------------------------------"
echo "Note: ROS2 Jazzy uses the new Gazebo, not Gazebo Classic"
sudo apt update
sudo apt install -y \
    ros-jazzy-ros-gz \
    ros-jazzy-ros-gz-bridge \
    ros-jazzy-ros-gz-sim \
    ros-jazzy-ros-gz-interfaces

echo ""
echo "Step 2: Installing Turtlebot3 dependencies..."
echo "----------------------------------------------"
sudo apt install -y \
    ros-jazzy-cartographer \
    ros-jazzy-cartographer-ros \
    ros-jazzy-navigation2 \
    ros-jazzy-nav2-bringup

echo ""
echo "Step 3: Cloning Turtlebot3 packages..."
echo "---------------------------------------"

# Clone turtlebot3_msgs (using jazzy branch)
check_and_clone \
    "https://github.com/ROBOTIS-GIT/turtlebot3_msgs.git" \
    "$SRC_PATH/turtlebot3_msgs" \
    "jazzy"

# Clone turtlebot3 (using jazzy branch)
check_and_clone \
    "https://github.com/ROBOTIS-GIT/turtlebot3.git" \
    "$SRC_PATH/turtlebot3" \
    "jazzy"

# Clone turtlebot3_simulations (using jazzy branch)
check_and_clone \
    "https://github.com/ROBOTIS-GIT/turtlebot3_simulations.git" \
    "$SRC_PATH/turtlebot3_simulations" \
    "jazzy"

echo ""
echo "Step 4: Setting up environment variables..."
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
echo "Step 5: Building the workspace..."
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
echo "Setup Complete!"
echo "=========================================="
echo ""
echo "To use the Turtlebot3 simulation environment:"
echo "1. Source the workspace: source $WORKSPACE_PATH/install/setup.bash"
echo "2. Launch simulation with empty world:"
echo "   ros2 launch turtlebot3_gazebo empty_world.launch.py"
echo "3. Or launch with Turtlebot3 world:"
echo "   ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py"
echo ""
echo "Note: If using the new Gazebo (not Classic), launch commands may differ."
echo "Check if turtlebot3_gazebo package supports new Gazebo or use ros_gz_sim."
echo ""
echo "Note: Default robot model is set to 'burger'."
echo "You can change it by setting: export TURTLEBOT3_MODEL=waffle or waffle_pi"
echo ""