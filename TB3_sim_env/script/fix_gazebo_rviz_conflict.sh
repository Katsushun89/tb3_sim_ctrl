#!/bin/bash

# Script to fix Gazebo-RViz positioning conflict
# This removes turtlebot3_fake_node which conflicts with Gazebo simulation

echo "==========================================="
echo "Fixing Gazebo-RViz positioning conflict"
echo "==========================================="

# Set workspace path
WORKSPACE_PATH="$HOME/ros2_ws"

# Source ROS2 setup
source /opt/ros/jazzy/setup.bash

echo ""
echo "Step 1: Removing conflicting packages..."
echo "----------------------------------------"

cd "$WORKSPACE_PATH"

# Remove turtlebot3_fake_node from install and build directories
if [ -d "install/turtlebot3_fake_node" ]; then
    echo "Removing turtlebot3_fake_node from install directory..."
    rm -rf install/turtlebot3_fake_node
fi

if [ -d "build/turtlebot3_fake_node" ]; then
    echo "Removing turtlebot3_fake_node from build directory..."
    rm -rf build/turtlebot3_fake_node
fi

echo ""
echo "Step 2: Rebuilding workspace without conflicting packages..."
echo "------------------------------------------------------------"

# Build only the packages we need for Gazebo simulation
colcon build --symlink-install --packages-select \
    turtlebot3_msgs \
    turtlebot3_description \
    turtlebot3_gazebo \
    turtlebot3_cartographer \
    turtlebot3_navigation2 \
    turtlebot3_teleop

echo ""
echo "Step 3: Checking for any running fake_node processes..."
echo "--------------------------------------------------------"

# Kill any running turtlebot3_fake_node processes
if pgrep -f "turtlebot3_fake_node" > /dev/null; then
    echo "Found running turtlebot3_fake_node processes. Terminating..."
    pkill -f "turtlebot3_fake_node"
else
    echo "No turtlebot3_fake_node processes found."
fi

echo ""
echo "==========================================="
echo "Fix Complete!"
echo "==========================================="
echo ""
echo "The conflict has been resolved by:"
echo "1. Removing turtlebot3_fake_node package"
echo "2. Rebuilding workspace without conflicting packages"
echo "3. Terminating any running fake_node processes"
echo ""
echo "Now you can run Gazebo simulation without positioning conflicts:"
echo "  ./launch_tb3_sim.sh world rviz"
echo ""