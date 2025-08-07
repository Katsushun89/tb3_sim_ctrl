#!/bin/bash

# Script to clean up all Gazebo and ROS processes to ensure clean startup

echo "========================================="
echo "Cleaning up Gazebo and ROS processes"
echo "========================================="

echo "Killing existing Gazebo processes..."
pkill -f gazebo || true
pkill -f gzserver || true
pkill -f gzclient || true
pkill -f ign || true

echo "Killing RViz2 processes..."
pkill -f rviz2 || true

echo "Killing turtlebot3 related processes..."
pkill -f turtlebot3 || true

echo "Killing robot_state_publisher processes..."
pkill -f robot_state_publisher || true

echo "Killing joint_state_publisher processes..."
pkill -f joint_state_publisher || true

echo "Waiting for processes to terminate..."
sleep 2

echo "Clearing shared memory segments..."
ipcs -s | awk '/^0x/ {print $2}' | xargs -r ipcrm sem || true

echo "Cleanup complete!"
echo ""
echo "You can now safely start the simulation:"
echo "  ./launch_tb3_sim.sh world rviz"
echo ""