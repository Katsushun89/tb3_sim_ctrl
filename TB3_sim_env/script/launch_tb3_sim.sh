#!/bin/bash

# Turtlebot3 Simulation Launcher Script

# Set default robot model if not already set
if [ -z "$TURTLEBOT3_MODEL" ]; then
    export TURTLEBOT3_MODEL=burger
    echo "Setting TURTLEBOT3_MODEL=burger"
else
    echo "Using TURTLEBOT3_MODEL=$TURTLEBOT3_MODEL"
fi

# Source ROS2 setup
source /opt/ros/jazzy/setup.bash

# Source workspace
if [ -f /home/s-katsu/ros2_ws/install/setup.bash ]; then
    source /home/s-katsu/ros2_ws/install/setup.bash
    echo "Workspace sourced"
else
    echo "Error: Workspace not built. Please run setup_turtlebot3_simulation.sh first"
    exit 1
fi

# Parse command line arguments
LAUNCH_TYPE=${1:-world}

case $LAUNCH_TYPE in
    empty)
        echo "Launching Turtlebot3 in empty world..."
        ros2 launch turtlebot3_gazebo empty_world.launch.py
        ;;
    world)
        echo "Launching Turtlebot3 in Turtlebot3 world..."
        ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
        ;;
    house)
        echo "Launching Turtlebot3 in house world..."
        ros2 launch turtlebot3_gazebo turtlebot3_house.launch.py
        ;;
    *)
        echo "Usage: $0 [empty|world|house]"
        echo "  empty - Launch in empty world"
        echo "  world - Launch in Turtlebot3 world (default)"
        echo "  house - Launch in house world"
        exit 1
        ;;
esac