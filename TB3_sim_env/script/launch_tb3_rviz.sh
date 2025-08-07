#!/bin/bash

# Turtlebot3 RViz2 Launcher Script

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

echo "Launching RViz2 with Turtlebot3 configuration..."
echo "This configuration includes:"
echo "- Robot model visualization"
echo "- LiDAR scan display (/scan)"
echo "- Map display (/map)"
echo "- TF frames"
echo "- Goal pose visualization"

ros2 run rviz2 rviz2 -d /home/s-katsu/ros2_ws/src/TB3_sim_ctrl/TB3_sim_env/config/tb3_simulation.rviz