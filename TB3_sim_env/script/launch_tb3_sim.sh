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
WITH_RVIZ=${2:-false}

# Function to launch RViz2
launch_rviz() {
    echo "Launching RViz2 with TB3 configuration..."
    ros2 run rviz2 rviz2 -d /home/s-katsu/ros2_ws/src/TB3_sim_ctrl/TB3_sim_env/config/tb3_simulation.rviz &
    RVIZ_PID=$!
    echo "RViz2 launched with PID: $RVIZ_PID"
}

case $LAUNCH_TYPE in
    empty)
        echo "Launching Turtlebot3 in empty world..."
        ros2 launch turtlebot3_gazebo empty_world.launch.py &
        SIM_PID=$!
        ;;
    world)
        echo "Launching Turtlebot3 in Turtlebot3 world..."
        ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py &
        SIM_PID=$!
        ;;
    house)
        echo "Launching Turtlebot3 in house world..."
        ros2 launch turtlebot3_gazebo turtlebot3_house.launch.py &
        SIM_PID=$!
        ;;
    rviz)
        echo "Launching only RViz2..."
        launch_rviz
        wait $RVIZ_PID
        exit 0
        ;;
    *)
        echo "Usage: $0 [empty|world|house|rviz] [rviz]"
        echo "  empty - Launch in empty world"
        echo "  world - Launch in Turtlebot3 world (default)"
        echo "  house - Launch in house world"
        echo "  rviz  - Launch only RViz2"
        echo ""
        echo "Optional second argument:"
        echo "  rviz  - Also launch RViz2 with simulation"
        echo ""
        echo "Examples:"
        echo "  $0 world      # Launch world simulation only"
        echo "  $0 world rviz # Launch world simulation with RViz2"
        echo "  $0 rviz       # Launch only RViz2"
        exit 1
        ;;
esac

# Launch RViz2 if requested
if [ "$WITH_RVIZ" = "rviz" ]; then
    sleep 3  # Wait for simulation to start
    launch_rviz
fi

# Wait for processes
if [ -n "$SIM_PID" ]; then
    echo "Simulation running with PID: $SIM_PID"
    if [ "$WITH_RVIZ" = "rviz" ]; then
        echo "Press Ctrl+C to stop both simulation and RViz2"
        wait $SIM_PID $RVIZ_PID
    else
        echo "Press Ctrl+C to stop simulation"
        wait $SIM_PID
    fi
fi