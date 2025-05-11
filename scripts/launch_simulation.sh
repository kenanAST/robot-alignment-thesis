#!/bin/bash

# Launch Gazebo with our world file
echo "Launching Gazebo with main.world..."
gz sim -r $(pwd)/gazebo/worlds/main.world
 
# If you want to run headless, use:
# gz sim -r -s $(pwd)/gazebo/worlds/main.world 