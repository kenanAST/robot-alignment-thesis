# Drone Tracking and Robot Control

This directory contains scripts for tracking a drone with a ground robot's camera and controlling the robot to center under the drone.

## Prerequisites

- Ubuntu Linux with Gazebo installed
- Python 3.x with OpenCV
- The robot-alignment-thesis codebase

## Scripts

1. **launch_simulation.sh**: Launches Gazebo with the main.world simulation environment
2. **find_camera_topic.py**: Utility script to identify camera and robot topics in Gazebo
3. **drone_tracker.py**: Basic drone tracking implementation with simulated camera input
4. **gazebo_drone_tracker.py**: Gazebo-specific implementation for drone tracking and robot control

## Usage

### Step 1: Launch the Gazebo simulation

```bash
./scripts/launch_simulation.sh
```

Wait for Gazebo to fully initialize. You should see the terrain, ground robot (robovolc), and quadcopter in the simulation.

### Step 2: Find the camera topic

```bash
./scripts/find_camera_topic.py
```

This will list all relevant topics in Gazebo, including camera topics and robot control topics.

### Step 3: Run the drone tracker

```bash
./scripts/gazebo_drone_tracker.py
```

If the camera topic is different from the default, edit the camera_topic variable in the GazeboInterface class before running.

## How It Works

1. The drone tracker reads images from the ground robot's camera
2. It processes these images to detect the drone in the field of view
3. It calculates how far the drone is from the center of the image
4. It generates control commands to move the robot toward centering the drone
5. These commands are sent to the robot's wheel actuators via Gazebo topics

## Customization

- Adjust control parameters in the DroneTracker class to change how the robot responds
- Modify the drone detection algorithm for better performance in different lighting conditions
- Update the GazeboInterface class if your system has different topic names

## Troubleshooting

- If the robot doesn't move, ensure Gazebo is publishing to the wheel topics
- If drone detection is unreliable, try adjusting the HSV color thresholds
- For camera connection issues, check that the camera topic exists and has the expected format 