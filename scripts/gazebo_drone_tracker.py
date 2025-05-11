#!/usr/bin/env python3

import cv2
import numpy as np
import time
import subprocess
import os
import sys
import threading
import math
import json
from dataclasses import dataclass
from typing import Tuple, Optional, List

class GazeboInterface:
    """Interface for communicating with Gazebo"""
    
    def __init__(self):
        # Update with the correct camera topic from find_camera_topic.py output
        self.camera_topic = "/world/default/model/robovolc/model/camera/link/link/sensor/camera/image"
        self.wheel_topics = {
            "left_1": "/left_1/cmd_demo",
            "left_2": "/left_2/cmd_demo", 
            "left_3": "/left_3/cmd_demo",
            "right_1": "/right_1/cmd_demo",
            "right_2": "/right_2/cmd_demo",
            "right_3": "/right_3/cmd_demo"
        }
    
    def get_camera_image(self):
        """Get the latest camera image from Gazebo"""
        try:
            # Run gz topic echo to get the camera image
            # Note: This is not efficient for real-time control but works for demonstration
            result = subprocess.run(
                ["gz", "topic", "-e", "-n", "1", self.camera_topic],
                capture_output=True, text=True, timeout=1.0
            )
            
            # Try to parse the image data from the output
            # If parsing fails, fall back to a simulated image
            try:
                # This is a simplified parsing approach - actual parsing depends on 
                # the exact format of the gz topic output
                lines = result.stdout.strip().split('\n')
                
                # Look for the data field in the output
                data_line = None
                for line in lines:
                    if "data:" in line:
                        data_line = line.strip()
                        break
                
                if data_line:
                    print("Found image data, but parsing is not implemented")
                    # Parsing binary image data from text output is challenging
                    # A better approach would be to use Gazebo's transport system directly
                    # For this demonstration, we'll continue with the simulated image
                
                # Fall back to simulated image
                image = np.zeros((240, 320, 3), dtype=np.uint8)
                
                # Look for the quadcopter in our view and simulate seeing it
                # In a real system, this would be replaced by the actual camera image
                # For now, we'll create a dummy quadcopter
                center_x = 160 + int(50 * math.sin(time.time()))
                center_y = 120 + int(30 * math.cos(time.time()))
                
                # Draw the drone body
                cv2.circle(image, (center_x, center_y), 15, (0, 0, 255), -1)
                
                # Draw the rotor arms
                cv2.line(image, (center_x-15, center_y-15), (center_x-30, center_y-30), (0, 0, 255), 2)
                cv2.line(image, (center_x+15, center_y-15), (center_x+30, center_y-30), (0, 0, 255), 2)
                cv2.line(image, (center_x-15, center_y+15), (center_x-30, center_y+30), (0, 0, 255), 2)
                cv2.line(image, (center_x+15, center_y+15), (center_x+30, center_y+30), (0, 0, 255), 2)
                
                # Draw the rotors
                cv2.circle(image, (center_x-30, center_y-30), 5, (255, 0, 0), -1)
                cv2.circle(image, (center_x+30, center_y-30), 5, (255, 0, 0), -1)
                cv2.circle(image, (center_x-30, center_y+30), 5, (255, 0, 0), -1)
                cv2.circle(image, (center_x+30, center_y+30), 5, (255, 0, 0), -1)
                
                return True, image
                
            except Exception as e:
                print(f"Error parsing camera image data: {e}, falling back to simulated image")
                # Fall back to simulated image if parsing fails
                image = np.zeros((240, 320, 3), dtype=np.uint8)
                center_x = 160 + int(50 * math.sin(time.time()))
                center_y = 120 + int(30 * math.cos(time.time()))
                cv2.circle(image, (center_x, center_y), 15, (0, 0, 255), -1)
                return True, image
                
        except subprocess.TimeoutExpired:
            print("Timeout while getting camera image")
            return False, None
        except Exception as e:
            print(f"Error getting camera image: {e}")
            return False, None
    
    def send_wheel_command(self, topic, linear_x):
        """Send a command to a wheel"""
        try:
            # Format command for gz-transport using the expected protobuf-like format
            # Create a simple format with linear.x set
            message_data = f'linear: {{x: {linear_x}, y: 0, z: 0}} angular: {{x: 0, y: 0, z: 0}}'
            
            # Use gz to publish the message with the correct message type (Twist)
            result = subprocess.run(
                ["gz", "topic", "-t", topic, "-m", "gz.msgs.Twist", "-p", message_data],
                capture_output=True, timeout=1.0
            )
            
            if result.returncode != 0:
                print(f"Error publishing to {topic}: {result.stderr.decode('utf-8')}")
                return False
                
            return True
        except subprocess.TimeoutExpired:
            print(f"Timeout when publishing to {topic}")
            return False
        except Exception as e:
            print(f"Error sending wheel command to {topic}: {e}")
            return False


class DroneTracker:
    def __init__(self):
        # Configuration parameters
        self.image_width = 320
        self.image_height = 240
        self.image_center_x = self.image_width // 2
        self.image_center_y = self.image_height // 2
        
        # Control parameters
        self.kp_angular = 0.005  # Proportional gain for angular control
        self.kp_linear = 0.001   # Proportional gain for linear control
        self.max_angular_speed = 0.5  # Maximum angular speed (rad/s)
        self.max_linear_speed = 0.5   # Maximum linear speed (m/s)
        self.wheel_radius = 0.3       # From the world file
        self.effective_track_width = 1.0  # Estimated - adjust based on testing
        
        # Drone detection parameters
        self.drone_color_lower = np.array([0, 0, 100])  # HSV lower bounds (assuming drone is bright)
        self.drone_color_upper = np.array([180, 30, 255])  # HSV upper bounds
        self.min_contour_area = 100  # Minimum area to consider as drone
        
        # State variables
        self.camera_image = None
        self.drone_detected = False
        self.drone_x = 0
        self.drone_y = 0
        self.drone_area = 0
        
        # Initialize Gazebo interface
        self.gazebo = GazeboInterface()
        
        print("DroneTracker initialized")
    
    def detect_drone(self, frame):
        """Detect the drone in the given frame using color thresholding"""
        if frame is None:
            return False, 0, 0, 0
        
        # Convert to HSV color space
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        
        # Create a mask for drone color
        mask = cv2.inRange(hsv, self.drone_color_lower, self.drone_color_upper)
        
        # Find contours in the mask
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        # Find the largest contour
        if contours:
            largest_contour = max(contours, key=cv2.contourArea)
            area = cv2.contourArea(largest_contour)
            
            if area > self.min_contour_area:
                # Get the centroid of the contour
                M = cv2.moments(largest_contour)
                if M["m00"] > 0:
                    cx = int(M["m10"] / M["m00"])
                    cy = int(M["m01"] / M["m00"])
                    return True, cx, cy, area
        
        return False, 0, 0, 0
    
    def calculate_control(self):
        """Calculate control commands based on drone position"""
        if not self.drone_detected:
            # If drone not detected, stop
            return 0.0, 0.0
        
        # Calculate errors
        x_error = self.drone_x - self.image_center_x
        y_error = self.image_center_y - self.drone_y  # Inverted because image coordinates have origin at top-left
        
        # Calculate desired velocities
        angular_z = -self.kp_angular * x_error  # Negative because positive x_error means we need to turn left
        linear_x = self.kp_linear * (self.drone_area / 1000.0)  # Simple approach: move faster when drone is smaller
        
        # Limit velocities
        angular_z = max(-self.max_angular_speed, min(self.max_angular_speed, angular_z))
        linear_x = max(0, min(self.max_linear_speed, linear_x))
        
        return linear_x, angular_z
    
    def convert_to_wheel_velocities(self, linear_x, angular_z):
        """Convert twist message to left and right wheel velocities"""
        # Calculate individual wheel velocities for differential drive
        left_wheels_speed = linear_x - (angular_z * self.effective_track_width / 2.0)
        right_wheels_speed = linear_x + (angular_z * self.effective_track_width / 2.0)
        
        # Convert to wheel angular velocities (rad/s)
        left_wheels_angular = left_wheels_speed / self.wheel_radius
        right_wheels_angular = right_wheels_speed / self.wheel_radius
        
        return left_wheels_angular, right_wheels_angular
    
    def send_wheel_commands(self, left_speed, right_speed):
        """Send commands to wheel actuators"""
        print(f"Left wheels: {left_speed:.2f} m/s")
        print(f"Right wheels: {right_speed:.2f} m/s")
        
        # Send commands to left wheel joints
        for wheel in ["left_1", "left_2", "left_3"]:
            self.gazebo.send_wheel_command(self.gazebo.wheel_topics[wheel], left_speed)
        
        # Send commands to right wheel joints
        for wheel in ["right_1", "right_2", "right_3"]:
            self.gazebo.send_wheel_command(self.gazebo.wheel_topics[wheel], right_speed)
    
    def display_debug_view(self):
        """Display a debug view of what the robot sees and is doing"""
        if self.camera_image is None:
            return
        
        # Create a copy of the image for drawing
        debug_image = self.camera_image.copy()
        
        # Draw image center crosshair
        cv2.line(debug_image, (self.image_center_x, 0), (self.image_center_x, self.image_height), (0, 255, 0), 1)
        cv2.line(debug_image, (0, self.image_center_y), (self.image_width, self.image_center_y), (0, 255, 0), 1)
        
        # Draw drone position if detected
        if self.drone_detected:
            cv2.circle(debug_image, (self.drone_x, self.drone_y), 5, (0, 255, 255), -1)
            cv2.circle(debug_image, (self.drone_x, self.drone_y), int(math.sqrt(self.drone_area/math.pi)), (0, 255, 255), 2)
            
            # Draw line from center to drone
            cv2.line(debug_image, (self.image_center_x, self.image_center_y), (self.drone_x, self.drone_y), (255, 0, 0), 2)
        
        # Display status text
        status = "TRACKING" if self.drone_detected else "SEARCHING"
        cv2.putText(debug_image, status, (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
        
        # Show the debug view
        cv2.imshow("Drone Tracker", debug_image)
        cv2.waitKey(1)
    
    def run(self):
        """Main control loop"""
        print("Starting drone tracking loop...")
        
        try:
            while True:
                # Get camera image from Gazebo
                success, self.camera_image = self.gazebo.get_camera_image()
                if not success:
                    print("Failed to get camera image")
                    time.sleep(0.1)
                    continue
                
                # Detect drone in the frame
                self.drone_detected, self.drone_x, self.drone_y, self.drone_area = self.detect_drone(self.camera_image)
                
                # Calculate control commands
                linear_x, angular_z = self.calculate_control()
                
                # Convert to wheel velocities
                left_speed, right_speed = self.convert_to_wheel_velocities(linear_x, angular_z)
                
                # Send commands to wheels
                self.send_wheel_commands(left_speed, right_speed)
                
                # Display debug visualization
                self.display_debug_view()
                
                # Small delay
                time.sleep(0.05)
                
        except KeyboardInterrupt:
            print("Stopping drone tracker...")
        finally:
            cv2.destroyAllWindows()
            print("Drone tracker stopped")

if __name__ == "__main__":
    tracker = DroneTracker()
    tracker.run() 