#!/usr/bin/env python3

import cv2
import numpy as np
import time
import sys
import threading
import math
import signal
import gz.transport13
from gz.msgs11 import image_pb2, twist_pb2
from typing import Tuple

class GazeboDirectInterface:
    """Interface for communicating directly with Gazebo using gz.transport"""
    
    def __init__(self):
        # Initialize Gazebo transport
        self.node = gz.transport13.Node()
        
        # Set up topics
        self.camera_topic = "/world/default/model/robovolc/model/camera/link/link/sensor/camera/image"
        self.wheel_topics = {
            "left_1": "/left_1/cmd_demo",
            "left_2": "/left_2/cmd_demo", 
            "left_3": "/left_3/cmd_demo",
            "right_1": "/right_1/cmd_demo",
            "right_2": "/right_2/cmd_demo",
            "right_3": "/right_3/cmd_demo"
        }
        
        # Set up subscribers
        self.latest_image = None
        self.image_lock = threading.Lock()
        
        # Subscribe to camera images - Note: msg_type must come first in the subscribe call
        if not self.node.subscribe(image_pb2.Image, self.camera_topic, self.on_camera_image):
            print(f"Error subscribing to {self.camera_topic}")
        else:
            print(f"Subscribed to {self.camera_topic}")
    
    def on_camera_image(self, msg):
        """Callback for camera images"""
        try:
            # Convert image message to OpenCV format
            width = msg.width
            height = msg.height
            
            # Extract pixel data (assuming RGB8 format)
            if msg.pixel_format_type == 3:  # RGB_INT8
                img_data = np.frombuffer(msg.data, dtype=np.uint8)
                img_data = img_data.reshape((height, width, 3))
                
                # OpenCV uses BGR, so convert if necessary
                img_data = cv2.cvtColor(img_data, cv2.COLOR_RGB2BGR)
            else:
                print(f"Unsupported pixel format: {msg.pixel_format_type}")
                return
            
            # Store the image
            with self.image_lock:
                self.latest_image = img_data
        except Exception as e:
            print(f"Error processing camera image: {e}")
    
    def get_camera_image(self):
        """Get the latest camera image"""
        # Wait a bit for the first image to arrive
        if self.latest_image is None:
            time.sleep(0.1)
        
        # Return a copy of the latest image
        with self.image_lock:
            if self.latest_image is not None:
                return True, self.latest_image.copy()
            else:
                # Fall back to simulated image if no real image available
                image = np.zeros((240, 320, 3), dtype=np.uint8)
                center_x = 160 + int(50 * math.sin(time.time()))
                center_y = 120 + int(30 * math.cos(time.time()))
                
                # Draw a simulated drone
                cv2.circle(image, (center_x, center_y), 15, (0, 0, 255), -1)
                cv2.line(image, (center_x-15, center_y-15), (center_x-30, center_y-30), (0, 0, 255), 2)
                cv2.line(image, (center_x+15, center_y-15), (center_x+30, center_y-30), (0, 0, 255), 2)
                cv2.line(image, (center_x-15, center_y+15), (center_x-30, center_y+30), (0, 0, 255), 2)
                cv2.line(image, (center_x+15, center_y+15), (center_x+30, center_y+30), (0, 0, 255), 2)
                
                cv2.circle(image, (center_x-30, center_y-30), 5, (255, 0, 0), -1)
                cv2.circle(image, (center_x+30, center_y-30), 5, (255, 0, 0), -1)
                cv2.circle(image, (center_x-30, center_y+30), 5, (255, 0, 0), -1)
                cv2.circle(image, (center_x+30, center_y+30), 5, (255, 0, 0), -1)
                
                return False, image
    
    def send_wheel_command(self, topic, linear_x):
        """Send a command to a wheel"""
        try:
            # Create a Twist message
            twist_msg = twist_pb2.Twist()
            # Set the linear and angular velocities
            twist_msg.linear.x = linear_x
            twist_msg.linear.y = 0.0
            twist_msg.linear.z = 0.0
            twist_msg.angular.x = 0.0
            twist_msg.angular.y = 0.0
            twist_msg.angular.z = 0.0
            
            # Create a publisher for this topic
            pub = self.node.advertise(topic, twist_pb2.Twist)
            
            # Publish the message
            pub.publish(twist_msg)
            return True
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
        self.gazebo = GazeboDirectInterface()
        
        # Setup signal handler for clean shutdown
        signal.signal(signal.SIGINT, self.signal_handler)
        
        print("DroneTracker initialized")
    
    def signal_handler(self, sig, frame):
        """Handle Ctrl+C gracefully"""
        print("Stopping drone tracker...")
        cv2.destroyAllWindows()
        print("Drone tracker stopped")
        sys.exit(0)
    
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