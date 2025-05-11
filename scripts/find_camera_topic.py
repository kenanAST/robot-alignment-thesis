#!/usr/bin/env python3

import subprocess
import time
import sys
import re

def find_camera_topics():
    """Find camera topics in Gazebo"""
    try:
        # Get all Gazebo topics
        result = subprocess.run(
            ["gz", "topic", "-l"],
            capture_output=True, text=True, timeout=5.0
        )
        
        if result.returncode != 0:
            print(f"Error: Failed to list Gazebo topics: {result.stderr}")
            return []
        
        # Split the output by lines
        topics = result.stdout.strip().split('\n')
        
        # Filter for camera-related topics (adjust pattern as needed)
        camera_topics = [
            topic for topic in topics 
            if re.search(r'camera|image|rgb|depth|video', topic, re.IGNORECASE)
        ]
        
        return camera_topics
        
    except subprocess.TimeoutExpired:
        print("Timeout while listing Gazebo topics")
        return []
    except Exception as e:
        print(f"Error listing Gazebo topics: {e}")
        return []

def find_model_topics(model_name):
    """Find topics related to a specific model"""
    try:
        # Get all Gazebo topics
        result = subprocess.run(
            ["gz", "topic", "-l"],
            capture_output=True, text=True, timeout=5.0
        )
        
        if result.returncode != 0:
            print(f"Error: Failed to list Gazebo topics: {result.stderr}")
            return []
        
        # Split the output by lines
        topics = result.stdout.strip().split('\n')
        
        # Filter for model-related topics
        model_topics = [topic for topic in topics if model_name in topic]
        
        return model_topics
        
    except subprocess.TimeoutExpired:
        print("Timeout while listing Gazebo topics")
        return []
    except Exception as e:
        print(f"Error listing Gazebo topics: {e}")
        return []

def list_all_topics():
    """List all available Gazebo topics"""
    try:
        # Get all Gazebo topics
        result = subprocess.run(
            ["gz", "topic", "-l"],
            capture_output=True, text=True, timeout=5.0
        )
        
        if result.returncode != 0:
            print(f"Error: Failed to list Gazebo topics: {result.stderr}")
            return []
        
        # Split the output by lines
        topics = result.stdout.strip().split('\n')
        
        return topics
        
    except subprocess.TimeoutExpired:
        print("Timeout while listing Gazebo topics")
        return []
    except Exception as e:
        print(f"Error listing Gazebo topics: {e}")
        return []

if __name__ == "__main__":
    print("Waiting 3 seconds for Gazebo to initialize...")
    time.sleep(3)
    
    print("\n=== All Gazebo Topics ===")
    all_topics = list_all_topics()
    for topic in all_topics:
        print(topic)
    
    print("\n=== Camera-related Topics ===")
    camera_topics = find_camera_topics()
    if camera_topics:
        for topic in camera_topics:
            print(topic)
    else:
        print("No camera topics found")
    
    print("\n=== robovolc Model Topics ===")
    robovolc_topics = find_model_topics("robovolc")
    if robovolc_topics:
        for topic in robovolc_topics:
            print(topic)
    else:
        print("No robovolc topics found")
    
    print("\n=== main_quadcopter Topics ===")
    quadcopter_topics = find_model_topics("main_quadcopter")
    if quadcopter_topics:
        for topic in quadcopter_topics:
            print(topic)
    else:
        print("No main_quadcopter topics found") 