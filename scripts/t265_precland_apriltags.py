#!/usr/bin/env python3

#####################################################
##          librealsense T265 to MAVLink           ##
#####################################################
# This script assumes pyrealsense2.[].so file is found under the same directory as this script
# Install required packages: 
#   pip install pyrealsense2
#   pip install transformations
#   pip3 install dronekit
#   pip3 install apscheduler
# sudo apt-get install python3-pip python3-yaml
# sudo pip3 install rospkg catkin_pkg

# Remove the path to python2 version (added by ROS, see here https://stackoverflow.com/questions/43019951/after-install-ros-kinetic-cannot-import-opencv)
import sys, os
sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages')

import pyrealsense2 as rs

# Prettier prints for reverse-engineering
from pprint import pprint

import numpy as np
import cv2
from apriltag import apriltag

# Get realsense pipeline handle
pipe = rs.pipeline()

# Configure the pipeline
cfg = rs.config()

# Prints a list of available streams, not all are supported by each device
print('Available streams:')
pprint(dir(rs.stream))

# Enable streams you are interested in
cfg.enable_stream(rs.stream.pose) # Positional data (translation, rotation, velocity etc)
cfg.enable_stream(rs.stream.fisheye, 1) # Left camera
cfg.enable_stream(rs.stream.fisheye, 2) # Right camera

# Start the configured pipeline
pipe.start(cfg)

try:
    for _ in range(10):
        frames = pipe.wait_for_frames()

        # Left fisheye camera frame
        left = frames.get_fisheye_frame(1)
        left_data = np.asanyarray(left.get_data())

        # Right fisheye camera frame
        right = frames.get_fisheye_frame(2)
        right_data = np.asanyarray(right.get_data())

        print('Left frame', left_data.shape)
        print('Right frame', right_data.shape)

        # Positional data frame
        pose = frames.get_pose_frame()
        if pose:
            pose_data = pose.get_pose_data()
            print('\nFrame number: ', pose.frame_number)
            print('Position: ', pose_data.translation)
            print('Velocity: ', pose_data.velocity)
            print('Acceleration: ', pose_data.acceleration)
            print('Rotation: ', pose_data.rotation)
finally:
    pipe.stop()
