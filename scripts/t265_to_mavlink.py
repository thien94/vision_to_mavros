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

 # Set the path for IDLE
import sys
sys.path.append("/usr/local/lib/")

# Set MAVLink protocol to 2.
import os
os.environ["MAVLINK20"] = "1"

# Import the libraries
import pyrealsense2 as rs
import numpy as np
import transformations as tf
import math as m
import time
import argparse
from apscheduler.schedulers.background import BackgroundScheduler

from dronekit import connect, VehicleMode
from pymavlink import mavutil

#######################################
# Parameters
#######################################

# Default configurations for connection to the FCU
connection_string_default = '/dev/ttyUSB0'
connection_baudrate_default = 921600
vision_msg_hz_default = 30
confidence_msg_hz_default = 1

# In NED frame, offset from the IMU or the center of gravity to the camera's origin point
body_offset_enabled = 0
body_offset_x = 0.05    # In meters, so 0.05 = 5cm
body_offset_y = 0       # In meters
body_offset_z = 0       # In meters

# Enable using yaw from compass to align north (zero degree is facing north)
compass_enabled = 0

# TODO: Explain this transformation by visualization
# Transformation to convert different camera orientations to NED convention
# Frontfacing:
#     Forward, USB port to the right (default): 
#           H_aeroRef_T265Ref = np.array([[0,0,-1,0],[1,0,0,0],[0,-1,0,0],[0,0,0,1]])
#           H_T265body_aeroBody = np.linalg.inv(H_aeroRef_T265Ref)
#     Forward, USB port to the left: 
# Downfacing (you need to tilt the vehicle's nose up a little (not flat) when launch the T265 realsense-ros node, otherwise the initial yaw will be randomized, read here: https://github.com/IntelRealSense/librealsense/issues/4080
# Tilt the vehicle to any other sides and the yaw might not be as stable):
#     Downfacing, USB port to the right : 
#           H_aeroRef_T265Ref = np.array([[0,1, 0,0],[1,0,0,0],[0,0,-1,0],[0,0,0,1]]), 
#           H_T265body_aeroBody = np.linalg.inv(np.array([[0,0,-1,0],[1,0,0,0],[0,-1,0,0],[0,0,0,1]]))
#     Downfacing, USB port to the left  : 
#     Downfacing, USB port to the back  :         
#     Downfacing, USB port to the front : 
H_aeroRef_T265Ref = np.array([[0,0,-1,0],[1,0,0,0],[0,-1,0,0],[0,0,0,1]])
H_T265body_aeroBody = np.linalg.inv(H_aeroRef_T265Ref)

# Default global position of home/ origin
home_lat = 151269321       # Somewhere in Africa
home_lon = 16624301        # Somewhere in Africa
home_alt = 163000 

vehicle = None
pipe = None

# pose data confidence: 0x0 - Failed / 0x1 - Low / 0x2 - Medium / 0x3 - High 
pose_data_confidence_level = ('Failed', 'Low', 'Medium', 'High')

#######################################
# Parsing user' inputs
#######################################

parser = argparse.ArgumentParser(description='Reboots vehicle')
parser.add_argument('--connect',
                    help="Vehicle connection target string. If not specified, a default string will be used.")
parser.add_argument('--baudrate', type=float,
                    help="Vehicle connection baudrate. If not specified, a default value will be used.")
parser.add_argument('--vision_msg_hz', type=float,
                    help="Update frequency for VISION_POSITION_ESTIMATE message. If not specified, a default value will be used.")
parser.add_argument('--confidence_msg_hz', type=float,
                    help="Update frequency for confidence level. If not specified, a default value will be used.")

args = parser.parse_args()

connection_string = args.connect
connection_baudrate = args.baudrate
vision_msg_hz = args.vision_msg_hz
confidence_msg_hz = args.confidence_msg_hz

# Using default values if no specified inputs
if not connection_string:
    connection_string = connection_string_default
    print("INFO: Using default connection_string", connection_string)
else:
    print("INFO: Using connection_string", connection_string)

if not connection_baudrate:
    connection_baudrate = connection_baudrate_default
    print("INFO: Using default connection_baudrate", connection_baudrate)
else:
    print("INFO: Using connection_baudrate", connection_baudrate)

if not vision_msg_hz:
    vision_msg_hz = vision_msg_hz_default
    print("INFO: Using default vision_msg_hz", vision_msg_hz)
else:
    print("INFO: Using vision_msg_hz", vision_msg_hz)
    
if not confidence_msg_hz:
    confidence_msg_hz = confidence_msg_hz_default
    print("INFO: Using default confidence_msg_hz", confidence_msg_hz)
else:
    print("INFO: Using confidence_msg_hz", confidence_msg_hz)

if body_offset_enabled == 1:
    print("INFO: Using camera position offset: Enabled, x y z is", body_offset_x, body_offset_y, body_offset_z)
else:
    print("INFO: Using camera position offset: Disabled")

if compass_enabled == 1:
    print("INFO: Using compass: Enabled. Heading will be aligned to north.")
else:
    print("INFO: Using compass: Disabled")

#######################################
# Functions
#######################################

# https://mavlink.io/en/messages/common.html#VISION_POSITION_ESTIMATE
def send_vision_position_message():
    global current_time, H_aeroRef_aeroBody

    if H_aeroRef_aeroBody is not None:
        rpy_rad = np.array( tf.euler_from_matrix(H_aeroRef_aeroBody, 'sxyz'))

        msg = vehicle.message_factory.vision_position_estimate_encode(
            current_time,       #us	Timestamp (UNIX time or time since system boot)
            H_aeroRef_aeroBody[0][3],	                #Global X position
            H_aeroRef_aeroBody[1][3],                  #Global Y position
            H_aeroRef_aeroBody[2][3],	                #Global Z position
            rpy_rad[0],	            #Roll angle
            rpy_rad[1],	            #Pitch angle
            rpy_rad[2]	                #Yaw angle
        )

        vehicle.send_mavlink(msg)
        vehicle.flush()

# For a lack of a dedicated message, we pack the confidence level into a message that will not be used, so we can view it on GCS
# Confidence level value: 0 - 3, remapped to 0 - 100: 0% - Failed / 33.3% - Low / 66.6% - Medium / 100% - High 
def send_confidence_level_dummy_message():
    global data
    if data is not None:
        print("INFO: Tracker confidence: ", pose_data_confidence_level[data.tracker_confidence])

        msg = vehicle.message_factory.vision_position_delta_encode(
            0,	            #us	Timestamp (UNIX time or time since system boot)
            0,	            #Time since last reported camera frame
            [0, 0, 0],      #angle_delta
            [0, 0, 0],      #position_delta
            float(data.tracker_confidence * 100 / 3)          
        )

        vehicle.send_mavlink(msg)
        vehicle.flush()

# Send a mavlink SET_GPS_GLOBAL_ORIGIN message (http://mavlink.org/messages/common#SET_GPS_GLOBAL_ORIGIN), which allows us to use local position information without a GPS.
def set_default_global_origin():
    msg = vehicle.message_factory.set_gps_global_origin_encode(
        int(vehicle._master.source_system),
        home_lat, 
        home_lon,
        home_alt
    )

    vehicle.send_mavlink(msg)
    vehicle.flush()

# Send a mavlink SET_HOME_POSITION message (http://mavlink.org/messages/common#SET_HOME_POSITION), which allows us to use local position information without a GPS.
def set_default_home_position():
    x = 0
    y = 0
    z = 0
    q = [1, 0, 0, 0]   # w x y z

    approach_x = 0
    approach_y = 0
    approach_z = 1

    msg = vehicle.message_factory.set_home_position_encode(
        int(vehicle._master.source_system),
        home_lat, 
        home_lon,
        home_alt,
        x,
        y,
        z,
        q,
        approach_x,
        approach_y,
        approach_z
    )

    vehicle.send_mavlink(msg)
    vehicle.flush()

# Request a timesync update from the flight controller, for future work.
# TODO: Inspect the usage of timesync_update 
def update_timesync(ts=0, tc=0):
    if ts == 0:
        ts = int(round(time.time() * 1000))
    msg = vehicle.message_factory.timesync_encode(
        tc,     # tc1
        ts      # ts1
    )
    vehicle.send_mavlink(msg)
    vehicle.flush()

# Listen to "GPS Glitch" and "GPS Glitch cleared" message, then set EKF home automatically.
def statustext_callback(self, attr_name, value):
    # These are the status texts that indicates EKF is ready to receive home position
    if value.text == "GPS Glitch" or value.text == "GPS Glitch cleared" or value.text == "EKF2 IMU1 ext nav yaw alignment complete":
        time.sleep(0.1)
        print("INFO: Set EKF home with default GPS location")
        set_default_global_origin()
        set_default_home_position()

# Listen to attitude data to acquire initial heading when compass is available
def att_msg_callback(self, attr_name, value):
    global heading_north_yaw
    if heading_north_yaw is None:
        heading_north_yaw = value.yaw
        print("INFO: Received first ATTITUDE message with heading yaw", heading_north_yaw * 180 / m.pi, "degrees")
    else:
        heading_north_yaw = value.yaw
        print("INFO: Received ATTITUDE message with heading yaw", heading_north_yaw * 180 / m.pi, "degrees")

def vehicle_connect():
    global vehicle
    
    try:
        vehicle = connect(connection_string, wait_ready = True, baud = connection_baudrate)
    except:
        print('Connection error! Retrying...')

    if vehicle == None:
        return False
    else:
        return True

def realsense_connect():
    global pipe
    # Declare RealSense pipeline, encapsulating the actual device and sensors
    pipe = rs.pipeline()

    # Build config object before requesting data
    cfg = rs.config()

    # Enable the stream we are interested in
    cfg.enable_stream(rs.stream.pose) # Positional data 

    # Start streaming with requested config
    pipe.start(cfg)

#######################################
# Main code starts here
#######################################

print("INFO: Connecting to Realsense camera.")
realsense_connect()
print("INFO: Realsense connected.")

print("INFO: Connecting to vehicle.")
while (not vehicle_connect()):
    pass
print("INFO: Vehicle connected.")

# Listen to the mavlink messages that will be used as trigger to set EKF home automatically
vehicle.add_message_listener('STATUSTEXT', statustext_callback)

if compass_enabled == 1:
    # Listen to the attitude data in aeronautical frame
    vehicle.add_message_listener('ATTITUDE', att_msg_callback)

data = None
H_aeroRef_aeroBody = None
heading_north_yaw = None

# Send MAVlink messages in the background
sched = BackgroundScheduler()

sched.add_job(send_vision_position_message, 'interval', seconds = 1/vision_msg_hz)
sched.add_job(send_confidence_level_dummy_message, 'interval', seconds = 1/confidence_msg_hz)

sched.start()

if compass_enabled == 1:
    # Wait a short while for yaw to be correctly initiated
    time.sleep(1)

print("INFO: Sending VISION_POSITION_ESTIMATE messages to FCU.")

try:
    while True:
        # Wait for the next set of frames from the camera
        frames = pipe.wait_for_frames()

        # Fetch pose frame
        pose = frames.get_pose_frame()

        if pose:
            # Store the timestamp for MAVLink messages
            current_time = int(round(time.time() * 1000000))

            # Pose data consists of translation and rotation
            data = pose.get_pose_data()
            
            # In transformations, Quaternions w+ix+jy+kz are represented as [w, x, y, z]!
            H_T265Ref_T265body = tf.quaternion_matrix([data.rotation.w, data.rotation.x, data.rotation.y, data.rotation.z]) 
            H_T265Ref_T265body[0][3] = data.translation.x
            H_T265Ref_T265body[1][3] = data.translation.y
            H_T265Ref_T265body[2][3] = data.translation.z

            # Transform to aeronautic coordinates (body AND reference frame!)
            H_aeroRef_aeroBody = H_aeroRef_T265Ref.dot( H_T265Ref_T265body.dot( H_T265body_aeroBody))

            # Take offsets from body's center of gravity (or IMU) to camera's origin into account
            if body_offset_enabled == 1:
                H_body_camera = tf.euler_matrix(0, 0, 0, 'sxyz')
                H_body_camera[0][3] = body_offset_x
                H_body_camera[1][3] = body_offset_y
                H_body_camera[2][3] = body_offset_z
                H_camera_body = np.linalg.inv(H_body_camera)
                H_aeroRef_aeroBody = H_body_camera.dot(H_aeroRef_aeroBody.dot(H_camera_body))

            # Realign heading to face north using initial compass data
            if compass_enabled == 1:
                H_aeroRef_aeroBody = H_aeroRef_aeroBody.dot( tf.euler_matrix(0, 0, heading_north_yaw, 'sxyz'))

except KeyboardInterrupt:
    print("INFO: KeyboardInterrupt has been caught. Cleaning up...")               

finally:
    pipe.stop()
    vehicle.close()
    print("INFO: Realsense pipeline and vehicle object closed.")
