#####################################################
##          librealsense T265 to MAVLink           ##
#####################################################
# This script assumes pyrealsense2.[].so file is found under the same directory as this script
# Install required packages: 
#   pip3 install pyrealsense2
#   pip3 install transformations
#   pip3 install dronekit
#   pip3 install apscheduler

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
confidence_msg_secs_default = 2

# TODO: Explain this transformation by visualization
# Transformation to convert different camera orientations to NED convention
# Frontfacing:
#     Forward, USB port to the right (default): 
#           H_aeroRef_T265Ref = np.array([[0,0,-1,0],[1,0,0,0],[0,-1,0,0],[0,0,0,1]])
#           H_T265body_aeroBody = np.linalg.inv(H_aeroRef_T265Ref)
#     Forward, USB port to the left: 
# Downfacing (you need to tilt the vehicle's nose up a little (not flat) when launch the T265 realsense-ros node, otherwise the initial yaw will be randomized, read here: https://github.com/IntelRealSense/librealsense/issues/4080
# Tilt the vehicle to any other sides and the yaw might not be as stable):
#     Downfacing, USB port to the right : H_aeroRef_T265Ref = np.array([[0,1, 0,0],[1,0,0,0],[0,0,-1,0],[0,0,0,1]])
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
# Connection configurations
#######################################

parser = argparse.ArgumentParser(description='Reboots vehicle')
parser.add_argument('--connect',
                    help="Vehicle connection target string. If not specified, a default string will be used.")
parser.add_argument('--baudrate',
                    help="Vehicle connection baudrate. If not specified, a default value will be used.")
parser.add_argument('--vision_msg_hz',
                    help="Update frequency for VISION_POSITION_ESTIMATE message. If not specified, a default value will be used.")
parser.add_argument('--confidence_msg_secs',
                    help="Update frequency for confidence level. If not specified, a default value will be used.")

args = parser.parse_args()

connection_string = args.connect
connection_baudrate = args.baudrate
vision_msg_hz = args.vision_msg_hz
confidence_msg_secs = args.confidence_msg_secs

# Using default values if no specified inputs
if not connection_string:
    connection_string = connection_string_default
    print("INFO: Using default connection_string", connection_string)

if not connection_baudrate:
    connection_baudrate = connection_baudrate_default
    print("INFO: Using default connection_baudrate", connection_baudrate_default)

if not vision_msg_hz:
    vision_msg_hz = vision_msg_hz_default
    print("INFO: Using default vision_msg_hz", vision_msg_hz)
    
if not confidence_msg_secs:
    confidence_msg_secs = confidence_msg_secs_default
    print("INFO: Using default confidence_msg_secs", confidence_msg_secs)

#######################################
# Functions
#######################################

# https://mavlink.io/en/messages/common.html#VISION_POSITION_ESTIMATE
def send_vision_position_message(timestamp_us, x, y, z, roll, pitch, yaw):
    msg = vehicle.message_factory.vision_position_estimate_encode(
        timestamp_us,       #us	Timestamp (UNIX time or time since system boot)
        x,	                #Global X position
        y,                  #Global Y position
        z,	                #Global Z position
        roll,	            #Roll angle
        pitch,	            #Pitch angle
        yaw	                #Yaw angle
    )
    vehicle.send_mavlink(msg)
    vehicle.flush()

# Pack the confidence level into a message that is not being process and send to FCU, so we can view it on GCS
# Confidence level value: 0 - 3, remapped to 0 - 100: 0% - Failed / 33.3% - Low / 66.6% - Medium / 100% - High 
def send_confidence_level_dummy_message():
    global data
    if data != None:
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
    # print("INFO: Received STATUSTEXT message")
    # print(value.text)
    if value.text == "GPS Glitch" or value.text == "GPS Glitch cleared":
        time.sleep(0.1)
        print("INFO: Set EKF home with default GPS location")
        set_default_global_origin()
        set_default_home_position()

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

    # Build config object and request pose data
    cfg = rs.config()

    # Enable the stream we are interested in
    cfg.enable_stream(rs.stream.pose) # Positional data 

    # Start streaming with requested config
    pipe.start(cfg)

#######################################
# Main code starts here
#######################################

print("INFO: Connecting to Realsense camera")
realsense_connect()
print("INFO: Realsense connected")

print("INFO: Connecting to vehicle")
while (not vehicle_connect()):
    pass
print("INFO: Vehicle connected")

# Listen to the mavlink messages that will be used as trigger to set EKF home automatically
vehicle.add_message_listener('STATUSTEXT', statustext_callback)

data = None

# Update confidence level in the background
update_confidence_level_scheduler = BackgroundScheduler()
update_confidence_level_scheduler.add_job(send_confidence_level_dummy_message, 'interval', seconds = confidence_msg_secs)
update_confidence_level_scheduler.start()

print("INFO: Sending VISION_POSITION_ESTIMATE messages to FCU")

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
            
            # 'sxyz': Rz(yaw)*Ry(pitch)*Rx(roll) body w.r.t. reference frame
            rpy_rad = np.array( tf.euler_from_matrix(H_aeroRef_aeroBody, 'sxyz'))

            send_vision_position_message(current_time, H_aeroRef_aeroBody[0][3], H_aeroRef_aeroBody[1][3], H_aeroRef_aeroBody[2][3], rpy_rad[0], rpy_rad[1], rpy_rad[2])

            # We don't want to flood the FCU with vision messages
            time.sleep(1.0 / vision_msg_hz)

except KeyboardInterrupt:
    print("INFO: KeyboardInterrupt has been caught. Cleaning up...")               

finally:
    pipe.stop()

    #Close vehicle object before exiting script
    print("INFO: Close vehicle object")
    vehicle.close()
