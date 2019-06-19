#####################################################
##          librealsense T265 to MAVLink           ##
#####################################################
# This script assumes pyrealsense2.[].so file is found under the same directory as this script
# Install required packages: 
#   pip install pyrealsense2
#   pip install transformations
#   pip install dronekit

# First import the libraries
import pyrealsense2 as rs
import numpy as np
import transformations as tf
import math as m
import time
import argparse
from dronekit import connect, VehicleMode

#######################################
# Parameters
#######################################

# Default configurations for connection to the FCU
connection_string_default = '/dev/ttyUSB0'
connection_baudrate_default = 921600
vision_msg_hz_default = 30

# Default global position of home/ origin
home_lat = 151269321       # Somewhere in Africa
home_lon = 16624301        # Somewhere in Africa
home_alt = 163000 

# Transformation to convert different camera orientations to NED convention
#   For forward-facing camera (with X to the right):  H_aeroRef_T265Ref = np.array([[0,0,-1,0],[1,0,0,0],[0,-1,0,0],[0,0,0,1]])
#   For down-facing camera (with X to the right):     H_aeroRef_T265Ref = np.array([[0,1, 0,0],[1,0,0,0],[0,0,-1,0],[0,0,0,1]])
# TODO: Explain this transformation by visualization
H_aeroRef_T265Ref = np.array([[0,0,-1,0],[1,0,0,0],[0,-1,0,0],[0,0,0,1]])
H_T265body_aeroBody = np.linalg.inv(H_aeroRef_T265Ref)

vehicle = None
pipe = None

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

args = parser.parse_args()

connection_string = args.connect
connection_baudrate = args.baudrate
vision_msg_hz = args.vision_msg_hz

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

#######################################
# Functions
#######################################

# https://mavlink.io/en/messages/common.html#VISION_POSITION_ESTIMATE
def send_vision_position_message(x,y,z,roll,pitch,yaw):
    msg = vehicle.message_factory.vision_position_estimate_encode(
        current_time,	#us	Timestamp (UNIX time or time since system boot)
        x,	            #Global X position
        y,              #Global Y position
        z,	            #Global Z position
        roll,	        #Roll angle
        pitch,	        #Pitch angle
        yaw	            #Yaw angle
        #0              #covariance :upper right triangle (states: x, y, z, roll, pitch, ya
        #0              #reset_counter:Estimate reset counter. 
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

# Listen to the mavlink messages
vehicle.add_message_listener('STATUSTEXT', statustext_callback)

print("INFO: Sending VISION_POSITION_ESTIMATE messages to FCU through MAVLink")
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
            H_aeroRef_aeroBody = H_aeroRef_T265Ref.dot( H_T265Ref_T265body.dot( H_T265body_aeroBody ))
            
            # 'sxyz': Rz(yaw)*Ry(pitch)*Rx(roll) body w.r.t. reference frame
            rpy_rad = np.array( tf.euler_from_matrix(H_aeroRef_aeroBody, 'sxyz'))

            # Send MAVLINK VISION_POSITION_MESSAGE to FCU
            send_vision_position_message(H_aeroRef_aeroBody[0][3], H_aeroRef_aeroBody[1][3], H_aeroRef_aeroBody[2][3], rpy_rad[0], rpy_rad[1], rpy_rad[2])

            # We don't want to flood the FCU
            time.sleep(1.0/vision_msg_hz)
                
finally:
    pipe.stop()

    #Close vehicle object before exiting script
    print("INFO: Close vehicle object")
    vehicle.close()
