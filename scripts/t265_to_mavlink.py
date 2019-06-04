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

from dronekit import connect, VehicleMode

# Enable printing debug messages
ENABLE_DEBUG_MSG = 0

# Global position of the origin
lat = 13669820    # Terni 425633500 
lon = 1036634300      # Terni  
alt = 163000 

# For forward-facing camera (with X to the right):  H_aeroRef_T265Ref = np.array([[0,0,-1,0],[1,0,0,0],[0,-1,0,0],[0,0,0,1]])
# For down-facing camera (with X to the right):     H_aeroRef_T265Ref = np.array([[0,1, 0,0],[1,0,0,0],[0,0,-1,0],[0,0,0,1]])
# TODO: Explain this transformation with visual example
H_aeroRef_T265Ref = np.array([[0,0,-1,0],[1,0,0,0],[0,-1,0,0],[0,0,0,1]])
H_T265body_aeroBody = np.linalg.inv(H_aeroRef_T265Ref)

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
def set_fake_global_origin():
    msg = vehicle.message_factory.set_gps_global_origin_encode(
        int(vehicle._master.source_system),
        lat, 
        lon,
        alt
    )
    vehicle.send_mavlink(msg)
    vehicle.flush()

# Send a mavlink SET_HOME_POSITION message (http://mavlink.org/messages/common#SET_HOME_POSITION), which should allow us to use local position information without a GPS
def set_fake_home_position():
    x = 0
    y = 0
    z = 0
    q = [1, 0, 0, 0]   # w x y z

    approach_x = 0
    approach_y = 0
    approach_z = 1

    msg = vehicle.message_factory.set_home_position_encode(
        int(vehicle._master.source_system),
        lat, 
        lon,
        alt,
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

# Request a timesync update from the flight controller
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


#######################################
# Main code starts here
#######################################

# Connect to the Vehicle.
print("\nConnecting to vehicle")
vehicle = connect('/dev/ttyUSB0', wait_ready=True, baud=921600)
print("\nFCU Connected")

# Declare RealSense pipeline, encapsulating the actual device and sensors
pipe = rs.pipeline()

print("\nRealsense connected")
# Build config object and request pose data
cfg = rs.config()

# Enable the stream we are interested in
cfg.enable_stream(rs.stream.pose) # Positional data 

# Start streaming with requested config
pipe.start(cfg)

print("\nSending vision pose messages to FCU through MAVLink NOW\n")

set_home_repeat = 0

try:
    while True:
        # Wait for the next set of frames from the camera
        frames = pipe.wait_for_frames()

        # Fetch pose frame
        pose = frames.get_pose_frame()

        if pose:
            # Store the timestamp for MAVLink messages
            current_time = int(round(time.time() * 1000000))

            # Print some of the pose data to the terminal
            data = pose.get_pose_data()
            
            # In transformations, Quaternions w+ix+jy+kz are represented as [w, x, y, z]!
            H_T265Ref_T265body = tf.quaternion_matrix([data.rotation.w, data.rotation.x, data.rotation.y, data.rotation.z]) 

            # Transform to aeronautic coordinates (body AND reference frame!)
            H_aeroRef_aeroBody = H_aeroRef_T265Ref.dot( H_T265Ref_T265body.dot( H_T265body_aeroBody ))
            
            # 'sxyz': Rz(yaw)*Ry(pitch)*Rx(roll) body w.r.t. reference frame
            rpy_rad = np.array( tf.euler_from_matrix(H_aeroRef_aeroBody, 'sxyz'))

            # Send MAVLINK VISION_POSITION_MESSAGE to FUC 
            send_vision_position_message(-data.translation.z, data.translation.x, -data.translation.y, rpy_rad[0], rpy_rad[1], rpy_rad[2])

            if ENABLE_DEBUG_MSG:
                rpy_deg = rpy_rad * 180 / m.pi
                print("Frame #{}".format(pose.frame_number), "RPY [deg]: {}".format(rpy_deg))

            # Skip some messages
            time.sleep(0.05)

            # Set fake home position through MAVLink
            # TODO: For sure there can be a more methodological approach
            if set_home_repeat == 100 or set_home_repeat == 200:
                print("\nAttempt to set home origin")
                set_fake_global_origin()
                set_fake_home_position()

            set_home_repeat = set_home_repeat + 1
                
finally:
    pipe.stop()

    #Close vehicle object before exiting script
    print("\nClose vehicle object")
    vehicle.close()
