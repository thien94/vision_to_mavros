#!/usr/bin/env python3

######################################################
##
##  Testing OBSTACLE_DISTANCE messages with ArduPilot and Mission Planner
##
######################################################

# Set MAVLink protocol to 2.
import os
os.environ["MAVLINK20"] = "1"

# Import the libraries
import sys
import numpy as np
import time
import argparse
import threading

from time import sleep
from apscheduler.schedulers.background import BackgroundScheduler
from dronekit import connect

######################################################
##  Reconfigurable parameters                       ##
######################################################

# Default configurations for connection to the FCU
connection_string_default = '/dev/ttyUSB0'
connection_baudrate_default = 921600

# Enable/disable each message/function individually
enable_msg_obstacle_distance = True
enable_msg_distance_sensor = False
obstacle_distance_msg_hz = 15.0

# lock for thread synchronization
lock = threading.Lock()

# FCU connection variables
vehicle = None
is_vehicle_connected = False

######################################################
##  Parsing user' inputs                            ##
######################################################

parser = argparse.ArgumentParser(description='Reboots vehicle')
parser.add_argument('--connect',
                    help="Vehicle connection target string. If not specified, a default string will be used.")
parser.add_argument('--baudrate', type=float,
                    help="Vehicle connection baudrate. If not specified, a default value will be used.")

args = parser.parse_args()

connection_string = args.connect
connection_baudrate = args.baudrate

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

######################################################
##  Functions - MAVLink                             ##
######################################################

# https://mavlink.io/en/messages/common.html#OBSTACLE_DISTANCE
def send_obstacle_distance_message():
    #
    # Set the parameters for obstacle distance here
    #
    # [0]    [35]   [71]    <- Output: distances[72]
    #  |      |      |      <- step = width / 72
    #  ---------------      <- horizontal line
    #  \      |      /
    #   \     |     /
    #    \    |    /
    #  ^  \   |   /  ^
    #  |   \  |  /   |
    #start  \ | /   end
    #       Camera          <- Input: depth image, obtained from depth camera (implemented in d4xx_to_mavlink.py)
    #
    
    angle_start = -39.5     # -FOV/2
    angle_end = 39.5        # 39.5 - real camera (2 arcs), <= 69.0: 2 arcs, > 70.0: 3 arcs 

    FOV = angle_end - angle_start
    angle_offset = angle_start
    distances_array_length = 72
    increment_f = FOV / distances_array_length

    min_dist_cm = 10
    max_dist_cm = 800
    cur_dist_cm = 200

    # Setup the distances array with the same value (cur_dist_cm)
    distances = np.ones((distances_array_length,), dtype=np.uint16) * cur_dist_cm

    current_time_us = int(round(time.time() * 1000000))

    msg = vehicle.message_factory.obstacle_distance_encode(
        current_time_us,    # us Timestamp (UNIX time or time since system boot)
        0,                  # sensor_type, defined here: https://mavlink.io/en/messages/common.html#MAV_DISTANCE_SENSOR
        distances,          # distances,    uint16_t[72],   cm
        0,                  # increment,    uint8_t,        deg
        min_dist_cm,	    # min_distance, uint16_t,       cm
        max_dist_cm,        # max_distance, uint16_t,       cm
        increment_f,	    # increment_f,  float,          deg
        angle_offset,       # angle_offset, float,          deg
        12                  # MAV_FRAME, vehicle-front aligned: https://mavlink.io/en/messages/common.html#MAV_FRAME_BODY_FRD    
    )

    vehicle.send_mavlink(msg)
    vehicle.flush()


# https://mavlink.io/en/messages/common.html#DISTANCE_SENSOR
def send_distance_sensor_message():
    # Use this to rotate all processed data
    camera_facing_angle_degree = 0
    orientation = int(camera_facing_angle_degree / 45)
    
    min_dist_cm = 10
    max_dist_cm = 800
    curr_dist_cm = 100
    current_time_ms = int(round(time.time() * 1000))

    msg = vehicle.message_factory.distance_sensor_encode(
        current_time_ms,# ms Timestamp (UNIX time or time since system boot) (ignored)
        min_dist_cm,    # min_distance, uint16_t, cm
        max_dist_cm,    # min_distance, uint16_t, cm
        curr_dist_cm,   # current_distance,	uint16_t, cm	
        0,	            # type : 0 (ignored)
        0,              # id : 0 (ignored)
        orientation,    # orientation
        0               # covariance : 0 (ignored)
    )

    vehicle.send_mavlink(msg)
    vehicle.flush()

def send_msg_to_gcs(text_to_be_sent):
    # MAV_SEVERITY: 0=EMERGENCY 1=ALERT 2=CRITICAL 3=ERROR, 4=WARNING, 5=NOTICE, 6=INFO, 7=DEBUG, 8=ENUM_END
    # Defined here: https://mavlink.io/en/messages/common.html#MAV_SEVERITY
    # MAV_SEVERITY = 3 will let the message be displayed on Mission Planner HUD, but 6 is ok for QGroundControl
    if is_vehicle_connected == True:
        text_msg = 'OA: ' + text_to_be_sent
        status_msg = vehicle.message_factory.statustext_encode(
            6,                      # MAV_SEVERITY
            text_msg.encode()	    # max size is char[50]       
        )
        vehicle.send_mavlink(status_msg)
        vehicle.flush()
        print("INFO: " + text_to_be_sent)
    else:
        print("INFO: Vehicle not connected. Cannot send text message to Ground Control Station (GCS)")

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

# Establish connection to the FCU
def vehicle_connect():
    global vehicle, is_vehicle_connected

    if vehicle == None:
        try:
            vehicle = connect(connection_string, wait_ready = True, baud = connection_baudrate, source_system = 1)
        except Exception as e:
            print(e)
            sleep(1)
        except:
            print('Connection error! Retrying...')
            sleep(1)

    if vehicle == None:
        is_vehicle_connected = False
        return False
    else:
        is_vehicle_connected = True
        return True

######################################################
##  Main code starts here                           ##
######################################################

print("INFO: Connecting to vehicle.")
while (not vehicle_connect()):
    pass
print("INFO: Vehicle connected.")

# Send MAVlink messages in the background at pre-determined frequencies
sched = BackgroundScheduler()

if enable_msg_obstacle_distance:
    sched.add_job(send_obstacle_distance_message, 'interval', seconds = 1/obstacle_distance_msg_hz)
    send_msg_to_gcs('Sending obstacle distance messages to FCU')
elif enable_msg_distance_sensor:
    sched.add_job(send_distance_sensor_message, 'interval', seconds = 1/obstacle_distance_msg_hz)
    send_msg_to_gcs('Sending distance sensor messages to FCU')
else:
    send_msg_to_gcs('Nothing to do. Check params to enable something')
    vehicle.close()
    print("INFO: Realsense pipe and vehicle object closed.")
    sys.exit()

sched.start()

try:
    while True:
        pass

except KeyboardInterrupt:
    send_msg_to_gcs('Closing the script...')  

except Exception as e:
    print(e)
    pass

except:
    send_msg_to_gcs('ERROR: Depth camera disconnected')  

finally:
    vehicle.close()
    sys.exit()
