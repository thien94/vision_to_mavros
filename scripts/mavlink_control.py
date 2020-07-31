#!/usr/bin/env python3
# -*- coding: utf-8 -*-

######################################################
##  Sending control commands to AP via MAVLink      ##
##  Based on set_attitude_target.py: https://github.com/dronekit/dronekit-python/blob/master/examples/set_attitude_target/set_attitude_target.py
######################################################

## Additional installation for SITL:
##      pip3 install dronekit-sitl -UI

from dronekit import connect, VehicleMode, LocationGlobal, LocationGlobalRelative
from pymavlink import mavutil # Needed for command message definitions
import time
import math

# Set MAVLink protocol to 2.
import os
os.environ["MAVLINK20"] = "1"

import sys

#######################################
# Parameters
#######################################

rc_control_channel = 6     # Channel to check value, start at 0 == chan1_raw
rc_control_thres = 2000    # Values to check

#######################################
# Global variables
#######################################

rc_channel_value = 0
vehicle = None

#######################################
# User input
#######################################

# Set up option parsing to get connection string
import argparse  
parser = argparse.ArgumentParser(description='Example showing how to set and clear vehicle channel-override information.')
parser.add_argument('--connect', 
                   help="vehicle connection target string. If not specified, SITL automatically started and used.")
args = parser.parse_args()

#######################################
# Functions
#######################################

connection_string = args.connect
sitl = None

# Start SITL if no connection string specified
if not connection_string:
    import dronekit_sitl
    sitl = dronekit_sitl.start_default()
    connection_string = sitl.connection_string()

print('Connecting to vehicle on: %s' % connection_string)
vehicle = connect(connection_string, wait_ready=True)

@vehicle.on_message('RC_CHANNELS')
def RC_CHANNEL_listener(vehicle, name, message):
    global rc_channel_value, rc_control_channel
    
    # TO-DO: find a less hard-coded solution
    curr_channels_values = [message.chan1_raw, message.chan2_raw, message.chan3_raw, message.chan4_raw, message.chan5_raw, message.chan6_raw, message.chan7_raw, message.chan8_raw]

    rc_channel_value = curr_channels_values[rc_control_channel]

    # # Print out the values to debug
    # print('%s attribute is: %s' % (name, message)) # Print all info from the messages
    # os.system('clear') # This helps in displaying the messages to be more readable
    # for channel in range(8):
    #     print("Number of RC channels: ", message.chancount, ". Individual RC channel value:")
    #     print(" CH", channel, curr_channels_values[channel])

def arm_and_takeoff_nogps(aTargetAltitude):
    """
    Arms vehicle and fly to aTargetAltitude without GPS data.
    """

    print("Basic pre-arm checks")

    # Don't let the user try to arm until autopilot is ready
    # If you need to disable the arming check,
    # just comment it with your own responsibility.
    while not vehicle.is_armable:
        print("- Waiting for vehicle to initialise...")
        time.sleep(1)

    print("Arming motors")
    # Copter should arm in GUIDED_NOGPS mode
    vehicle.mode = VehicleMode("GUIDED_NOGPS")
    vehicle.armed = True
    while not vehicle.armed:
        print("- Waiting for arming...")
        time.sleep(1)

    print("Taking off!")
    vehicle.simple_takeoff(aTargetAltitude)  # Take off to target altitude

    # Wait until the vehicle reaches a safe height before processing the goto
    #  (otherwise the command after Vehicle.simple_takeoff will execute
    #   immediately).
    while True:
        print(" Altitude: ", vehicle.location.global_relative_frame.alt)
        # Break and return from function just below target altitude.
        if vehicle.location.global_relative_frame.alt >= aTargetAltitude * 0.95:
            print("Reached target altitude")
            break
        time.sleep(1)

def send_attitude_target(roll_angle = 0.0, pitch_angle = 0.0,
                         yaw_angle = None, yaw_rate = 0.0, use_yaw_rate = False,
                         thrust = 0.5):
    """
    use_yaw_rate: the yaw can be controlled using yaw_angle OR yaw_rate.
                  When one is used, the other is ignored by Ardupilot.
    thrust: 0 <= thrust <= 1, as a fraction of maximum vertical thrust.
            Note that as of Copter 3.5, thrust = 0.5 triggers a special case in
            the code for maintaining current altitude.
    """
    if yaw_angle is None:
        # this value may be unused by the vehicle, depending on use_yaw_rate
        yaw_angle = vehicle.attitude.yaw
    # Thrust >  0.5: Ascend
    # Thrust == 0.5: Hold the altitude
    # Thrust <  0.5: Descend
    msg = vehicle.message_factory.set_attitude_target_encode(
        0, # time_boot_ms
        1, # Target system
        1, # Target component
        0b00000000 if use_yaw_rate else 0b00000100,
        to_quaternion(roll_angle, pitch_angle, yaw_angle), # Quaternion
        0, # Body roll rate in radian
        0, # Body pitch rate in radian
        math.radians(yaw_rate), # Body yaw rate in radian/second
        thrust  # Thrust
    )
    vehicle.send_mavlink(msg)

def goto_position_target_local_ned(north, east, down):
    """
    Send SET_POSITION_TARGET_LOCAL_NED command to request the vehicle fly to a specified
    location in the North, East, Down frame.
    """
    msg = vehicle.message_factory.set_position_target_local_ned_encode(
        0,       # time_boot_ms (not used)
        0, 0,    # target system, target component
        mavutil.mavlink.MAV_FRAME_LOCAL_NED, # frame
        0b0000111111111000, # type_mask (only positions enabled)
        north, east, down,
        0, 0, 0, # x, y, z velocity in m/s  (not used)
        0, 0, 0, # x, y, z acceleration (not supported yet, ignored in GCS_Mavlink)
        0, 0)    # yaw, yaw_rate (not supported yet, ignored in GCS_Mavlink)
    # send command to vehicle
    vehicle.send_mavlink(msg)

def set_attitude(roll_angle = 0.0, pitch_angle = 0.0,
                 yaw_angle = None, yaw_rate = 0.0, use_yaw_rate = False,
                 thrust = 0.5, duration = 0):
    """
    Note that from AC3.3 the message should be re-sent more often than every
    second, as an ATTITUDE_TARGET order has a timeout of 1s.
    In AC3.2.1 and earlier the specified attitude persists until it is canceled.
    The code below should work on either version.
    Sending the message multiple times is the recommended way.
    """
    send_attitude_target(roll_angle, pitch_angle,
                         yaw_angle, yaw_rate, False,
                         thrust)
    start = time.time()
    while time.time() - start < duration:
        send_attitude_target(roll_angle, pitch_angle,
                             yaw_angle, yaw_rate, False,
                             thrust)
        time.sleep(0.1)
    # Reset attitude, or it will persist for 1s more due to the timeout
    send_attitude_target(0, 0,
                         0, 0, True,
                         thrust)

def to_quaternion(roll = 0.0, pitch = 0.0, yaw = 0.0):
    """
    Convert degrees to quaternions
    """
    t0 = math.cos(math.radians(yaw * 0.5))
    t1 = math.sin(math.radians(yaw * 0.5))
    t2 = math.cos(math.radians(roll * 0.5))
    t3 = math.sin(math.radians(roll * 0.5))
    t4 = math.cos(math.radians(pitch * 0.5))
    t5 = math.sin(math.radians(pitch * 0.5))

    w = t0 * t2 * t4 + t1 * t3 * t5
    x = t0 * t3 * t4 - t1 * t2 * t5
    y = t0 * t2 * t5 + t1 * t3 * t4
    z = t1 * t2 * t4 - t0 * t3 * t5

    return [w, x, y, z]

#######################################
# Main program starts here
#######################################

try:
    # Wait until the RC channel is turned on and the corresponding channel is switch
    if sitl is None:
        while rc_channel_value < rc_control_thres:
            print("Checking rc channel:", rc_control_channel, ", current value:", rc_channel_value, ", threshold to start: ", rc_control_thres)
            time.sleep(1)
    print("Starting autonomous control...")

    # Take off in GUIDED_NOGPS mode.
    if sitl is not None:
        arm_and_takeoff_nogps(20)
        print("Hold position for 3 seconds")
        set_attitude(duration = 3)
    
    while vehicle.mode.name=="GUIDED": #Stop action if we are no longer in guided mode.

        print("SQUARE path using SET_POSITION_TARGET_LOCAL_NED and position parameters")
        DURATION_SEC = 10 #Set duration for each segment.
        HEIGHT_M = 2
        SIZE_M  = 5

        """
        Fly the vehicle in a SIZE_M meter square path, using the SET_POSITION_TARGET_LOCAL_NED command 
        and specifying a target position (rather than controlling movement using velocity vectors). 
        The command is called from goto_position_target_local_ned() (via `goto`).

        The position is specified in terms of the NED (North East Down) relative to the Home location.

        WARNING: The "D" in NED means "Down". Using a positive D value will drive the vehicle into the ground!

        The code sleeps for a time (DURATION_SEC) to give the vehicle time to reach each position (rather than 
        sending commands based on proximity).

        The code also sets the region of interest (MAV_CMD_DO_SET_ROI) via the `set_roi()` method. This points the 
        camera gimbal at the the selected location (in this case it aligns the whole vehicle to point at the ROI).
        """	

        print("North (m): ", SIZE_M, ", East (m): 0m, Height (m):", HEIGHT_M," for", DURATION_SEC, "seconds")
        goto_position_target_local_ned(SIZE_M, 0, -HEIGHT_M)
        time.sleep(DURATION_SEC)

        print("North (m): ", SIZE_M, ", East (m): ", SIZE_M, " Height (m):", HEIGHT_M," for", DURATION_SEC, "seconds")
        goto_position_target_local_ned(SIZE_M, SIZE_M, -HEIGHT_M)
        time.sleep(DURATION_SEC)

        print("North (m): 0m, East (m): 0m ", SIZE_M, ", Height (m):", HEIGHT_M," for", DURATION_SEC, "seconds")
        goto_position_target_local_ned(0, SIZE_M, -HEIGHT_M)
        time.sleep(DURATION_SEC)

        print("North (m): 0m, East (m): 0m, Height (m):", HEIGHT_M," for", DURATION_SEC, "seconds")
        goto_position_target_local_ned(0, 0, -HEIGHT_M)
        time.sleep(DURATION_SEC)


    print("Setting LAND mode...")
    vehicle.mode = VehicleMode("LAND")
    time.sleep(1)

    # Close vehicle object before exiting script
    print("Close vehicle object")
    vehicle.close()

    # Shut down simulator if it was started.
    if sitl is not None:
        sitl.stop()

    print("Completed")

except KeyboardInterrupt:
    vehicle.close()
    print("Vehicle object closed.")
    sys.exit()
