#!/usr/bin/env python3

#####################################################
##   Precision Landing with T265 using AprilTags   ##
#####################################################
# Install required packages:
#   pip3 install transformations
#   pip3 install dronekit
#   pip3 install apscheduler
# Next, put the apriltags3.py (Python wrapper for apriltag3) in the same folder as this script:
#   cd /path/to/script
#   wget https://raw.githubusercontent.com/duckietown/apriltags3-py/master/apriltags3.py

# Set the path for pyrealsense2.[].so
import sys
sys.path.append("/usr/local/lib/")

# Remove the path to python2 version that is added by ROS, see here https://stackoverflow.com/questions/43019951/after-install-ros-kinetic-cannot-import-opencv
if '/opt/ros/kinetic/lib/python2.7/dist-packages' in sys.path:
    sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages') 

# Set MAVLink protocol to 2.
import os
os.environ["MAVLINK20"] = "1"

# Import the libraries
import pyrealsense2 as rs
import cv2
import numpy as np
import transformations as tf
import math as m
import time
import argparse
import threading

from apscheduler.schedulers.background import BackgroundScheduler
from dronekit import connect, VehicleMode
from pymavlink import mavutil

try:
    import apriltags3 
except ImportError:
    raise ImportError('Please download the Python wrapper for apriltag3 (apriltags3.py) and put it in the same folder as this script or add the directory path to the PYTHONPATH environment variable.')

#######################################
# Parameters for FCU and MAVLink
#######################################

# Default configurations for connection to the FCU
connection_string_default = '/dev/ttyUSB0'
connection_baudrate_default = 921600
vision_msg_hz_default = 20
landing_target_msg_hz_default = 20
confidence_msg_hz_default = 1
camera_orientation_default = 1

# In NED frame, offset from the IMU or the center of gravity to the camera's origin point
body_offset_enabled = 0
body_offset_x = 0.05    # In meters (m), so 0.05 = 5cm
body_offset_y = 0       # In meters (m)
body_offset_z = 0       # In meters (m)

# Global scale factor, position x y z will be scaled up/down by this factor
scale_factor = 1.0

# Enable using yaw from compass to align north (zero degree is facing north)
compass_enabled = 0

# Default global position of home/ origin
home_lat = 151269321       # Somewhere in Africa
home_lon = 16624301        # Somewhere in Africa
home_alt = 163000 

# Timestamp (UNIX Epoch time or time since system boot)
current_time = 0

vehicle = None
pipe = None

# pose data confidence: 0x0 - Failed / 0x1 - Low / 0x2 - Medium / 0x3 - High 
pose_data_confidence_level = ('Failed', 'Low', 'Medium', 'High')

#######################################
# Parsing user' inputs
#######################################

parser = argparse.ArgumentParser(description='ArduPilot + Realsense T265 + AprilTags')
parser.add_argument('--connect',
                    help="Vehicle connection target string. If not specified, a default string will be used.")
parser.add_argument('--baudrate', type=float,
                    help="Vehicle connection baudrate. If not specified, a default value will be used.")
parser.add_argument('--vision_msg_hz', type=float,
                    help="Update frequency for VISION_POSITION_ESTIMATE message. If not specified, a default value will be used.")
parser.add_argument('--landing_target_msg_hz', type=float,
                    help="Update frequency for LANDING_TARGET message. If not specified, a default value will be used.")
parser.add_argument('--confidence_msg_hz', type=float,
                    help="Update frequency for confidence level. If not specified, a default value will be used.")
parser.add_argument('--scale_calib_enable', type=bool,
                    help="Scale calibration. Only run while NOT in flight")
parser.add_argument('--camera_orientation', type=int,
                    help="Configuration for camera orientation. Currently supported: forward, usb port to the right - 0; downward, usb port to the right - 1")
parser.add_argument('--visualization',type=int,
                    help="Enable visualization. Ensure that a monitor is connected")
parser.add_argument('--debug_enable',type=int,
                    help="Enable debug messages on terminal")

args = parser.parse_args()

connection_string = args.connect
connection_baudrate = args.baudrate
vision_msg_hz = args.vision_msg_hz
landing_target_msg_hz = args.landing_target_msg_hz
confidence_msg_hz = args.confidence_msg_hz
scale_calib_enable = args.scale_calib_enable
camera_orientation = args.camera_orientation
visualization = args.visualization
debug_enable = args.debug_enable

# Using default values if no input is provided
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

if not landing_target_msg_hz:
    landing_target_msg_hz = landing_target_msg_hz_default
    print("INFO: Using default landing_target_msg_hz", landing_target_msg_hz)
else:
    print("INFO: Using landing_target_msg_hz", landing_target_msg_hz)

if body_offset_enabled == 1:
    print("INFO: Using camera position offset: Enabled, x y z is", body_offset_x, body_offset_y, body_offset_z)
else:
    print("INFO: Using camera position offset: Disabled")

if compass_enabled == 1:
    print("INFO: Using compass: Enabled. Heading will be aligned to north.")
else:
    print("INFO: Using compass: Disabled")

if scale_calib_enable == True:
    print("\nINFO: SCALE CALIBRATION PROCESS. DO NOT RUN DURING FLIGHT.\nINFO: TYPE IN NEW SCALE IN FLOATING POINT FORMAT\n")
else:
    if scale_factor == 1.0:
        print("INFO: Using default scale factor", scale_factor)
    else:
        print("INFO: Using scale factor", scale_factor)

if not visualization:
    visualization = 0
    print("INFO: Visualization: Disabled")
else:
    visualization = 1
    print("INFO: Visualization: Enabled. Checking if monitor is connected...")
    WINDOW_TITLE = 'Apriltag detection from T265 images'
    cv2.namedWindow(WINDOW_TITLE, cv2.WINDOW_AUTOSIZE)
    print("INFO: Monitor is connected. Press `q` to exit.")
    display_mode = "stack"

if not debug_enable:
    debug_enable = 0
else:
    debug_enable = 1
    np.set_printoptions(precision=4, suppress=True) # Format output on terminal 
    print("INFO: Debug messages enabled.")

# Transformation to convert different camera orientations to NED convention. Replace camera_orientation_default for your configuration.
#   0: Forward, USB port to the right
#   1: Downfacing, USB port to the right 
# Important note for downfacing camera: you need to tilt the vehicle's nose up a little - not flat - before you run the script, otherwise the initial yaw will be randomized, read here for more details: https://github.com/IntelRealSense/librealsense/issues/4080. Tilt the vehicle to any other sides and the yaw might not be as stable.

if not camera_orientation:
    camera_orientation = camera_orientation_default
    print("INFO: Using default camera orientation", camera_orientation)
else:
    print("INFO: Using camera orientation", camera_orientation)

if camera_orientation == 0: 
    # Forward, USB port to the right
    H_aeroRef_T265Ref = np.array([[0,0,-1,0],[1,0,0,0],[0,-1,0,0],[0,0,0,1]])
    H_T265body_aeroBody = np.linalg.inv(H_aeroRef_T265Ref)
elif camera_orientation == 1:
    # Downfacing, USB port to the right
    H_aeroRef_T265Ref = np.array([[0,0,-1,0],[1,0,0,0],[0,-1,0,0],[0,0,0,1]])
    H_T265body_aeroBody = np.array([[0,1,0,0],[1,0,0,0],[0,0,-1,0],[0,0,0,1]])
else: 
    # Default is facing forward, USB port to the right
    H_aeroRef_T265Ref = np.array([[0,0,-1,0],[1,0,0,0],[0,-1,0,0],[0,0,0,1]])
    H_T265body_aeroBody = np.linalg.inv(H_aeroRef_T265Ref)

#######################################
# Functions for OpenCV 
#######################################
"""
In this section, we will set up the functions that will translate the camera
intrinsics and extrinsics from librealsense into parameters that can be used
with OpenCV.
The T265 uses very wide angle lenses, so the distortion is modeled using a four
parameter distortion model known as Kanalla-Brandt. OpenCV supports this
distortion model in their "fisheye" module, more details can be found here:
https://docs.opencv.org/3.4/db/d58/group__calib3d__fisheye.html
"""

"""
Returns R, T transform from src to dst
"""
def get_extrinsics(src, dst):
    extrinsics = src.get_extrinsics_to(dst)
    R = np.reshape(extrinsics.rotation, [3,3]).T
    T = np.array(extrinsics.translation)
    return (R, T)

"""
Returns a camera matrix K from librealsense intrinsics
"""
def camera_matrix(intrinsics):
    return np.array([[intrinsics.fx,             0, intrinsics.ppx],
                    [            0, intrinsics.fy, intrinsics.ppy],
                    [            0,             0,              1]])

"""
Returns the fisheye distortion from librealsense intrinsics
"""
def fisheye_distortion(intrinsics):
    return np.array(intrinsics.coeffs[:4])

#######################################
# Functions for AprilTag detection
#######################################
tag_landing_id = 0
tag_landing_size = 0.144            # tag's border size, measured in meter
tag_image_source = "right"   # for Realsense T265, we can use "left" or "right"

at_detector = apriltags3.Detector(searchpath=['apriltags'],
                       families='tag36h11',
                       nthreads=1,
                       quad_decimate=1.0,
                       quad_sigma=0.0,
                       refine_edges=1,
                       decode_sharpening=0.25,
                       debug=0)

#######################################
# Functions for MAVLink
#######################################

# Define function to send landing_target mavlink message for mavlink based precision landing
# http://mavlink.org/messages/common#LANDING_TARGET
def send_land_target_message():
    global current_time, H_camera_tag, is_landing_tag_detected

    if is_landing_tag_detected == True:
        x = H_camera_tag[0][3]
        y = H_camera_tag[1][3]
        z = H_camera_tag[2][3]

        x_offset_rad = m.atan(x / z)
        y_offset_rad = m.atan(y / z)
        distance = np.sqrt(x * x + y * y + z * z)

        msg = vehicle.message_factory.landing_target_encode(
            current_time,                       # time target data was processed, as close to sensor capture as possible
            0,                                  # target num, not used
            mavutil.mavlink.MAV_FRAME_BODY_NED, # frame, not used
            x_offset_rad,                       # X-axis angular offset, in radians
            y_offset_rad,                       # Y-axis angular offset, in radians
            distance,                           # distance, in meters
            0,                                  # Target x-axis size, in radians
            0,                                  # Target y-axis size, in radians
            0,                                  # x	float	X Position of the landing target on MAV_FRAME
            0,                                  # y	float	Y Position of the landing target on MAV_FRAME
            0,                                  # z	float	Z Position of the landing target on MAV_FRAME
            (1,0,0,0),      # q	float[4]	Quaternion of landing target orientation (w, x, y, z order, zero-rotation is 1, 0, 0, 0)
            2,              # type of landing target: 2 = Fiducial marker
            1,              # position_valid boolean
        )
        vehicle.send_mavlink(msg)
        vehicle.flush()

# https://mavlink.io/en/messages/common.html#VISION_POSITION_ESTIMATE
def send_vision_position_message():
    global current_time, H_aeroRef_aeroBody

    if H_aeroRef_aeroBody is not None:
        rpy_rad = np.array( tf.euler_from_matrix(H_aeroRef_aeroBody, 'sxyz'))

        msg = vehicle.message_factory.vision_position_estimate_encode(
            current_time,                       # us Timestamp (UNIX time or time since system boot)
            H_aeroRef_aeroBody[0][3],	        # Global X position
            H_aeroRef_aeroBody[1][3],           # Global Y position
            H_aeroRef_aeroBody[2][3],	        # Global Z position
            rpy_rad[0],	                        # Roll angle
            rpy_rad[1],	                        # Pitch angle
            rpy_rad[2]	                        # Yaw angle
        )

        vehicle.send_mavlink(msg)
        vehicle.flush()

# For a lack of a dedicated message, we pack the confidence level into a message that will not be used, so we can view it on GCS
# Confidence level value: 0 - 3, remapped to 0 - 100: 0% - Failed / 33.3% - Low / 66.6% - Medium / 100% - High 
def send_confidence_level_dummy_message():
    global data, current_confidence
    if data is not None:
        # Show confidence level on terminal
        print("INFO: Tracking confidence: ", pose_data_confidence_level[data.tracker_confidence])

        # Send MAVLink message to show confidence level numerically
        msg = vehicle.message_factory.vision_position_delta_encode(
            0,	            #us	Timestamp (UNIX time or time since system boot)
            0,	            #Time since last reported camera frame
            [0, 0, 0],      #angle_delta
            [0, 0, 0],      #position_delta
            float(data.tracker_confidence * 100 / 3)          
        )
        vehicle.send_mavlink(msg)
        vehicle.flush()

        # If confidence level changes, send MAVLink message to show confidence level textually and phonetically
        if current_confidence is None or current_confidence != data.tracker_confidence:
            current_confidence = data.tracker_confidence
            confidence_status_string = 'Tracking confidence: ' + pose_data_confidence_level[data.tracker_confidence]
            status_msg = vehicle.message_factory.statustext_encode(
                3,	            #severity, defined here: https://mavlink.io/en/messages/common.html#MAV_SEVERITY, 3 will let the message be displayed on Mission Planner HUD
                confidence_status_string.encode()	  #text	char[50]       
            )
            vehicle.send_mavlink(status_msg)
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

# Listen to messages that indicate EKF is ready to set home, then set EKF home automatically.
def statustext_callback(self, attr_name, value):
    # These are the status texts that indicates EKF is ready to receive home position
    if value.text == "GPS Glitch" or value.text == "GPS Glitch cleared" or value.text == "EKF2 IMU1 ext nav yaw alignment complete":
        time.sleep(0.1)
        print("INFO: Set EKF home with default GPS location")
        set_default_global_origin()
        set_default_home_position()

# Listen to attitude data to acquire heading when compass data is enabled
def att_msg_callback(self, attr_name, value):
    global heading_north_yaw
    if heading_north_yaw is None:
        heading_north_yaw = value.yaw
        print("INFO: Received first ATTITUDE message with heading yaw", heading_north_yaw * 180 / m.pi, "degrees")
    else:
        heading_north_yaw = value.yaw
        print("INFO: Received ATTITUDE message with heading yaw", heading_north_yaw * 180 / m.pi, "degrees")

# Monitor user input from the terminal and update scale factor accordingly
def scale_update():
    global scale_factor
    while True:
        scale_factor = float(input("INFO: Type in new scale as float number\n"))
        print("INFO: New scale is ", scale_factor)  

# Connect to FCU through serial port
def vehicle_connect():
    global vehicle

    try:
        vehicle = connect(connection_string, wait_ready = True, baud = connection_baudrate, source_system = 1)
    except KeyboardInterrupt:    
        pipe.stop()
        print("INFO: Exiting")
        sys.exit()
    except:
        print('Connection error! Retrying...')

    if vehicle == None:
        return False
    else:
        return True

# Connect to the T265 through USB 3.0 (must be USB 3.0 since image streams are being consumed)
def realsense_connect():
    global pipe
    # Declare RealSense pipeline, encapsulating the actual device and sensors
    pipe = rs.pipeline()

    # Build config object and stream everything before requesting data
    cfg = rs.config()

    # Enable the streams we are interested in
    # Note: It is not currently possible to enable only one image stream
    cfg.enable_stream(rs.stream.pose)               # Positional data 
    cfg.enable_stream(rs.stream.fisheye, 1)         # Image stream left
    cfg.enable_stream(rs.stream.fisheye, 2)         # Image stream right

    # Start streaming with requested config and callback
    pipe.start(cfg)

#######################################
# Main code starts here
#######################################

# Set up a mutex to share data between threads 
frame_mutex = threading.Lock()

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
current_confidence = None
H_aeroRef_aeroBody = None
H_camera_tag = None
is_landing_tag_detected = False # This flag returns true only if the tag with landing id is currently detected
heading_north_yaw = None

# Send MAVlink messages in the background
sched = BackgroundScheduler()
sched.add_job(send_vision_position_message, 'interval', seconds = 1/vision_msg_hz)
sched.add_job(send_confidence_level_dummy_message, 'interval', seconds = 1/confidence_msg_hz)
sched.add_job(send_land_target_message, 'interval', seconds = 1/landing_target_msg_hz_default)

# For scale calibration, we will use a thread to monitor user input
if scale_calib_enable == True:
    scale_update_thread = threading.Thread(target=scale_update)
    scale_update_thread.daemon = True
    scale_update_thread.start()

sched.start()

if compass_enabled == 1:
    # Wait a short while for yaw to be correctly initiated
    time.sleep(1)

print("INFO: Starting main loop...")

try:
    # Configure the OpenCV stereo algorithm. See
    # https://docs.opencv.org/3.4/d2/d85/classcv_1_1StereoSGBM.html for a
    # description of the parameters
    window_size = 5
    min_disp = 16
    # must be divisible by 16
    num_disp = 112 - min_disp
    max_disp = min_disp + num_disp
    stereo = cv2.StereoSGBM_create(minDisparity = min_disp,
                                    numDisparities = num_disp,
                                    blockSize = 16,
                                    P1 = 8*3*window_size**2,
                                    P2 = 32*3*window_size**2,
                                    disp12MaxDiff = 1,
                                    uniquenessRatio = 10,
                                    speckleWindowSize = 100,
                                    speckleRange = 32)

    # Retreive the stream and intrinsic properties for both cameras
    profiles = pipe.get_active_profile()

    streams = {"left"  : profiles.get_stream(rs.stream.fisheye, 1).as_video_stream_profile(),
               "right" : profiles.get_stream(rs.stream.fisheye, 2).as_video_stream_profile()}
    intrinsics = {"left"  : streams["left"].get_intrinsics(),
                  "right" : streams["right"].get_intrinsics()}

    # Print information about both cameras
    print("INFO: Using stereo fisheye cameras")
    if debug_enable == 1:
        print("INFO: T265 Left camera:",  intrinsics["left"])
        print("INFO: T265 Right camera:", intrinsics["right"])

    # Translate the intrinsics from librealsense into OpenCV
    K_left  = camera_matrix(intrinsics["left"])
    D_left  = fisheye_distortion(intrinsics["left"])
    K_right = camera_matrix(intrinsics["right"])
    D_right = fisheye_distortion(intrinsics["right"])
    (width, height) = (intrinsics["left"].width, intrinsics["left"].height)

    # Get the relative extrinsics between the left and right camera
    (R, T) = get_extrinsics(streams["left"], streams["right"])

    # We need to determine what focal length our undistorted images should have
    # in order to set up the camera matrices for initUndistortRectifyMap.  We
    # could use stereoRectify, but here we show how to derive these projection
    # matrices from the calibration and a desired height and field of view

    # We calculate the undistorted focal length:
    #
    #         h
    # -----------------
    #  \      |      /
    #    \    | f  /
    #     \   |   /
    #      \ fov /
    #        \|/
    stereo_fov_rad = 90 * (m.pi/180)    # desired fov degree, 90 seems to work ok
    stereo_height_px = 300              # 300x300 pixel stereo output
    stereo_focal_px = stereo_height_px/2 / m.tan(stereo_fov_rad/2)

    # We set the left rotation to identity and the right rotation
    # the rotation between the cameras
    R_left = np.eye(3)
    R_right = R

    # The stereo algorithm needs max_disp extra pixels in order to produce valid
    # disparity on the desired output region. This changes the width, but the
    # center of projection should be on the center of the cropped image
    stereo_width_px = stereo_height_px + max_disp
    stereo_size = (stereo_width_px, stereo_height_px)
    stereo_cx = (stereo_height_px - 1)/2 + max_disp
    stereo_cy = (stereo_height_px - 1)/2

    # Construct the left and right projection matrices, the only difference is
    # that the right projection matrix should have a shift along the x axis of
    # baseline * focal_length
    P_left = np.array([[stereo_focal_px, 0, stereo_cx, 0],
                       [0, stereo_focal_px, stereo_cy, 0],
                       [0,               0,         1, 0]])
    P_right = P_left.copy()
    P_right[0][3] = T[0] * stereo_focal_px

    # Construct Q for use with cv2.reprojectImageTo3D. Subtract max_disp from x
    # since we will crop the disparity later
    Q = np.array([[1, 0,       0, -(stereo_cx - max_disp)],
                  [0, 1,       0, -stereo_cy],
                  [0, 0,       0, stereo_focal_px],
                  [0, 0, -1/T[0], 0]])

    # Create an undistortion map for the left and right camera which applies the
    # rectification and undoes the camera distortion. This only has to be done
    # once
    m1type = cv2.CV_32FC1
    (lm1, lm2) = cv2.fisheye.initUndistortRectifyMap(K_left, D_left, R_left, P_left, stereo_size, m1type)
    (rm1, rm2) = cv2.fisheye.initUndistortRectifyMap(K_right, D_right, R_right, P_right, stereo_size, m1type)
    undistort_rectify = {"left"  : (lm1, lm2),
                         "right" : (rm1, rm2)}

    # For AprilTag detection
    camera_params = [stereo_focal_px, stereo_focal_px, stereo_cx, stereo_cy]

    while True:
        # Wait for the next set of frames from the camera
        frames = pipe.wait_for_frames()

        # Fetch pose frame
        pose = frames.get_pose_frame()

        # Process pose streams
        if pose:
            # Store the timestamp for MAVLink messages
            current_time = int(round(time.time() * 1000000))

            # Pose data consists of translation and rotation
            data = pose.get_pose_data()

            # In transformations, Quaternions w+ix+jy+kz are represented as [w, x, y, z]!
            H_T265Ref_T265body = tf.quaternion_matrix([data.rotation.w, data.rotation.x, data.rotation.y, data.rotation.z]) 
            H_T265Ref_T265body[0][3] = data.translation.x * scale_factor
            H_T265Ref_T265body[1][3] = data.translation.y * scale_factor
            H_T265Ref_T265body[2][3] = data.translation.z * scale_factor

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

            # Show debug messages here
            if debug_enable == 1:
                os.system('clear') # This helps in displaying the messages to be more readable
                print("DEBUG: Raw RPY[deg]: {}".format( np.array( tf.euler_from_matrix( H_T265Ref_T265body, 'sxyz')) * 180 / m.pi))
                print("DEBUG: NED RPY[deg]: {}".format( np.array( tf.euler_from_matrix( H_aeroRef_aeroBody, 'sxyz')) * 180 / m.pi))
                print("DEBUG: Raw pos xyz : {}".format( np.array( [data.translation.x, data.translation.y, data.translation.z])))
                print("DEBUG: NED pos xyz : {}".format( np.array( tf.translation_from_matrix( H_aeroRef_aeroBody))))

        # Fetch raw fisheye image frames
        f1 = frames.get_fisheye_frame(1).as_video_frame()
        left_data = np.asanyarray(f1.get_data())
        f2 = frames.get_fisheye_frame(2).as_video_frame()
        right_data = np.asanyarray(f2.get_data())

        # Process image streams
        frame_copy = {"left" : left_data, "right" : right_data}

        # Undistort and crop the center of the frames
        center_undistorted = {"left" : cv2.remap(src = frame_copy["left"],
                                      map1 = undistort_rectify["left"][0],
                                      map2 = undistort_rectify["left"][1],
                                      interpolation = cv2.INTER_LINEAR),
                              "right" : cv2.remap(src = frame_copy["right"],
                                      map1 = undistort_rectify["right"][0],
                                      map2 = undistort_rectify["right"][1],
                                      interpolation = cv2.INTER_LINEAR)}

        # Run AprilTag detection algorithm on rectified image. 
        # Params:
        #   tag_image_source for "left" or "right"
        #   tag_landing_size for actual size of the tag
        tags = at_detector.detect(center_undistorted[tag_image_source], True, camera_params, tag_landing_size)

        if tags != []:
            for tag in tags:
                # Check for the tag that we want to land on
                if tag.tag_id == tag_landing_id:
                    is_landing_tag_detected = True
                    H_camera_tag = tf.euler_matrix(0, 0, 0, 'sxyz')
                    H_camera_tag[0][3] = tag.pose_t[0]
                    H_camera_tag[1][3] = tag.pose_t[1]
                    H_camera_tag[2][3] = tag.pose_t[2]
                    print("INFO: Detected landing tag", str(tag.tag_id), " relative to camera at x:", H_camera_tag[0][3], ", y:", H_camera_tag[1][3], ", z:", H_camera_tag[2][3])
        else:
            # print("INFO: No tag detected")
            is_landing_tag_detected = False

        # If enabled, display tag-detected image in a pop-up window, required a monitor to be connected
        if visualization == 1:
            # Create color image from source
            tags_img = center_undistorted[tag_image_source]
                
            # For each detected tag, draw a bounding box and put the id of the tag in the center
            for tag in tags:
                # Setup bounding box
                for idx in range(len(tag.corners)):
                    cv2.line(tags_img, 
                            tuple(tag.corners[idx-1, :].astype(int)), 
                            tuple(tag.corners[idx, :].astype(int)), 
                            thickness = 2,
                            color = (255, 0, 0))

                # The text to be put in the image, here we simply put the id of the detected tag
                text = str(tag.tag_id)

                # get boundary of this text
                textsize = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, 1, 2)[0]

                # Put the text in the middle of the image
                cv2.putText(tags_img, 
                            text,
                            org = (((tag.corners[0, 0] + tag.corners[2, 0] - textsize[0])/2).astype(int), 
                                   ((tag.corners[0, 1] + tag.corners[2, 1] + textsize[1])/2).astype(int)),
                            fontFace = cv2.FONT_HERSHEY_SIMPLEX,
                            fontScale = 0.5,
                            thickness = 2,
                            color = (255, 0, 0))

            # Display the image in a window
            cv2.imshow(WINDOW_TITLE, tags_img)

            # Read keyboard input on the image window
            key = cv2.waitKey(1)
            if key == ord('q') or cv2.getWindowProperty(WINDOW_TITLE, cv2.WND_PROP_VISIBLE) < 1:
                break

except KeyboardInterrupt:
    print("INFO: KeyboardInterrupt has been caught. Cleaning up...")     

finally:
    pipe.stop()
    vehicle.close()
    print("INFO: Realsense pipeline and vehicle object closed.")
    sys.exit()