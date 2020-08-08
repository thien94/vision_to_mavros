#!/usr/bin/env python3

######################################################
## Python implementation of the following examples  ##
## https://github.com/IntelRealSense/librealsense/blob/master/doc/post-processing-filters.md
## https://github.com/IntelRealSense/librealsense/tree/master/wrappers/opencv/depth-filter
## https://github.com/IntelRealSense/librealsense/tree/master/examples/C/depth
## https://github.com/IntelRealSense/librealsense/blob/master/wrappers/python/examples/python-rs400-advanced-mode-example.py
######################################################

# Install required packages: 
#   pip3 install pyrealsense2
#   pip3 install opencv-python

# First import the libraries
import sys
import pyrealsense2 as rs           # Intel RealSense cross-platform open-source API
import time
import numpy as np                  # fundamental package for scientific computing
import json

# in order to import cv2 under python3 when you also have ROS installed
import os
if os.path.exists("/opt/ros/kinetic/lib/python2.7/dist-packages"):
    sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages') 
if os.path.exists("~/anaconda3/lib/python3.7/site-packages"):
    sys.path.append('~/anaconda3/lib/python3.7/site-packages')
import cv2

######################################################
##      These parameters are reconfigurable         ##
######################################################
STREAM_TYPE = [rs.stream.depth, rs.stream.color]  # rs2_stream is a types of data provided by RealSense device
FORMAT      = [rs.format.z16, rs.format.bgr8]     # rs2_format is identifies how binary data is encoded within a frame
WIDTH       = 848              # Defines the number of columns for each frame or zero for auto resolve
HEIGHT      = 480              # Defines the number of lines for each frame or zero for auto resolve
FPS         = 30               # Defines the rate of frames per second
DISPLAY_WINDOW_NAME = 'Input/output depth'
OPTION_WINDOW_NAME  = 'Filter options'

USE_PRESET_FILE = True
PRESET_FILE  = "../cfg/d4xx-default.json"

# List of filters to be applied, in this order.
# Depth Frame                       (input)
# >> Decimation Filter              (reduces depth frame density) 
# >> Threshold Filter               (removes values outside recommended range)
# >> Depth2Disparity Transform**    (transform the scene into disparity domain)
# >> Spatial Filter                 (edge-preserving spatial smoothing)
# >> Temporal Filter                (reduces temporal noise)
# >> Hole Filling Filter            (rectify missing data in the resulting image)
# >> Disparity2Depth Transform**    (revert the results back to depth)
# >> Filtered Depth                 (output)
filters = [
    [True, "Decimation Filter",     rs.decimation_filter()],
    [True, "Threshold Filter",      rs.threshold_filter()],
    [True, "Depth to Disparity",    rs.disparity_transform(True)],
    [True, "Spatial Filter",        rs.spatial_filter()],
    [True, "Temporal Filter",       rs.temporal_filter()],
    [True, "Hole Filling Filter",   rs.hole_filling_filter()],
    [True, "Disparity to Depth",    rs.disparity_transform(False)]
]

######################################################
##   Functions to change filtering options online   ##
##   Description for each option can be found at:   ##
## https://github.com/IntelRealSense/librealsense/blob/master/doc/post-processing-filters.md
######################################################
decimation_magnitude_min = 2
decimation_magnitude_max = 8
def on_trackbar_decimation(val):
    # Sanity check
    if val < decimation_magnitude_min:
        print("\nFilter magnitude for Decimation Filter cannot be smaller than ", decimation_magnitude_min)
        val = decimation_magnitude_min
    filters[0][2].set_option(rs.option.filter_magnitude, val)

threshold_min_m = 0.15
threshold_max_m = 10.0
def on_trackbar_max_threshold(val_m):
    # Sanity check
    if val_m < threshold_min_m:
        print("\nMaximum threshold cannot be smaller than ", threshold_min_m)
        val_m = threshold_min_m
    elif val_m > threshold_max_m:
        print("\nMaximum threshold cannot be larger than ", threshold_max_m)
        val_m = threshold_max_m
    # filters[1][2].set_option(rs.option.min_distance, val_m)
    filters[1][2].set_option(rs.option.max_distance, val_m)


spatial_magnitude_min = 1
spatial_magnitude_max = 5
def on_trackbar_spatial_magnitude(val):
    # Sanity check
    if val < spatial_magnitude_min:
        print("\nFilter magnitude for Spatial Filter cannot be smaller than ", spatial_magnitude_min)
        val = spatial_magnitude_min
    filters[3][2].set_option(rs.option.filter_magnitude, val)

spatial_smooth_alpha_min = 0.25
spatial_smooth_alpha_max = 1
spatial_smooth_alpha_scaled_max = 10
def on_trackbar_spatial_smooth_alpha(val):
    # Step in cv2 trackbar only allows discrete value, so we remap it from [0-spatial_smooth_alpha_scaled_max] to [0-spatial_smooth_alpha_max]
    val = val / spatial_smooth_alpha_scaled_max * spatial_smooth_alpha_max
    # Sanity check
    if val < spatial_smooth_alpha_min:
        print("\nFilter magnitude for Spatial Filter cannot be smaller than ", spatial_smooth_alpha_min)
        val = spatial_smooth_alpha_min
    filters[3][2].set_option(rs.option.filter_smooth_alpha, val)

spatial_smooth_delta_min = 1
spatial_smooth_delta_max = 50
def on_trackbar_spatial_smooth_delta(val):
    # Sanity check
    if val < spatial_smooth_delta_min:
        print("\nSmooth alpha for Spatial Filter cannot be smaller than ", spatial_smooth_delta_min)
        val = spatial_smooth_delta_min
    filters[3][2].set_option(rs.option.filter_smooth_delta, val)

spatial_hole_filling_min = 0
spatial_hole_filling_max = 5
def on_trackbar_spatial_hole_filling(val):
    # Sanity check
    if val < spatial_hole_filling_min:
        print("\nSmooth alpha for Spatial Filter cannot be smaller than ", spatial_hole_filling_min)
        val = spatial_hole_filling_min
    filters[3][2].set_option(rs.option.holes_fill, val)

hole_filling_filter_min = 0
hole_filling_filter_max = 2 
def on_trackbar_hole_filling(val):
    # direction: 0-from left, 1-farest from around, 2-nearest from around
    filters[5][2].set_option(rs.option.holes_fill, val)

######################################################
##      Functions to interface with D4xx cameras    ##
######################################################
DS5_product_ids = ["0AD1", "0AD2", "0AD3", "0AD4", "0AD5", "0AF6", "0AFE", "0AFF", "0B00", "0B01", "0B03", "0B07","0B3A"]

def find_device_that_supports_advanced_mode() :
    ctx = rs.context()
    ds5_dev = rs.device()
    devices = ctx.query_devices();
    for dev in devices:
        if dev.supports(rs.camera_info.product_id) and str(dev.get_info(rs.camera_info.product_id)) in DS5_product_ids:
            if dev.supports(rs.camera_info.name):
                print("Found device that supports advanced mode:", dev.get_info(rs.camera_info.name))
            return dev
    raise Exception("No device that supports advanced mode was found")

# Loop until we successfully enable advanced mode
def d4xx_enable_advanced_mode(advnc_mode):
    while not advnc_mode.is_enabled():
        print("Trying to enable advanced mode...")
        advnc_mode.toggle_advanced_mode(True)
        # At this point the device will disconnect and re-connect.
        print("Sleeping for 5 seconds...")
        time.sleep(5)
        # The 'dev' object will become invalid and we need to initialize it again
        dev = find_device_that_supports_advanced_mode()
        advnc_mode = rs.rs400_advanced_mode(dev)
        print("Advanced mode is", "enabled" if advnc_mode.is_enabled() else "disabled")

# Load the settings stored in the JSON file
def d4xx_load_settings_file(advnc_mode, setting_file):
    # Sanity checks
    if os.path.isfile(setting_file):
        print("Setting file found", setting_file)
    else:
        print("Cannot find setting file ", setting_file)
        exit()

    if advnc_mode.is_enabled():
        print("Advanced mode is enabled")
    else:
        print("Device does not support advanced mode")
        exit()
    
    # Input for load_json() is the content of the json file, not the file path
    with open(setting_file, 'r') as file:
        json_text = file.read().strip()

    advnc_mode.load_json(json_text)

######################################################
##      Main program starts here                    ##
######################################################
try:
    if USE_PRESET_FILE:
        device = find_device_that_supports_advanced_mode()
        advnc_mode = rs.rs400_advanced_mode(device)
        d4xx_enable_advanced_mode(advnc_mode)
        d4xx_load_settings_file(advnc_mode, PRESET_FILE)

    # Create a context object. This object owns the handles to all connected realsense devices
    pipeline = rs.pipeline()

    # Configure depth and color streams
    config = rs.config()
    config.enable_stream(STREAM_TYPE[0], WIDTH, HEIGHT, FORMAT[0], FPS)
    config.enable_stream(STREAM_TYPE[1], WIDTH, HEIGHT, FORMAT[1], FPS)
    colorizer = rs.colorizer()

    for i in range(len(filters)):
        if filters[i][0] is True:
            print("Applying: ", filters[i][1])
        else:
            print("NOT applying: ", filters[i][1])
    
    # Start streaming
    profile = pipeline.start(config)

    # Create the image windows to be used
    cv2.namedWindow(OPTION_WINDOW_NAME, cv2.WINDOW_NORMAL)
    cv2.namedWindow(DISPLAY_WINDOW_NAME, cv2.WINDOW_AUTOSIZE)

    # Create trackbars for options modifiers
    # NOTE: - The trackbar's minimum is always zero and cannot be changed
    #       - The trackbar's steps are discrete (so 0-1-2 etc.)
    cv2.createTrackbar('Decimation magnitude [2-8]', OPTION_WINDOW_NAME, 0, decimation_magnitude_max, on_trackbar_decimation)
    cv2.createTrackbar('Threshold [0-any] (cm)', OPTION_WINDOW_NAME, int(threshold_max_m), int(threshold_max_m), on_trackbar_max_threshold)
    cv2.createTrackbar('Spatial magnitude [1-5]', OPTION_WINDOW_NAME, 0, spatial_magnitude_max, on_trackbar_spatial_magnitude)
    cv2.createTrackbar('Spatial smooth alpha [0.25-1]', OPTION_WINDOW_NAME, 0, spatial_smooth_alpha_scaled_max, on_trackbar_spatial_smooth_alpha)
    cv2.createTrackbar('Spatial smooth delta [1-50]', OPTION_WINDOW_NAME, 0, spatial_smooth_delta_max, on_trackbar_spatial_smooth_delta)
    cv2.createTrackbar('Spatial hole filling [0-5]', OPTION_WINDOW_NAME, 0, spatial_hole_filling_max, on_trackbar_spatial_hole_filling)
    cv2.createTrackbar('Hole filling direction [0-2]', OPTION_WINDOW_NAME, 0, hole_filling_filter_max, on_trackbar_hole_filling)
    # Avoid unnecessary blank space in the displayed window
    cv2.resizeWindow(OPTION_WINDOW_NAME, 600, 100)

    last_time = time.time()
    while True:
        # This call waits until a new coherent set of frames is available on a device
        # Calls to get_frame_data(...) and get_frame_timestamp(...) on a device will return stable values until wait_for_frames(...) is called
        frames = pipeline.wait_for_frames()
        depth_frame = frames.get_depth_frame()

        if not depth_frame:
            continue
        
        # Apply the filters
        filtered_frame = depth_frame
        for i in range(len(filters)):
            if filters[i][0] is True:
                filtered_frame = filters[i][2].process(filtered_frame)
        
        # Show the processing speed
        processing_speed = 1/(time.time() - last_time)
        print("\r>> Processing speed %.2f fps" %(processing_speed), end='')
        last_time = time.time()

        # Prepare the images
        input_image = np.asanyarray(colorizer.colorize(depth_frame).get_data())
        output_image = np.asanyarray(colorizer.colorize(filtered_frame).get_data())
        display_image = np.hstack((input_image, cv2.resize(output_image, (WIDTH, HEIGHT))))

        # Put the fps in the corner of the image
        text = ("%0.2f" % (processing_speed,)) + ' fps'
        textsize = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, 1, 2)[0]
        cv2.putText(display_image, 
                    text,
                    org = (int((display_image.shape[1] - textsize[0]/2)), int((textsize[1])/2)),
                    fontFace = cv2.FONT_HERSHEY_SIMPLEX,
                    fontScale = 0.5,
                    thickness = 1,
                    color = (255, 255, 255))

        # Show the images with the fps
        cv2.imshow(DISPLAY_WINDOW_NAME, display_image)
        cv2.waitKey(1)

except KeyboardInterrupt:
    print('Keyboard interrupt. Closing the script...')

except Exception as e:
    print(e)
    pass