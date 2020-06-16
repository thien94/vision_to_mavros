#!/usr/bin/env python3

######################################################
## Python implementation of the following examples  ##
## https://github.com/IntelRealSense/librealsense/blob/master/doc/post-processing-filters.md
## https://github.com/IntelRealSense/librealsense/tree/master/wrappers/opencv/depth-filter
## https://github.com/IntelRealSense/librealsense/tree/master/examples/C/depth
######################################################

# First import the libraries
import sys
import pyrealsense2 as rs           # Intel RealSense cross-platform open-source API
import time
import numpy as np                  # fundamental package for scientific computing

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
ENABLE_SHOW_IMAGE = True

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

# Configure the options of the filters
# filters[0][2].set_option(rs.option.filter_magnitude, 4)
# filters[4][2].set_option(rs.option.filter_magnitude, 5)
# filters[4][2].set_option(rs.option.filter_smooth_alpha, 1)
# filters[4][2].set_option(rs.option.filter_smooth_delta, 50)
# filters[4][2].set_option(rs.option.holes_fill, 3)

######################################################
##      Main program starts here                    ##
######################################################
try:
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

        if ENABLE_SHOW_IMAGE:
            # Prepare the images
            display_name  = 'Input/output depth'
            input_image = np.asanyarray(colorizer.colorize(depth_frame).get_data())
            output_image = np.asanyarray(colorizer.colorize(filtered_frame).get_data())
            display_image = np.hstack((input_image, cv2.resize(output_image, (WIDTH, HEIGHT))))

            # Put the fps in the corner of the image
            text = ("%0.2f" % (processing_speed,)) + ' fps'
            textsize = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, 1, 2)[0]
            cv2.namedWindow(display_name, cv2.WINDOW_AUTOSIZE)
            cv2.putText(display_image, 
                        text,
                        org = (int((display_image.shape[1] - textsize[0]/2)), int((textsize[1])/2)),
                        fontFace = cv2.FONT_HERSHEY_SIMPLEX,
                        fontScale = 0.5,
                        thickness = 1,
                        color = (255, 255, 255))

            # Show the images with the fps
            cv2.imshow(display_name, display_image)
            cv2.waitKey(1)

except KeyboardInterrupt:
    print('Keyboard interrupt. Closing the script...')

except Exception as e:
    print(e)
    pass