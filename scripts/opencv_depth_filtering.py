#!/usr/bin/env python3

######################################################
## Python implementation of rs-depth.c example      ##
## https://github.com/IntelRealSense/librealsense/tree/master/examples/C/depth
######################################################

# First import the library
import sys
import pyrealsense2 as rs           # Intel RealSense cross-platform open-source API
import time
import numpy as np                  # fundamental package for scientific computing

# in order to import cv2 under python3 when you also have ROS installed
sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages') 
sys.path.append('~/anaconda3/lib/python3.7/site-packages') # Remove if not applicable to your system
import cv2

######################################################
##      These parameters are reconfigurable         ##
######################################################
STREAM_TYPE = [rs.stream.depth, rs.stream.color]  # rs2_stream is a types of data provided by RealSense device
FORMAT      = [rs.format.z16, rs.format.bgr8]     # rs2_format is identifies how binary data is encoded within a frame
WIDTH       = 640              # Defines the number of columns for each frame or zero for auto resolve
HEIGHT      = 480              # Defines the number of lines for each frame or zero for auto resolve
FPS         = 30               # Defines the rate of frames per second
ENABLE_SHOW_IMAGE = True

# List of filters to be applied, in this order.
# The order is according to this recommendation: https://github.com/IntelRealSense/librealsense/blob/master/doc/post-processing-filters.md#using-filters-in-application-code
# Enable/disable - Name - Object (to be used in the main code)
filters = [
    [True,  "Decimation Filter",    None],
    [True,  "Threshold Filter",     None],
    [True,  "Depth to Disparity",   None],
    [True,  "Spatial Filter",       None],
    [True,  "Temporal Filter",      None],
    [False, "Hole Filling Filter",  None],
    [True,  "Disparity to Depth",   None]
]
            
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

    for i in range(len(filters)):
        if filters[i][0] is True:
            print("Applying: ", filters[i][1])
        else:
            print("NOT applying: ", filters[i][1])

    # Filters to be applied, the recommended scheme used in librealsense tools and demos is elaborated below: Depth Frame >> Decimation Filter >> Depth2Disparity Transform** -> Spatial Filter >> Temporal Filter >> Disparity2Depth Transform** >> Hole Filling Filter >> Filtered Depth.
    filters[0][2] = rs.decimation_filter()          # Decimation - reduces depth frame density
    filters[1][2] = rs.threshold_filter()           # Threshold  - removes values outside recommended range
    filters[2][2] = rs.disparity_transform(True)    # depth_to_disparity - transform the scene into disparity domain
    filters[3][2] = rs.spatial_filter()             # Spatial    - edge-preserving spatial smoothing
    filters[4][2] = rs.temporal_filter()            # Temporal   - reduces temporal noise
    filters[5][2] = rs.hole_filling_filter()        # Hole filling - rectify missing data in the resulting image
    filters[6][2] = rs.disparity_transform(False)   # disparity_to_depth - revert the results back to depth
    colorizer = rs.colorizer()

    # Configure the options of the filters
    # filters[0][2].set_option(rs.option.filter_magnitude, 4)
    # filters[4][2].set_option(rs.option.filter_magnitude, 5)
    # filters[4][2].set_option(rs.option.filter_smooth_alpha, 1)
    # filters[4][2].set_option(rs.option.filter_smooth_delta, 50)
    # filters[4][2].set_option(rs.option.holes_fill, 3)

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
                # Apply the filter
                filtered_frame = filters[i][2].process(filtered_frame)
        
        # Show the processing speed
        processing_speed = 1/(time.time() - last_time)
        print("\r>> Processing speed %.2f fps" %(processing_speed), end='')
        last_time = time.time()

        if ENABLE_SHOW_IMAGE:
            # Convert images to colorized numpy arrays
            input_image = np.asanyarray(colorizer.colorize(depth_frame).get_data())
            output_image = np.asanyarray(colorizer.colorize(filtered_frame).get_data())

            # Configure the settings
            display_image = np.hstack((input_image, cv2.resize(output_image, (WIDTH, HEIGHT))))
            display_name  = 'Input/output depth'

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