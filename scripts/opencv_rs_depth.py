######################################################
## Python implementation of rs-depth.c example      ##
## https://github.com/IntelRealSense/librealsense/tree/master/examples/C/depth
######################################################

# First import the library
import sys
import pyrealsense2 as rs           # Intel RealSense cross-platform open-source API
import time
import numpy as np                  # fundamental package for scientific computing
from numba import njit              # pip install numba, or use anaconda to install

sys.path.remove('/opt/ros/kinetic/lib/python2.7/dist-packages') # in order to import cv2 under python3
sys.path.append('~/anaconda3/lib/python3.7/site-packages')
import cv2


######################################################
##      These parameters are reconfigurable         ##
######################################################
STREAM_TYPE  = rs.stream.depth  # rs2_stream is a types of data provided by RealSense device
FORMAT       = rs.format.z16    # rs2_format is identifies how binary data is encoded within a frame
WIDTH        = 640              # Defines the number of columns for each frame or zero for auto resolve
HEIGHT       = 480              # Defines the number of lines for each frame or zero for auto resolve
FPS          = 30               # Defines the rate of frames per second
HEIGHT_RATIO = 20               # Defines the height ratio between the original frame to the new frame
WIDTH_RATIO  = 10               # Defines the width ratio between the original frame to the new frame
MAX_DEPTH    = 1                # Approximate the coverage of pixels within this range (meter)
ROW_LENGTH   = int(WIDTH / WIDTH_RATIO)
pixels       = " .:nhBXWW"      # The text-based representation of depth

depth_scale = 0

######################################################
##      Functions                                   ##
######################################################

@njit
def calculate_depth_txt_img(depth_mat):
    img_txt = ""
    coverage = [0] * ROW_LENGTH
    for y in range(HEIGHT):
        # Create a depth histogram for each row
        for x in range(WIDTH):
            # dist = depth_frame.get_distance(x, y) is simplier to implement, but calling `get_distance` excessively can result in bad performance (much slower), so it could be beneficial to read the `DEPTH_UNITS` option directly from the `depth_sensor` and use it to convert raw depth pixels to meters, which is what we do here
            dist = depth_mat[y,x] * depth_scale
            if 0 < dist and dist < MAX_DEPTH:
                coverage[x // WIDTH_RATIO] += 1

        if y % HEIGHT_RATIO is (HEIGHT_RATIO - 1):
            line = ""
            for c in coverage:
                pixel_index = c // int(HEIGHT_RATIO * WIDTH_RATIO / (len(pixels) - 1))  # Magic number: c // 25
                line += pixels[pixel_index]
            coverage = [0] * ROW_LENGTH
            img_txt += line + "\n"
            
    return img_txt

######################################################
##      Main program starts here                    ##
######################################################
try:
    # Create a context object. This object owns the handles to all connected realsense devices
    pipeline = rs.pipeline()

    # Configure depth and color streams
    config = rs.config()
    config.enable_stream(STREAM_TYPE, WIDTH, HEIGHT, FORMAT, FPS)
    config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)

    # Filters to be applied
    decimation = rs.decimation_filter()
    # decimation.set_option(rs.option.filter_magnitude, 4)

    spatial = rs.spatial_filter()
    # spatial.set_option(rs.option.filter_magnitude, 5)
    # spatial.set_option(rs.option.filter_smooth_alpha, 1)
    # spatial.set_option(rs.option.filter_smooth_delta, 50)
    # spatial.set_option(rs.option.holes_fill, 3)

    hole_filling = rs.hole_filling_filter()

    depth_to_disparity = rs.disparity_transform(True)
    disparity_to_depth = rs.disparity_transform(False)

    # Start streaming
    profile = pipeline.start(config)
    colorizer = rs.colorizer()

    while True:
        # This call waits until a new coherent set of frames is available on a device
        # Calls to get_frame_data(...) and get_frame_timestamp(...) on a device will return stable values until wait_for_frames(...) is called
        frames = pipeline.wait_for_frames()
        depth_frame = frames.get_depth_frame()

        if not depth_frame:
            continue
        
        filtered_depth = depth_frame
        # Apply the filter(s) according to this recommendation: https://github.com/IntelRealSense/librealsense/blob/master/doc/post-processing-filters.md#using-filters-in-application-code
        # filtered_depth = decimation.process(filtered_depth)
        # filtered_depth = depth_to_disparity.process(filtered_depth)
        # filtered_depth = spatial.process(filtered_depth)
        # filtered_depth = disparity_to_depth.process(filtered_depth)
        filtered_depth = hole_filling.process(filtered_depth)

        # Convert image to numpy arrays
        colorized_depth = np.asanyarray(colorizer.colorize(filtered_depth).get_data())

        # Show images
        cv2.namedWindow('Colorized depth', cv2.WINDOW_AUTOSIZE)
        cv2.imshow('Colorized depth', colorized_depth)
        cv2.waitKey(1)

except KeyboardInterrupt:
    print('Keyboard interrupt. Closing the script...')

except Exception as e:
    print(e)
    pass