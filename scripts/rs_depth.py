######################################################
## Python implementation of rs-depth.c example      ##
## https://github.com/IntelRealSense/librealsense/tree/master/examples/C/depth
######################################################

# First import the library
import pyrealsense2 as rs
import sys

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

######################################################
##      Main program starts here                    ##
######################################################
try:
    # Create a context object. This object owns the handles to all connected realsense devices
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_stream(STREAM_TYPE, WIDTH, HEIGHT, FORMAT, FPS)
    pipeline.start(config)

    while True:
        # This call waits until a new coherent set of frames is available on a device
        # Calls to get_frame_data(...) and get_frame_timestamp(...) on a device will return stable values until wait_for_frames(...) is called
        frames = pipeline.wait_for_frames()
        depth = frames.get_depth_frame()

        if not depth:
            continue

        # Print a simple text-based representation of the image, by breaking it into WIDTH_RATIO x HEIGHT_RATIO pixel regions and approximating the coverage of pixels within MAX_DEPTH
        img_txt = ""
        coverage = [0] * ROW_LENGTH

        for y in range(HEIGHT):
            for x in range(WIDTH):
                dist = depth.get_distance(x, y)
                if 0 < dist and dist < MAX_DEPTH:
                    coverage[x//WIDTH_RATIO] += 1
            
            if y % HEIGHT_RATIO is (HEIGHT_RATIO - 1):
                line = ""
                for c in coverage:
                    pixel_index = c // int(HEIGHT_RATIO * WIDTH_RATIO / (len(pixels) - 1))  # Magic number: c // 25
                    line += pixels[pixel_index]
                coverage = [0] * ROW_LENGTH
                img_txt += line + "\n"
        
        print(img_txt)

except Exception as e:
    print(e)
    pass