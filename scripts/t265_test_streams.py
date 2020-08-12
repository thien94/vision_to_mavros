#####################################################
##          librealsense T265 streams test         ##
#####################################################
# This assumes .so file is found on the same directory
import pyrealsense2 as rs

# Prettier prints for reverse-engineering
from pprint import pprint
import numpy as np

# Get realsense pipeline handle
pipe = rs.pipeline()

# Print all connected devices and find the T265
devices = rs.context().devices
for i in range(len(devices)):
    print('Found device:', devices[i].get_info(rs.camera_info.name), ', with serial number: ', devices[i].get_info(rs.camera_info.serial_number))

# Configure the pipeline
cfg = rs.config()

# Prints a list of available streams, not all are supported by each device
print('Available streams:')
pprint(dir(rs.stream))

# Enable streams you are interested in
cfg.enable_stream(rs.stream.pose) # Positional data (translation, rotation, velocity etc)
cfg.enable_stream(rs.stream.fisheye, 1) # Left camera
cfg.enable_stream(rs.stream.fisheye, 2) # Right camera

# Start the configured pipeline
pipe.start(cfg)

try:
    while(1):
        frames = pipe.wait_for_frames()

        # Left fisheye camera frame
        left = frames.get_fisheye_frame(1)
        left_data = np.asanyarray(left.get_data())

        # Right fisheye camera frame
        right = frames.get_fisheye_frame(2)
        right_data = np.asanyarray(right.get_data())

        print('Left frame', left_data.shape)
        print('Right frame', right_data.shape)

        # Positional data frame
        pose = frames.get_pose_frame()
        if pose:
            pose_data = pose.get_pose_data()
            print("\nFrame number: %5.0f" % (pose.frame_number))
            print("Position xyz: % 2.4f % 2.4f % 2.4f" % (pose_data.translation.x, pose_data.translation.y, pose_data.translation.z))
            print("Velocity xyz: % 2.4f % 2.4f % 2.4f" % (pose_data.velocity.x, pose_data.velocity.y, pose_data.velocity.z))
            print("Accelera xyz: % 2.4f % 2.4f % 2.4f" % (pose_data.acceleration.x, pose_data.acceleration.y, pose_data.acceleration.z))
            print("Quatern xyzw: % 2.4f % 2.4f % 2.4f % 2.4f" % (pose_data.rotation.x, pose_data.rotation.y, pose_data.rotation.z, pose_data.rotation.w))
finally:
    pipe.stop()
