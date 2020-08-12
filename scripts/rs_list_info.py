#####################################################
##          librealsense streams test              ##
#####################################################
# This assumes .so file is found on the same directory
import pyrealsense2 as rs

# Prettier prints for reverse-engineering
from pprint import pprint

# Get realsense pipeline handle
pipe = rs.pipeline()

# Print all connected devices and find the T265
devices = rs.context().devices
for i in range(len(devices)):
    print('---------------------------')
    # Other fields of camera_info: https://intelrealsense.github.io/librealsense/python_docs/_generated/pyrealsense2.camera_info.html
    print('Found connected device #', i + 1, ':', devices[i].get_info(rs.camera_info.name), ', serial no: ', devices[i].get_info(rs.camera_info.serial_number))
    print('Available streams for this device:')
    pprint(dir(rs.stream))