# vision_to_mavros
## Note: Ongoing project in very early stage
ROS package that listens to `/tf`, transforms the pose of `camera_frame` to `tag_frame`, then transforms to `body_frame` with the specify roll, pitch, yaw rotations that converts the camera to the body. The first transform is done [here](https://github.com/hoangthien94/vision_to_mavros/blob/18c5186abe311615678c25ad18c9049f4913c6a1/src/vision_to_mavros.cpp#L35) while the latter [here](https://github.com/hoangthien94/vision_to_mavros/blob/18c5186abe311615678c25ad18c9049f4913c6a1/src/vision_to_mavros.cpp#L66).

Example RPY for:
- Downfacing camera (Z down), X to the front: r = M_PI, p = 0, y = 0
- Downfacing camear (Z down), X to the right: r = M_PI, p = 0, y = M_PI / 2

## Installation:
```
cd ~/catkin_ws/src
git clone https://github.com/hoangthien94/vision_to_mavros.git
cd ../
catkin_make
```

## Example usage:
* With [Intel® RealSense™ Tracking Camera T265](https://www.intelrealsense.com/tracking-camera-t265/)
```
roslaunch vision_to_mavros t265_to_mavros.launch
```


* With apriltags:
```
roslaunch vision_to_mavros apriltags_to_mavros.launch
```
This will launch `usb_cam` to capture raw images, perform rectification through `image_proc`, use `apriltag_ros` to obtain the pose of the tag in the camera frame, and finally `vision_to_mavros` to first get the pose of camera in the tag frame, transform to body frame by using camera orientation, and publish the body pose to `/mavros/vision_pose/pose` topic. Note that `mavros` should be launch separately since it has a lot of output on the terminal.


