# vision_to_mavros
## Note: Ongoing project in very early stage
ROS package that listens to `/tf`, transforms the pose of `source_frame_id` to `target_frame_id`, then rotate the frame to match `body_frame` according to [ENU convention](https://dev.px4.io/en/ros/external_position_estimation.html#ros_reference_frames) with appropriate roll, pitch, yaw angles. 

## Installation:
```
cd ~/catkin_ws/src
git clone https://github.com/hoangthien94/vision_to_mavros.git
cd ../
catkin_make
```

## Usage with [Intel® RealSense™ Tracking Camera T265](https://www.intelrealsense.com/tracking-camera-t265/):

There are 3 nodes running in this setup. In 3 separated terminals on RPi:

* T265 node: `roslaunch realsense2_camera rs_t265.launch`. The topic `/camera/odom/sample/` and `/tf` should be published.

* MAVROS node: `roslaunch mavros apm.launch` (with `fcu_url` and other parameters in `apm.launch` modified accordingly). 

`rostopic echo /mavros/state` should show that FCU is connected.

`rostopic echo /mavros/vision_pose/pose` is not published

* vision_to_mavros node: `roslaunch vision_to_mavros t265_tf_to_mavros.launch`

`rostopic echo /mavros/vision_pose/pose` should now show pose data from the T265.

`rostopic hz /mavros/vision_pose/pose` should show that the topic is being published at 30Hz.

Once you have verified each node can run successfully, next time you can launch all 3 nodes at once with: `roslaunch vision_to_mavros t265_all_nodes.launch`, with:

* `rs_t265.launch` as originally provided by `realsense-ros`.
* `apm.launch` modified with your own configuration.
* `t265_tf_to_mavros.launch` as is.


## Usage with [AprilTag](https://github.com/AprilRobotics/apriltag):
```
roslaunch vision_to_mavros apriltags_to_mavros.launch
```
This will launch `usb_cam` to capture raw images, perform rectification through `image_proc`, use `apriltag_ros` to obtain the pose of the tag in the camera frame, and finally `vision_to_mavros` to first get the pose of camera in the tag frame, transform to body frame by using camera orientation, and publish the body pose to `/mavros/vision_pose/pose` topic. Note that `mavros` should be launch separately since it has a lot of output on the terminal.


