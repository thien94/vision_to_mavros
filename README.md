# vision_to_mavros

<p align="center"><img src="https://i.imgur.com/ycQPhMi.png"/> 
  
ROS package that listens to `/tf`, transforms the pose of `source_frame_id` to `target_frame_id`, then rotate the frame to match `body_frame` according to [ENU convention](https://dev.px4.io/en/ros/external_position_estimation.html#ros_reference_frames) with user input roll, pitch, yaw, gamma angles. 

## Installation (tested with Ubuntu 16.04 LTS):
To install this package with [ROS Kinetic](http://wiki.ros.org/kinetic/Installation/Ubuntu):
1. No dependencies required.
2. Setup `catkin` workspace (if not already done so)
```
mkdir -p ~/catkin_ws/src
cd ~/catkin_ws
catkin init
```
3. Clone the repository and build with `catkin_make`:
```
cd ~/catkin_ws/src
git clone https://github.com/hoangthien94/vision_to_mavros.git
cd ../
catkin_make
```
## How it works
- Suppose we have a frame named `source_frame_id` that is measured in a frame named `target_frame_id`. Let `target_frame_id` be the `world {W}` frame, we want to transform `source_frame_id` to `body {B}` frame so that `{B}` and `{W}` conform to `ENU` convention (x is pointing to East direction, y is pointing to the North and z is pointing up).

<p align="center"><img src="https://i.imgur.com/IxkSIt2.png"/> 

- Now assume we already have a default `{B}` and `{W}` that are correct in `ENU`. We will rotate `{B}` in `{W}` by an angle `gamma_world`, in right hand rule. For example, `gamma_world` equals `-1.5707963 (-PI/2)` will make `{B}`'s x axis aligns with `{W}`'s y axis.

- `source_frame_id` will be aligned with that default `{B}` by rotating around its own x, y, z axis by angles defined by `roll_cam`, `pitch_cam`, `yaw_cam`, in that order.

## Node: vision_to_mavros

### Parameters:

* `target_frame_id`: id of target frame (world/map/base_link)
* `source_frame_id`: id of source frame (camera/imu/body_link)
* `output_rate`: the output rate at which the pose data will be published.
* `roll_cam`, `pitch_cam`, `yaw_cam`, `gamma_world`: angles (in radians) that will convert pose received from `source_frame_id` to body frame, according to ENU conventions.

### Subscribed topic:
* `/tf` containing pose/odometry data.

### Published topic:
* `/vision_pose` of type [geometry_msgs/PoseStamped](http://docs.ros.org/api/geometry_msgs/html/msg/PoseStamped.html) - single pose to be sent to the FCU autopilot (ArduPilot / PX4), published at a frequency defined by `output_rate`.
* `/body_frame/path` of type [nav_msgs/Path](http://docs.ros.org/api/nav_msgs/html/msg/Path.html) - visualize trajectory of body frame in rviz.

## Example applications
### Autonomous flight with [Intel® RealSense™ Tracking Camera T265](https://www.intelrealsense.com/tracking-camera-t265/) and [ArduPilot](http://ardupilot.org/):

<p align="center"><img src="https://i.imgur.com/YT6dGMp.png"/> 

* A complete guide including installation, configuration and flight tests can be found by the following [blog posts](https://discuss.ardupilot.org/t/gsoc-2019-integration-of-ardupilot-and-vio-tracking-camera-for-gps-less-localization-and-navigation/42394).

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

## View trajectory on rviz
After running ```roslaunch vision_to_mavros t265_all_nodes.launch```, here's how to view the trajectory of t265 on rviz:
1. On host computer, open up rviz: `rosrun rviz rviz`.
2. Add [`Path`](http://docs.ros.org/api/nav_msgs/html/msg/Path.html), topic name: `/body_frame/path` to rviz. 
3. Change `Fixed Frame` to `target_frame_id`, in the case of Realsense T265: `camera_odom_frame`.

<p align="center"><img src="https://i.imgur.com/Kp8y2Ts.png"/> 

### Usage with [AprilTag](https://github.com/AprilRobotics/apriltag):
```
roslaunch vision_to_mavros apriltags_to_mavros.launch
```
This will launch `usb_cam` to capture raw images, perform rectification through `image_proc`, use `apriltag_ros` to obtain the pose of the tag in the camera frame, and finally `vision_to_mavros` to first get the pose of camera in the tag frame, transform to body frame by using camera orientation, and publish the body pose to `/mavros/vision_pose/pose` topic. Note that `mavros` should be launch separately since it has a lot of output on the terminal.


