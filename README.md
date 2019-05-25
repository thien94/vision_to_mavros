# vision_to_mavros
## Note: Ongoing project in very early stage
ROS package that listens to `/tf`, transforms the pose of `camera_frame` to `tag_frame`, then transforms to `body_frame` with the specify roll, pitch, yaw rotations that converts the camera to the body. The first transform is done [here](https://github.com/hoangthien94/vision_to_mavros/blob/18c5186abe311615678c25ad18c9049f4913c6a1/src/vision_to_mavros.cpp#L35) while the latter [here](https://github.com/hoangthien94/vision_to_mavros/blob/18c5186abe311615678c25ad18c9049f4913c6a1/src/vision_to_mavros.cpp#L66).

Example RPY for:
- Downfacing camera (Z down), X to the front: r = M_PI, p = 0, y = 0
- Downfacing camear (Z down), X to the right: r = M_PI, p = 0, y = M_PI / 2

More to come.
