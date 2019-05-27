#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PoseStamped.h>

tf::StampedTransform transform;

geometry_msgs::PoseStamped msg_camera_pose;

void align_camera_frame_to_body_frame(void);

int main(int argc, char** argv){

  ros::init(argc, argv, "vision_to_mavros");

  ros::NodeHandle node;
  
  ros::Publisher camera_pose_publisher = node.advertise<geometry_msgs::PoseStamped>("vision_pose", 10);

  tf::TransformListener tf_listener;

  // Wait for the first transform to become available. 
  tf_listener.waitForTransform("/camera_odom_frame", "/camera_link", ros::Time::now(), ros::Duration(3.0));

  ros::Time last_tf_time = ros::Time::now();

  // Limited the rate of publishing data, otherwise the other telemetry port might be flooded
  ros::Rate rate(30.0);

  while (node.ok())
  {
    try
    {
      // For tf, Time(0) means "the latest available" transform in the buffer.
      ros::Time now = ros::Time(0);

      // lookupTransform(frame_2, frame_1, at_this_time, this_transform)
      //    will give the transfrom from frame_1 to frame_2
      tf_listener.lookupTransform("/camera_odom_frame", "/camera_link", now, transform);

      // Only publish pose when we have new transform data.
      if (last_tf_time < transform.stamp_)
      {
        last_tf_time = transform.stamp_;

        align_camera_frame_to_body_frame();

        camera_pose_publisher.publish(msg_camera_pose);
      }
    }
    catch (tf::TransformException ex)
    {
        ROS_WARN("%s",ex.what());
        ros::Duration(1.0).sleep();
    }

    rate.sleep();
  }
  return 0;
};


void align_camera_frame_to_body_frame()
{
  static tf::Vector3 position_body;

  static tf::Quaternion quat_cam, quat_rot_x, quat_rot_y, quat_rot_z, quat_body;

  // Perform alignment between camera frame and body frame (ENU)
  quat_cam = transform.getRotation();

  // Rotation from camera to body frame.
  // In each step, rotate the camera frame around its own axis (roll = x, pitch = y, yaw = z)
  // so that at the end, the camera frame is aligned with the body frame (ENU, x forward, y to the left, z upwards)
  //    Step 1: Take a look at the camera frame in the world frame
  //    Step 2: Rotate camera frame around its x axis -> roll angle (radians)
  //    Step 3: From the new frame, rotate camera frame around its current y axis -> pitch angle (radians)
  //    Step 4: From the new frame, rotate camera frame around its current z axis -> yaw angle (radians)
  // Examples some camera orientation with respect to body frame:
  //    Downfacing (Z down), X to the front: r = M_PI, p = 0, y = 0
  //    Downfacing (Z down), X to the right: r = M_PI, p = 0, y = M_PI / 2
  static double r = 0, p = 0, y = 0;

  quat_rot_x = tf::createQuaternionFromRPY(r, 0, 0);
  quat_rot_y = tf::createQuaternionFromRPY(0, p, 0);
  quat_rot_z = tf::createQuaternionFromRPY(0, 0, y);

  quat_body = quat_cam * quat_rot_x * quat_rot_y * quat_rot_z;
  quat_body.normalize();

  position_body = transform.getOrigin();

  msg_camera_pose.header.stamp = transform.stamp_;
  msg_camera_pose.header.frame_id = transform.frame_id_;
  msg_camera_pose.pose.position.x = position_body.getX();
  msg_camera_pose.pose.position.y = position_body.getY();
  msg_camera_pose.pose.position.z = position_body.getZ();
  msg_camera_pose.pose.orientation.x = quat_body.getX();
  msg_camera_pose.pose.orientation.y = quat_body.getY();
  msg_camera_pose.pose.orientation.z = quat_body.getZ();
  msg_camera_pose.pose.orientation.w = quat_body.getW();
}
