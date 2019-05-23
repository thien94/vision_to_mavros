#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PoseStamped.h>

int main(int argc, char** argv){

  ros::init(argc, argv, "vision_to_mavros");

  ros::NodeHandle node;

  tf::StampedTransform transform;
  tf::TransformListener tf_listener;

  geometry_msgs::PoseStamped msg_camera_pose;
  ros::Publisher camera_pose_publisher = node.advertise<geometry_msgs::PoseStamped>("vision_pose", 10);

  ros::Time last_tf_time;

  // Wait for up to timeout for the first transform to become available. 
  tf_listener.waitForTransform("/A3_paper", "/camera_frame", ros::Time::now(), ros::Duration(10.0));

  while (node.ok())
  {
    try
    {
      // For tf, Time(0) means "the latest available" transform in the buffer.
      ros::Time now = ros::Time(0);

      // lookupTransform(frame_2, frame_1, at_this_time, this_transform)
      //    We want the transfrom from camera_frame (frame_1) to tag_frame (frame_2)
      tf_listener.lookupTransform("/A3_paper", "/camera_frame", now, transform);

      // Only publish pose when we have new transform data.
      if (last_tf_time < transform.stamp_)
      {
        last_tf_time = transform.stamp_;

        // Perform orientation to align the camera frames and mavros
        tf::Vector3 camera_position = transform.getOrigin();
        tf::Quaternion camera_quaternion = transform.getRotation();

        msg_camera_pose.header.stamp = transform.stamp_;
        msg_camera_pose.header.frame_id = transform.frame_id_;
        msg_camera_pose.pose.position.x = camera_position.getX();
        msg_camera_pose.pose.position.y = camera_position.getY();
        msg_camera_pose.pose.position.z = camera_position.getZ();
        msg_camera_pose.pose.orientation.x = camera_quaternion.getY();
        msg_camera_pose.pose.orientation.y = camera_quaternion.getX();
        msg_camera_pose.pose.orientation.z = camera_quaternion.getZ();
        msg_camera_pose.pose.orientation.w = camera_quaternion.getW();

        camera_pose_publisher.publish(msg_camera_pose);
      }
    }
    catch (tf::TransformException ex)
    {
        ROS_WARN("%s",ex.what());
        ros::Duration(1.0).sleep();
    }
  }
  return 0;
};