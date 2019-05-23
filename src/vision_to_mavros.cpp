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
  tf_listener.waitForTransform("/A3_paper", "/camera_frame", ros::Time::now(), ros::Duration(3.0));

  while (node.ok())
  {
    try
    {
      // For tf, Time(0) means "the latest available" transform in the buffer.
      ros::Time now = ros::Time(0);

      // lookupTransform(frame_2, frame_1, at_this_time, this_transform)
      //    will give the transfrom from frame_1 to frame_2
      tf_listener.lookupTransform("/A3_paper", "/camera_frame", now, transform);

      // Only publish pose when we have new transform data.
      if (last_tf_time < transform.stamp_)
      {
        last_tf_time = transform.stamp_;

        // Perform alignment between coordinate frames
        tf::Vector3 position_body = transform.getOrigin();

        tf::Quaternion quat_cam, quat_rot, quat_rot1, quat_body;
        quat_cam = transform.getRotation();

        double r = M_PI, p = 0, y = 0;
        double r1 = 0, p1 = 0, y1 = M_PI / 2;

        quat_rot = tf::createQuaternionFromRPY(r, p, y);
        quat_rot1 = tf::createQuaternionFromRPY(r1, p1, y1);
        quat_body = quat_cam * quat_rot * quat_rot1;
        quat_body.normalize();

        msg_camera_pose.header.stamp = transform.stamp_;
        msg_camera_pose.header.frame_id = transform.frame_id_;
        msg_camera_pose.pose.position.x = position_body.getX();
        msg_camera_pose.pose.position.y = position_body.getY();
        msg_camera_pose.pose.position.z = position_body.getZ();
        msg_camera_pose.pose.orientation.x = quat_body.getX();
        msg_camera_pose.pose.orientation.y = quat_body.getY();
        msg_camera_pose.pose.orientation.z = quat_body.getZ();
        msg_camera_pose.pose.orientation.w = quat_body.getW();

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