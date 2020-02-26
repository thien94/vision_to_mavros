#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>
#include <mavros_msgs/LandingTarget.h>

#include <string.h>

int main(int argc, char** argv)
{
  ros::init(argc, argv, "vision_to_mavros");

  ros::NodeHandle node;
  
  //////////////////////////////////////////////////
  // Variables for precision navigation
  //////////////////////////////////////////////////

  ros::Publisher body_path_pubisher = node.advertise<nav_msgs::Path>("vicon/path", 1);

  nav_msgs::Path body_path;

  tf::TransformListener tf_listener;

  tf::StampedTransform transform;

  std::string target_frame_id = "/camera_odom_frame";

  std::string source_frame_id = "/camera_link";

  std::string output_frame_id = "/leica";

  geometry_msgs::PoseStamped msg_body_pose;

  double output_rate = 20, roll_cam = 0, pitch_cam = 0, yaw_cam = 1.5707963, gamma_world = -1.5707963;

  // Read parameters from launch file, including: target_frame_id, source_frame_id, output_rate
  {
    // The frame in which we find the transform into, the original "world" frame
    if(node.getParam("target_frame_id", target_frame_id))
    {
      ROS_INFO("Get target_frame_id parameter: %s", target_frame_id.c_str());
    }
    else
    {
      ROS_WARN("Using default target_frame_id: %s", target_frame_id.c_str());
    }

    // The frame for which we find the tranform to target_frame_id, the original "camera" frame
    if(node.getParam("source_frame_id", source_frame_id))
    {
      ROS_INFO("Get source_frame_id parameter: %s", source_frame_id.c_str());
    }
    else
    {
      ROS_WARN("Using default source_frame_id: %s", source_frame_id.c_str());
    }

    // The frame for which we find the tranform to target_frame_id, the original "camera" frame
    if(node.getParam("output_frame_id", output_frame_id))
    {
      ROS_INFO("Get output_frame_id parameter: %s", output_frame_id.c_str());
    }
    else
    {
      ROS_WARN("Using default output_frame_id: %s", output_frame_id.c_str());
    }

    // The rate at which we wish to publish final pose data
    if(node.getParam("output_rate", output_rate))
    {
      ROS_INFO("Get output_rate parameter: %f", output_rate);
    }
    else
    {
      ROS_WARN("Using default output_rate: %f", output_rate);
    }

  }

  //////////////////////////////////////////////////
  // Wait for the first transform to become available.
  //////////////////////////////////////////////////
  tf_listener.waitForTransform(target_frame_id, source_frame_id, ros::Time::now(), ros::Duration(3.0));

  ros::Time last_tf_time = ros::Time::now();

  // Limit the rate of publishing data, otherwise the other telemetry port might be flooded
  ros::Rate rate(output_rate);

  while (node.ok())
  {
    // For tf, Time(0) means "the latest available" transform in the buffer.
    ros::Time now = ros::Time(0);

    //////////////////////////////////////////////////
    // Publish vision_pose_estimate message if transform is available
    //////////////////////////////////////////////////
    try
    {
      // lookupTransform(frame_2, frame_1, at_this_time, this_transform)
      //    will give the transfrom from frame_1 to frame_2
      tf_listener.lookupTransform(target_frame_id, source_frame_id, now, transform);

      // Only publish pose messages when we have new transform data.
      // if (last_tf_time < transform.stamp_)
      {
        last_tf_time = transform.stamp_;

        static tf::Vector3 position_orig, position_body;

        static tf::Quaternion quat_orig, quat_body, quat_offset = tf::createIdentityQuaternion();

        // For anchor_at_leica_MH01_2020-02-24-20-19-39.bag 
        // static tf::Matrix3x3 rot_mat (
        //   -0.896,  0.307, -0.32,
        //   -0.334, -0.942,  0.032,
        //   -0.291,  0.136,  0.947);
        // static tf::Vector3 position_offset (5.174, -0.143, 0.591);

        // // For anchor_at_leica_MH02
        // static tf::Matrix3x3 rot_mat (
        //   -0.925,  0.045, -0.376,
        //   -0.045, -0.999, -0.009,
        //   -0.376,  0.008,  0.926);
        // static tf::Vector3 position_offset (4.595, -1.418, 1.07);

        // // For anchor_at_leica_MH03
        // static tf::Matrix3x3 rot_mat (
        //   -0.848,  0.423, -0.319,
        //   -0.44 , -0.898, -0.018,
        //   -0.294,  0.125,  0.948);
        // static tf::Vector3 position_offset (4.761, 0.654, 0.659);

        // // For anchor_at_leica_MH04
        // static tf::Matrix3x3 rot_mat (
        //    0.692,  0.635, -0.344,
        //   -0.682,  0.732, -0.02 ,
        //    0.238,  0.248,  0.939);
        // static tf::Vector3 position_offset (-2.096, 4.461, -1.764);

        // // For anchor_at_leica_MH05
        // static tf::Matrix3x3 rot_mat (
        //    0.557,  0.75 , -0.358,
        //   -0.815,  0.575, -0.063,
        //    0.159,  0.327,  0.932);
        // static tf::Vector3 position_offset (-1.549, 4.739, -1.273);

        // // For anchor_at_leica_V101
        static tf::Matrix3x3 rot_mat (
           0.903,  0.206, -0.376,
          -0.234,  0.972, -0.03 ,
           0.36 ,  0.116,  0.926);
        static tf::Vector3 position_offset (-0.843, -1.992, -1.596);

        // 1) Rotation from original world frame to world frame with y forward.
        // 2) Rotation from camera to body frame.
        // See the full rotation matrix at https://en.wikipedia.org/wiki/Rotation_matrix#Basic_rotations
        rot_mat.getRotation(quat_offset);

        position_orig = transform.getOrigin();
        quat_orig = transform.getRotation();

        position_body = rot_mat * position_orig + position_offset;
        quat_body = quat_orig;
        
        static tf::TransformBroadcaster br;
        static tf::Transform tf_aligned_ground_truth;
        tf_aligned_ground_truth.setOrigin(position_body);
        tf_aligned_ground_truth.setRotation(quat_body);
        br.sendTransform(tf::StampedTransform(
          tf_aligned_ground_truth,
          last_tf_time,
          "map",
          output_frame_id));
        
        // To-do: test fixed length path publisher
        // msg_body_pose.header.stamp = transform.stamp_;
        // msg_body_pose.header.frame_id = "/vicon";
        // msg_body_pose.pose.position.x = position_body.getX();
        // msg_body_pose.pose.position.y = position_body.getY();
        // msg_body_pose.pose.position.z = position_body.getZ();
        // msg_body_pose.pose.orientation.x = quat_body.getX();
        // msg_body_pose.pose.orientation.y = quat_body.getY();
        // msg_body_pose.pose.orientation.z = quat_body.getZ();
        // msg_body_pose.pose.orientation.w = quat_body.getW();

        // // Publish trajectory path for visualization
        // body_path.header.stamp = msg_body_pose.header.stamp;
        // body_path.header.frame_id = "/map";
        // body_path.poses.push_back(msg_body_pose);

        // // if (body_path.poses.size() > 10)
        // {
        //   body_path.poses.clear();
        // }

        // body_path_pubisher.publish(body_path);
      }
    }
    catch (tf::TransformException ex)
    {
      ROS_WARN("%s",ex.what());
      ros::Duration(1.0).sleep();
    }

    // Repeat
    rate.sleep();
  }
  return 0;
};