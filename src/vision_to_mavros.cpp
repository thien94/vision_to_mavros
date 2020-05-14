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

  geometry_msgs::PoseStamped msg_body_pose, msg_leica_pose;
  ros::Publisher camera_pose_pub = node.advertise<geometry_msgs::PoseStamped>("camera_link_pose", 10);
  ros::Publisher leica_pose_pub = node.advertise<geometry_msgs::PoseStamped>("leica_pose", 10);

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
        static tf::Vector3 position_orig, position_body;

        static tf::Quaternion quat_orig, quat_body, quat_offset;

        // Create PoseStamped message to be sent for leica
        tf_listener.lookupTransform("/map", "leica_rel", now, transform);
        position_orig = transform.getOrigin();
        quat_orig = transform.getRotation();
        msg_leica_pose.header.stamp = transform.stamp_;
        msg_leica_pose.header.frame_id = transform.frame_id_;
        msg_leica_pose.pose.position.x = position_orig.getX();
        msg_leica_pose.pose.position.y = position_orig.getY();
        msg_leica_pose.pose.position.z = position_orig.getZ();
        msg_leica_pose.pose.orientation.x = quat_orig.getX();
        msg_leica_pose.pose.orientation.y = quat_orig.getY();
        msg_leica_pose.pose.orientation.z = quat_orig.getZ();
        msg_leica_pose.pose.orientation.w = quat_orig.getW();

        // Publish pose of body frame in world frame
        leica_pose_pub.publish(msg_leica_pose);

      // lookupTransform(frame_2, frame_1, at_this_time, this_transform)
      //    will give the transfrom from frame_1 to frame_2
      tf_listener.lookupTransform(target_frame_id, source_frame_id, now, transform);

      // Only publish pose messages when we have new transform data.
      // if (last_tf_time < transform.stamp_)
      {
        last_tf_time = transform.stamp_;
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
        // static tf::Matrix3x3 rot_mat (
        //    0.903,  0.206, -0.376,
        //   -0.234,  0.972, -0.03 ,
        //    0.36 ,  0.116,  0.926);
        // static tf::Vector3 position_offset (-0.843, -1.992, -1.596);

        // // For SC01/HH01
        // static tf::Matrix3x3 rot_mat (
        //    0.997,  0.083, -0.011,
        //   -0.083,  0.996,  0.021,
        //    0.012, -0.02 ,  1.   );
        // static tf::Vector3 position_offset (-0.043, -0.032, 0.256);

        // // For SC04/HH06
        // static tf::Matrix3x3 rot_mat (
        //    0.993,  0.107,  0.051,
        //   -0.106,  0.994, -0.012,
        //   -0.052,  0.007,  0.999);
        // static tf::Vector3 position_offset (0.012, -0.14, 0.461);

        // // For AuRO/MH02
        // static tf::Matrix3x3 rot_mat (
        // -0.925,  0.045, -0.376,
        // -0.045, -0.999, -0.009,
        // -0.376,  0.008,  0.926);
        // static tf::Vector3 position_offset (4.595,-1.418, 1.07);

        // // For AuRO/VC01_001_much_bigger
        // static tf::Matrix3x3 rot_mat (
        //   0.983,  0.035,  0.183,
        //  -0.031,  0.999, -0.026,
        //  -0.183,  0.02 ,  0.983);
        // static tf::Vector3 position_offset (0.031, -0.001, 0.495);

        // // For AuRO/VC01_003_bigger
        // static tf::Matrix3x3 rot_mat (
        // 0.995, -0.077,  0.07 ,
        // 0.075,  0.996,  0.038,
        // -0.073, -0.033,  0.997);
        // static tf::Vector3 position_offset (0.046, 0.014, 0.265);

        // // For AuRO/VC01_004_smaller
        // static tf::Matrix3x3 rot_mat (
        //   0.989,  0.049,  0.142,
        //   -0.039,  0.996, -0.075,
        //   -0.145,  0.069,  0.987);
        // static tf::Vector3 position_offset (0.03, -0.03, 0.655);

        // // For AuRO/VC01_005_smaller
        // static tf::Matrix3x3 rot_mat (
        //   0.995, -0.076,  0.068,
        //   0.073,  0.997,  0.04 ,
        //  -0.071, -0.035,  0.997);
        // static tf::Vector3 position_offset (0.048, 0.013, 0.263);

        // // For AuRO/proposed Mono UWB/Cor_01
        // static tf::Matrix3x3 rot_mat (
        //   0.995,  0.011,  0.098,
        //  -0.022,  0.994,  0.105,
        //  -0.096, -0.107,  0.99 );
        // static tf::Vector3 position_offset (0.0, 0.0, 0.276);
        // static tf::Vector3 position_anchor_offset (0,-0.045, 0.276);

        // // For AuRO/t265/Cor_01
        // static tf::Matrix3x3 rot_mat (
        //   1.   , -0.017, -0.009,
        //   0.018,  0.663,  0.749,
        //  -0.007, -0.749,  0.663);
        // static tf::Vector3 position_offset (0.0, 0.0, 0.0);
        // static tf::Vector3 position_anchor_offset (0,-0.045, 0.276);

        // // For AuRO/vins/Cor_01
        // static tf::Matrix3x3 rot_mat (
        //   0.013,  1.   ,  0.008,
        //  -0.859,  0.007,  0.512,
        //   0.512, -0.014,  0.859);
        // static tf::Vector3 position_offset (0.0, 0.0, 0.0);
        // static tf::Vector3 position_anchor_offset (0,-0.045, 0.276);

        // // For AuRO/Proposed/Cor_02_
        // static tf::Matrix3x3 rot_mat (
        //   0.996, -0.028,  0.081,
        //   0.022,  0.996,  0.084,
        //  -0.083, -0.082,  0.993);
        // static tf::Vector3 position_offset (0, 0, 0.149);
        // static tf::Vector3 position_anchor_offset (0,-0.045, 0.276);

        // // For AuRO/t265/Cor_02_
        // static tf::Matrix3x3 rot_mat (
        //   1.   ,  0.005, -0.013,
        //  -0.005,  1.   ,  0.003,
        //   0.013, -0.003,  1.   );
        // static tf::Vector3 position_offset (0.0, 0.0, 0);
        // static tf::Vector3 position_anchor_offset (0,-0.045, 0.276);

        // // For AuRO/VINS/Cor_02_
        // static tf::Matrix3x3 rot_mat (
        // -0.009,  1.   , -0.005,
        // -0.888, -0.01 , -0.46 ,
        // -0.46 , -0.   ,  0.888);
        // static tf::Vector3 position_offset (0.0, 0, -0.223);
        // static tf::Vector3 position_anchor_offset (0,-0.045, 0.276);

        // // For AuRO/Proposed/OP_03
        // static tf::Matrix3x3 rot_mat (
        //   0.985,  0.105, -0.137,
        //  -0.112,  0.992, -0.049,
        //   0.131,  0.064,  0.989);
        // static tf::Vector3 position_offset (-0.082, 0.6, 0.498);
        // static tf::Vector3 position_anchor_offset (0,-0.045, 0.276);

        // // For AuRO/t265/OP_03
        // static tf::Matrix3x3 rot_mat (
        //   1.   ,  0.005, -0.013,
        //  -0.005,  1.   ,  0.003,
        //   0.013, -0.003,  1.   );
        // static tf::Vector3 position_offset (0,0,0);
        // static tf::Vector3 position_anchor_offset (0,-0.045, 0.276);

        // // For AuRO/VINS/OP_03
        // static tf::Matrix3x3 rot_mat (
        //   -0.002, -1.   , -0.003,
        //   1.   , -0.002,  0.003,
        //  -0.003, -0.003,  1.   );
        // static tf::Vector3 position_offset (-0.16, 0.032, 0.009);
        // static tf::Vector3 position_anchor_offset (0,-0.045, 0.276);

        // // For RAL2020/Cor_01/VIU all/
        static tf::Matrix3x3 rot_mat (
          1.   ,  0.023,  0.013,
        -0.026,  0.947,  0.32 ,
        -0.005, -0.321,  0.9479);
        static tf::Vector3 position_offset (0.0, 0.0, 0);
        static tf::Vector3 position_anchor_offset (0,-0.045, 0.276);

        // // For RAL2020/Cor_01/VINS_Mono/
        // static tf::Matrix3x3 rot_mat (
        //   1.   ,  0.015,  0.016,
        //  -0.022,  0.67 ,  0.742,
        //   0.   , -0.742,  0.67 );
        // static tf::Vector3 position_offset (0.0, 0.0, 0);
        // static tf::Vector3 position_anchor_offset (0,-0.045, 0.276);


        // 1) Rotation from original world frame to world frame with y forward.
        // 2) Rotation from camera to body frame.
        // See the full rotation matrix at https://en.wikipedia.org/wiki/Rotation_matrix#Basic_rotations
        quat_offset.setRPY(0,0,0);
        position_orig = transform.getOrigin();
        quat_orig = transform.getRotation();

        position_body = rot_mat * position_orig + position_offset;
        quat_body = quat_orig;
        // quat_body = quat_offset * quat_orig;
        // quat_body.normalize();

        static tf::TransformBroadcaster br;
        static tf::Transform tf_aligned_ground_truth;
        tf_aligned_ground_truth.setOrigin(position_body);
        tf_aligned_ground_truth.setRotation(quat_body);
        br.sendTransform(tf::StampedTransform(
          tf_aligned_ground_truth,
          last_tf_time,
          "map",
          output_frame_id));
        
        // Create PoseStamped message to be sent for uwbs
        msg_body_pose.header.stamp = transform.stamp_;
        msg_body_pose.header.frame_id = "map";
        msg_body_pose.pose.position.x = position_body.getX();
        msg_body_pose.pose.position.y = position_body.getY();
        msg_body_pose.pose.position.z = position_body.getZ();
        msg_body_pose.pose.orientation.x = quat_body.getX();
        msg_body_pose.pose.orientation.y = quat_body.getY();
        msg_body_pose.pose.orientation.z = quat_body.getZ();
        msg_body_pose.pose.orientation.w = quat_body.getW();

        // Publish pose of body frame in world frame
        camera_pose_pub.publish(msg_body_pose);

        // Publish position of uwb 100 and 102
        tf_aligned_ground_truth.setIdentity();

        // Republish anchor 100 true
        tf_listener.lookupTransform(target_frame_id, "vicon/uwb100", now, transform);
        position_orig = transform.getOrigin();
        tf_aligned_ground_truth.setOrigin(position_orig);
        br.sendTransform(tf::StampedTransform(
          tf_aligned_ground_truth,
          last_tf_time,
          "map",
          "true_100"));

        // Publish aligned 100 estimate
        tf_listener.lookupTransform(target_frame_id, "uwb100", now, transform);
        position_orig = transform.getOrigin();
        position_body = rot_mat * position_orig + position_anchor_offset;
        tf_aligned_ground_truth.setOrigin(position_body);
        br.sendTransform(tf::StampedTransform(
          tf_aligned_ground_truth,
          last_tf_time,
          "map",
          "estimate_100"));

        // Republish anchor 102 true
        tf_listener.lookupTransform(target_frame_id, "vicon/uwb102", now, transform);
        position_orig = transform.getOrigin();
        tf_aligned_ground_truth.setOrigin(position_orig);
        br.sendTransform(tf::StampedTransform(
          tf_aligned_ground_truth,
          last_tf_time,
          "map",
          "true_102"));

        // Publish aligned 102 estimate
        tf_listener.lookupTransform(target_frame_id, "uwb102", now, transform);
        position_orig = transform.getOrigin();
        position_body = rot_mat * position_orig + position_anchor_offset;
        tf_aligned_ground_truth.setOrigin(position_body);
        br.sendTransform(tf::StampedTransform(
          tf_aligned_ground_truth,
          last_tf_time,
          "map",
          "estimate_102"));
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