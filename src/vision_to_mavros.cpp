#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Path.h>

#include <string.h>

int main(int argc, char** argv){

  ros::init(argc, argv, "vision_to_mavros");

  ros::NodeHandle node;
  
  ros::Publisher camera_pose_publisher = node.advertise<geometry_msgs::PoseStamped>("vision_pose", 10);

  ros::Publisher body_path_pubisher = node.advertise<nav_msgs::Path>("body_frame/path", 1);

  tf::TransformListener tf_listener;

  tf::StampedTransform transform;

  geometry_msgs::PoseStamped msg_body_pose;

  nav_msgs::Path body_path;

  std::string target_frame_id = "/camera_odom_frame";

  std::string source_frame_id = "/camera_link";

  double output_rate = 30, roll_cam = 0, pitch_cam = 0, yaw_cam = 1.5707963, gamma_world = -1.5707963, scale_factor = 1.0;

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

    // The rate at which we wish to publish final pose data
    if(node.getParam("output_rate", output_rate))
    {
      ROS_INFO("Get output_rate parameter: %f", output_rate);
    }
    else
    {
      ROS_WARN("Using default output_rate: %f", output_rate);
    }

    // The rotation around z axis between original world frame and target world frame, assuming the z axis needs not to be changed
    // In this case, target world frame has y forward, x to the right and z upwards (ENU as ROS dictates)
    if(node.getParam("gamma_world", gamma_world))
    {
      ROS_INFO("Get gamma_world parameter: %f", gamma_world);
    }
    else
    {
      ROS_WARN("Using default gamma_world: %f", gamma_world);
    }

    // The roll angle around camera's own axis to align with body frame 
    if(node.getParam("roll_cam", roll_cam))
    {
      ROS_INFO("Get roll_cam parameter: %f", roll_cam);
    }
    else
    {
      ROS_WARN("Using default roll_cam: %f", roll_cam);
    }

    // The pitch angle around camera's own axis to align with body frame 
    if(node.getParam("pitch_cam", pitch_cam))
    {
      ROS_INFO("Get pitch_cam parameter: %f", pitch_cam);
    }
    else
    {
      ROS_WARN("Using default pitch_cam: %f", pitch_cam);
    }

    // The yaw angle around camera's own axis to align with body frame 
    if(node.getParam("yaw_cam", yaw_cam))
    {
      ROS_INFO("Get yaw_cam parameter: %f", yaw_cam);
    }
    else
    {
      ROS_WARN("Using default yaw_cam: %f", yaw_cam);
    }

    // The scale factor that will be applied to position
    if(node.getParam("scale_factor", scale_factor))
    {
      ROS_INFO("Get scale_factor parameter: %f", scale_factor);
    }
    else
    {
      ROS_WARN("Using default scale_factor: %f", scale_factor);
    }
  }

  // Wait for the first transform to become available. 
  tf_listener.waitForTransform(target_frame_id, source_frame_id, ros::Time::now(), ros::Duration(3.0));

  ros::Time last_tf_time = ros::Time::now();

  // Limited the rate of publishing data, otherwise the other telemetry port might be flooded
  ros::Rate rate(output_rate);

  while (node.ok())
  {
    try
    {
      // For tf, Time(0) means "the latest available" transform in the buffer.
      ros::Time now = ros::Time(0);

      // lookupTransform(frame_2, frame_1, at_this_time, this_transform)
      //    will give the transfrom from frame_1 to frame_2
      tf_listener.lookupTransform(target_frame_id, source_frame_id, now, transform);

      // Only publish pose messages when we have new transform data.
      if (last_tf_time < transform.stamp_)
      {
        last_tf_time = transform.stamp_;

        static tf::Vector3 position_orig, position_body;

        static tf::Quaternion quat_cam, quat_cam_to_body_x, quat_cam_to_body_y, quat_cam_to_body_z, quat_rot_z, quat_body;

        // 1) Rotation from original world frame to world frame with y forward.
        // See the full rotation matrix at https://en.wikipedia.org/wiki/Rotation_matrix#Basic_rotations
        position_orig = transform.getOrigin();

        node.getParam("scale_factor", scale_factor);

        position_body.setX(( cos(gamma_world) * position_orig.getX() + sin(gamma_world) * position_orig.getY()) * scale_factor);
        position_body.setY((-sin(gamma_world) * position_orig.getX() + cos(gamma_world) * position_orig.getY()) * scale_factor);
        position_body.setZ((position_orig.getZ()) * scale_factor);

        // 2) Rotation from camera to body frame.
        quat_cam = transform.getRotation();

        quat_cam_to_body_x = tf::createQuaternionFromRPY(roll_cam, 0, 0);
        quat_cam_to_body_y = tf::createQuaternionFromRPY(0, pitch_cam, 0);
        quat_cam_to_body_z = tf::createQuaternionFromRPY(0, 0, yaw_cam);
        
        // 3) Rotate body frame 90 degree (align body x with world y at launch)
        quat_rot_z = tf::createQuaternionFromRPY(0, 0, -gamma_world);

        quat_body = quat_rot_z * quat_cam * quat_cam_to_body_x * quat_cam_to_body_y * quat_cam_to_body_z;
        quat_body.normalize();

        // Create PoseStamped message to be sent
        msg_body_pose.header.stamp = transform.stamp_;
        msg_body_pose.header.frame_id = transform.frame_id_;
        msg_body_pose.pose.position.x = position_body.getX();
        msg_body_pose.pose.position.y = position_body.getY();
        msg_body_pose.pose.position.z = position_body.getZ();
        msg_body_pose.pose.orientation.x = quat_body.getX();
        msg_body_pose.pose.orientation.y = quat_body.getY();
        msg_body_pose.pose.orientation.z = quat_body.getZ();
        msg_body_pose.pose.orientation.w = quat_body.getW();

        // Publish pose of body frame in world frame
        camera_pose_publisher.publish(msg_body_pose);

        // Publish trajectory path for visualization
        body_path.header.stamp = msg_body_pose.header.stamp;
        body_path.header.frame_id = msg_body_pose.header.frame_id;
        body_path.poses.push_back(msg_body_pose);
        body_path_pubisher.publish(body_path);
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