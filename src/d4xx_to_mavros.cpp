#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <string.h>

//////////////////////////////////////////////////
// Global variables
//////////////////////////////////////////////////

std::string depth_img_topic = "depth_rect_image";
std::string depth_img_format = "16UC1";

bool debug_enable = false;
std::string debug_window_name = "depth_image";

//////////////////////////////////////////////////
// Functions
//////////////////////////////////////////////////

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  if (debug_enable)
  {
    try
    {
      cv::imshow(debug_window_name, cv_bridge::toCvShare(msg, depth_img_format)->image);
      cv::waitKey(30);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("Could not convert from '%s' to '%s.", msg->encoding.c_str(), depth_img_format.c_str());
    }
  }
}

//////////////////////////////////////////////////
// Main
//////////////////////////////////////////////////

int main(int argc, char **argv)
{
  ros::init(argc, argv, "d4xx_to_mavros");

  ros::NodeHandle node;

  image_transport::ImageTransport it(node);

  image_transport::Subscriber sub = it.subscribe(depth_img_topic, 1, imageCallback);


  // The yaw angle around camera's own axis to align with body frame 
  if(node.getParam("debug_enable", debug_enable))
  {
    ROS_INFO("Get debug_enable parameter: %s", debug_enable ? "true" : "false");
  }
  else
  {
    ROS_WARN("Using default debug_enable: %s", debug_enable ? "true" : "false");
  }

  if (debug_enable)
  {
    cv::namedWindow(debug_window_name);
    cv::startWindowThread();
  }

  ros::spin();

  if (debug_enable)
  {
    cv::destroyWindow(debug_window_name);
  }
}