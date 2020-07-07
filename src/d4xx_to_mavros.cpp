#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <string.h>

std::string debug_window_name = debug_window_name;

void imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
  try
  {
    cv::imshow(debug_window_name, cv_bridge::toCvShare(msg, "16UC1")->image);
    cv::waitKey(30);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "d4xx_to_mavros");
  ros::NodeHandle nh;
  cv::namedWindow(debug_window_name);
  cv::startWindowThread();
  image_transport::ImageTransport it(nh);
  image_transport::Subscriber sub = it.subscribe("depth_rect_image", 1, imageCallback);
  ros::spin();
  cv::destroyWindow(debug_window_name);
}