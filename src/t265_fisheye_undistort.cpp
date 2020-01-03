#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

image_transport::Publisher rect_left_pub, rect_right_pub;

void left_image_callback(const sensor_msgs::ImageConstPtr& msg)
{
  try
  {
    // cv::imshow("view_left", cv_bridge::toCvShare(msg, "bgr8")->image);
    // cv::waitKey(30);

    rect_left_pub.publish(msg);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }
}

void right_image_callback(const sensor_msgs::ImageConstPtr& msg)
{
  try
  {
    // cv::imshow("view_right", cv_bridge::toCvShare(msg, "bgr8")->image);
    // cv::waitKey(30);

    rect_right_pub.publish(msg);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "t265_fisheye_undistort");

  ros::NodeHandle nh;

  cv::startWindowThread();

  image_transport::ImageTransport it(nh);

  image_transport::Subscriber raw_left_sub = it.subscribe("/t265/fisheye1/image_raw", 1, left_image_callback);

  image_transport::Subscriber raw_right_sub = it.subscribe("/t265/fisheye2/image_raw", 1, right_image_callback);

  rect_left_pub = it.advertise("rect/left/image", 1);

  rect_right_pub = it.advertise("rect/right/image", 1);

  ros::spin();
}