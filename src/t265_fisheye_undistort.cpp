#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

using namespace cv;
using namespace std;

// Declare all the calibration matrices as Mat variables.
Mat lmapx, lmapy, rmapx, rmapy;

FileStorage config_file;

image_transport::Publisher pub_img_rect_left;
image_transport::Publisher pub_img_rect_right;

// This function undistorts and rectifies the src image into dst. 
// The homographic mappings lmapx, lmapy, rmapx, and rmapy are found from OpenCV’s initUndistortRectifyMap function.
void undistortRectifyImage(Mat& src, Mat& dst, int left = 1)
{
  if (left == 1) 
  {
    remap(src, dst, lmapx, lmapy, cv::INTER_LINEAR);
  } 
  else 
  {
    remap(src, dst, rmapx, rmapy, cv::INTER_LINEAR);
  }
}

// This function computes all the projection matrices and the rectification transformations 
// using the stereoRectify and initUndistortRectifyMap functions respectively.
// See documentation for stereoRectify: https://docs.opencv.org/2.4/modules/calib3d/doc/camera_calibration_and_3d_reconstruction.html#stereorectify
void init_rectification_map(FileStorage& config_file) 
{
  Mat Q, P1, P2;
  Mat R1, R2, K1, K2, D1, D2, R;
  Vec3d T;

  config_file["K1"] >> K1;
  config_file["D1"] >> D1;
  config_file["K2"] >> K2;
  config_file["D2"] >> D2;
  config_file["R"]  >> R;
  config_file["T"]  >> T;

  // The resolution of the input images used for stereo calibration.
  Size input_img_size(848, 800);

  // The resolution of the output rectified images. Lower resolution images require less computation time.
  Size output_img_size(800, 800);
  double alpha = 0.0;

  stereoRectify(K1, D1, K2, D2, 
                input_img_size, 
                R, T, 
                R1, R2, P1, P2, 
                Q,
                CV_CALIB_ZERO_DISPARITY, 
                alpha, 
                output_img_size);
 
  fisheye::initUndistortRectifyMap(K1, D1, R1, P1, output_img_size, CV_32FC1, lmapx, lmapy);
  fisheye::initUndistortRectifyMap(K2, D2, R2, P2, output_img_size, CV_32FC1, rmapx, rmapy);

  ROS_INFO("Initialization for rectification mapping complete. Publishing rectified images when raw images arrive.");
}

// This callback function takes a raw stereo (left) image as input, 
// then it undistorts and rectifies the image using the void undistortRectifyImage
// function defined above and publishes on the rectified image topic using pub_img_left.
// Similarly img_raw_callback_right is defined for the right stereo image.
void img_raw_callback_left(const sensor_msgs::ImageConstPtr& msg)
{
  try
  {
    Mat tmp = cv_bridge::toCvShare(msg, "bgr8")->image;

    if (tmp.empty())
    {
      ROS_ERROR("Could not convert left image msg to OpenCV-compatible CvImage");
      return;
    }

    Mat dst;

    undistortRectifyImage(tmp, dst, 1);

    sensor_msgs::ImagePtr rect_left_img = cv_bridge::CvImage(msg->header, "bgr8", dst).toImageMsg();

    pub_img_rect_left.publish(rect_left_img);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Error when trying to rectify left images: %s", e.what());
  }
}

// Similar to img_raw_callback_left, but for the right stereo image.
void img_raw_callback_right(const sensor_msgs::ImageConstPtr& msg)
{
  try
  {
    Mat tmp = cv_bridge::toCvShare(msg, "bgr8")->image;

    if (tmp.empty())
    {
      ROS_ERROR("Could not convert right image msg to OpenCV-compatible CvImage");
      return;
    }

    Mat dst;

    undistortRectifyImage(tmp, dst, 2);

    sensor_msgs::ImagePtr rect_right_img = cv_bridge::CvImage(msg->header, "bgr8", dst).toImageMsg();

    pub_img_rect_right.publish(rect_right_img);
  }
  catch (cv_bridge::Exception& e)
  {
    ROS_ERROR("Error when trying to rectify right images: %s", e.what());
  }
}

// The stereo calibration information is read into config_file. 
// sub_img_raw_left/right subscribes to the topics publishing the raw stereo image data,
// subsequently rectify and publish output images.
int main(int argc, char **argv)
{
  ros::init(argc, argv, "t265_fisheye_undistort");

  ros::NodeHandle nh;

  image_transport::ImageTransport it(nh);

  // Read the input parameters
  config_file = FileStorage(argv[1], FileStorage::READ);
  init_rectification_map(config_file);

  // The raw stereo images should be published by a node which publishes messages of the type sensor_msgs/Image
  image_transport::Subscriber sub_img_raw_left  = it.subscribe("/t265/fisheye1/image_raw", 1, img_raw_callback_left);
  image_transport::Subscriber sub_img_raw_right = it.subscribe("/t265/fisheye2/image_raw", 1, img_raw_callback_right);

  // The rectified output images are published to the following topics:
  pub_img_rect_left  = it.advertise("/t265_rect/image_left",  1);
  pub_img_rect_right = it.advertise("/t265_rect/image_right", 1);

  ros::spin();
}