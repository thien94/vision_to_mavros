#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace cv;
using namespace std;

enum class CameraSide { LEFT, RIGHT };

//////////////////////////////////////////////////
// Declare all the calibration matrices as Mat variables.
//////////////////////////////////////////////////
Mat lmapx, lmapy, rmapx, rmapy;

image_transport::Publisher pub_img_rect_left, pub_img_rect_right;

sensor_msgs::CameraInfo output_camera_info_left, output_camera_info_right;

ros::Publisher left_camera_info_output_pub, right_camera_info_output_pub;

//////////////////////////////////////////////////
// This function computes all the projection matrices and the rectification transformations 
// using the stereoRectify and initUndistortRectifyMap functions respectively.
// See documentation for stereoRectify: https://docs.opencv.org/2.4/modules/calib3d/doc/camera_calibration_and_3d_reconstruction.html#stereorectify
//////////////////////////////////////////////////
void init_rectification_map(string param_file_path) 
{
  Mat Q, P1, P2;
  Mat R1, R2, K1, K2, D1, D2, R;
  Vec3d T;
  Vec2d size_input, size_output;

  FileStorage param_file = FileStorage(param_file_path, FileStorage::READ);

  param_file["K1"] >> K1;
  param_file["D1"] >> D1;
  param_file["K2"] >> K2;
  param_file["D2"] >> D2;
  param_file["R"]  >> R;
  param_file["T"]  >> T;
  param_file["input"]  >> size_input;
  param_file["output"] >> size_output;

  // The resolution of the input images used for stereo calibration.
  Size input_img_size(size_input[0], size_input[1]);

  // The resolution of the output rectified images. Lower resolution images require less computation time.
  Size output_img_size(size_output[0], size_output[1]);
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

  // Copy the parameters for rectified images to the camera_info messages
  output_camera_info_left.width   = size_output[0];
  output_camera_info_left.height  = size_output[1];
  output_camera_info_left.D       = vector<double>(5, 0);

  output_camera_info_right.width  = size_output[0];
  output_camera_info_right.height = size_output[1];
  output_camera_info_right.D      = vector<double>(5, 0);

  for (int i = 0; i < 9; i++)
  {
    output_camera_info_left.K[i]  = K1.at<double>(i);
    output_camera_info_right.K[i] = K2.at<double>(i);
    output_camera_info_left.R[i]  = R1.at<double>(i);
    output_camera_info_right.R[i] = R2.at<double>(i);
  }  
  for (int i = 0; i < 12; i++)
  {
    output_camera_info_left.P[i]  = P1.at<double>(i);
    output_camera_info_right.P[i] = P2.at<double>(i);
  }
  
  ROS_INFO("Initialization complete. Publishing rectified images and camera_info when raw images arrive...");
}

//////////////////////////////////////////////////
// This function undistorts and rectifies the src image into dst. 
// The homographic mappings lmapx, lmapy, rmapx, and rmapy are found from OpenCV’s initUndistortRectifyMap function.
//////////////////////////////////////////////////
void undistort_rectify_image(Mat& src, Mat& dst, const CameraSide& side)
{
  if (side == CameraSide::LEFT) 
  {
    remap(src, dst, lmapx, lmapy, cv::INTER_LINEAR);
  } 
  else 
  {
    remap(src, dst, rmapx, rmapy, cv::INTER_LINEAR);
  }
}

//////////////////////////////////////////////////
// This callback function takes a pair of raw stereo images as inputs, 
// then undistorts and rectifies the images using the undistort_rectify_image function 
// defined above and publishes on the rectified image topic using pub_img_left/right.
//////////////////////////////////////////////////
void synched_img_callback(const sensor_msgs::ImageConstPtr& msg_left, const sensor_msgs::ImageConstPtr& msg_right)
{
    Mat tmp_left  = cv_bridge::toCvShare(msg_left, "bgr8")->image;
    Mat tmp_right = cv_bridge::toCvShare(msg_right, "bgr8")->image;

    Mat dst_left, dst_right;

    undistort_rectify_image(tmp_left, dst_left, CameraSide::LEFT);
    undistort_rectify_image(tmp_right, dst_right, CameraSide::RIGHT);

    sensor_msgs::ImagePtr rect_img_left  = cv_bridge::CvImage(msg_left->header, "bgr8", dst_left).toImageMsg();
    sensor_msgs::ImagePtr rect_img_right = cv_bridge::CvImage(msg_right->header, "bgr8", dst_right).toImageMsg();

    pub_img_rect_left.publish(rect_img_left);
    pub_img_rect_right.publish(rect_img_right);

    std_msgs::Header header = msg_left->header;
    output_camera_info_left.header = header;
    output_camera_info_right.header = header;

    left_camera_info_output_pub.publish(output_camera_info_left);
    right_camera_info_output_pub.publish(output_camera_info_right);
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "camera_fisheye_undistort");

  ros::NodeHandle nh("~");

  string param_file_path;
  if(nh.getParam("param_file_path", param_file_path))
  {
    ROS_WARN("Using parameter file: %s", param_file_path.c_str());
  }
  else
  {
    ROS_ERROR("Failed to get param file path. Please check and try again.");
    ros::shutdown();
    return 0;
  }

  // Read the input parameters and perform initialization
  init_rectification_map(param_file_path);

  // The raw stereo images should be published as type sensor_msgs/Image
  image_transport::ImageTransport it(nh);
  message_filters::Subscriber<sensor_msgs::Image> sub_img_left(nh, "/camera/fisheye1/image_raw", 1);
  message_filters::Subscriber<sensor_msgs::Image> sub_img_right(nh, "/camera/fisheye2/image_raw", 1);
  
  // Having time synced stereo images might be important for other purposes, say generating accurate disparity maps. 
  // To sync the left and right image messages by their header time stamps, ApproximateTime is used.
  // More info here: http://wiki.ros.org/message_filters#Time_Synchronizer
  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::Image> SyncPolicy;
  message_filters::Synchronizer<SyncPolicy> sync(SyncPolicy(10), sub_img_left, sub_img_right);
  sync.registerCallback(boost::bind(&synched_img_callback, _1, _2));

  // The output data include rectified images and their corresponding camera info
  pub_img_rect_left  = it.advertise("/camera/fisheye1/rect/image", 1);
  pub_img_rect_right = it.advertise("/camera/fisheye2/rect/image", 1);

  left_camera_info_output_pub  = nh.advertise<sensor_msgs::CameraInfo>("/camera/fisheye1/rect/camera_info", 1);
  right_camera_info_output_pub = nh.advertise<sensor_msgs::CameraInfo>("/camera/fisheye2/rect/camera_info", 1);

  // Processing start
  ros::spin();
}