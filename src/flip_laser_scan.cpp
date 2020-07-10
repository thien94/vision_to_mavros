#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <string.h>

//////////////////////////////////////////////////
// Global variables
//////////////////////////////////////////////////

ros::Publisher flipped_laser_scan_pub;

//////////////////////////////////////////////////
// Functions
//////////////////////////////////////////////////

void laserscan_cb(const sensor_msgs::LaserScan::ConstPtr& input_laser_scan_msg)
{
  sensor_msgs::LaserScan flipped_laser_scan_msg = *input_laser_scan_msg;
  flipped_laser_scan_msg.angle_min = -input_laser_scan_msg->angle_max;
  flipped_laser_scan_msg.angle_max = -input_laser_scan_msg->angle_min;

  int len_ranges_array = input_laser_scan_msg->ranges.size();
  for (int i = 0; i < len_ranges_array; ++i)
  {
    flipped_laser_scan_msg.ranges[i] = input_laser_scan_msg->ranges[len_ranges_array - 1 - i];
  }

  flipped_laser_scan_pub.publish(flipped_laser_scan_msg);
}

//////////////////////////////////////////////////
// Main
//////////////////////////////////////////////////

int main(int argc, char **argv)
{
  ros::init(argc, argv, "flip_laser_scan");

  ros::NodeHandle node;

  std::string input_laser_scan_topic = "scan";
  std::string flipped_laser_scan_topic = "flipped_laser_scan";

  ros::Subscriber depthimage_to_laserscan_output_sub = node.subscribe(input_laser_scan_topic, 1, laserscan_cb);
  flipped_laser_scan_pub = node.advertise<sensor_msgs::LaserScan>(flipped_laser_scan_topic, 10);

  ros::spin();
}