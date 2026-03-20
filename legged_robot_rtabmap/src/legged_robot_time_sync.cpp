#include "ros/ros.h"
#include "rosbag/bag.h"
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/LaserScan.h>
#include "sensor_msgs/CompressedImage.h"
#include "sensor_msgs/Imu.h"
#include "sensor_msgs/CameraInfo.h"

#include "image_transport/image_transport.h"
#include "nav_msgs/Odometry.h"

#include <iostream>
#include <mutex>

#include "ctime"
#include "time.h"

using namespace std;

ros::Subscriber sub_depth_img, sub_depth_info, sub_left_img, sub_left_info;
ros::Publisher pub_depth_img, pub_depth_info, pub_left_img, pub_left_info;

void depth_img_calbak(const sensor_msgs::ImagePtr& msg)
{
    msg->header.stamp = ros::Time::now();
    pub_depth_img.publish(msg);
}

void depth_info_calbak(const sensor_msgs::CameraInfoPtr& msg)
{
    msg->header.stamp = ros::Time::now();
    pub_depth_info.publish(msg);
}

void left_img_calbak(const sensor_msgs::ImagePtr& msg)
{
    msg->header.stamp = ros::Time::now();
    pub_left_img.publish(msg);
}

void left_info_calbak(const sensor_msgs::CameraInfoPtr& msg)
{
    msg->header.stamp = ros::Time::now();
    pub_left_info.publish(msg);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "legged_robot_time_sync");
  ros::Time::init();
  ros::NodeHandle nh;

  ROS_INFO("start message sync publish...");

  sub_depth_img = nh.subscribe("/realsense/depth/image", 2000, depth_img_calbak);
  sub_depth_info = nh.subscribe("/realsense/depth/camera_info", 2000, depth_info_calbak);
  sub_left_img = nh.subscribe("/realsense/rgb/left_image_raw", 2000, left_img_calbak);
  sub_left_info = nh.subscribe("/realsense/rgb/left_image_info", 2000, left_info_calbak);

  pub_depth_img = nh.advertise<sensor_msgs::Image>("/legged_robot/depth/image",1000);
  pub_depth_info = nh.advertise<sensor_msgs::CameraInfo>("/legged_robot/depth/camera_info",1000);
  pub_left_img = nh.advertise<sensor_msgs::Image>("/legged_robot/rgb/left_image_raw",1000);
  pub_left_info = nh.advertise<sensor_msgs::CameraInfo>("/legged_robot/rgb/left_image_info",1000);

  ros::spin();
  return 0;
}