#include <ros/ros.h>
#include <ros/console.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/exact_time.h>

#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>

#include <boost/filesystem.hpp>

#include "extract_zed_data/ZedData.hpp"

using namespace message_filters;

std::string out_foldername;

// Callback for synchronized messages
void callback(const sensor_msgs::CompressedImage::ConstPtr &l_img,
             const sensor_msgs::CompressedImage::ConstPtr &r_img,
             const sensor_msgs::Image::ConstPtr &d_img,
             const sensor_msgs::CameraInfo::ConstPtr &l_info,
             const sensor_msgs::CameraInfo::ConstPtr &r_info,
             const sensor_msgs::CameraInfo::ConstPtr &d_info)
{
  // Zd is class variable to store data
  static uint32_t frame = 1;
  ZedData zd(l_img, r_img, d_img, l_info, r_info, d_info);
  zd.saveData(frame, out_foldername);
  frame = frame + 1;
}

int main(int argc, char** argv)
{
  // Initialize node
  ros::init(argc, argv, "extract_topic_zed");
  ros::NodeHandle nh("~");

  // read node paramters
  nh.param<std::string>("out_foldername", out_foldername;

  // Topic names
  std::string image_ns = "/zed";
  std::string l_cam = image_ns + "/left";
  std::string r_cam = image_ns + "/right";
  std::string d_cam = image_ns + "/depth";
  // sensor_msgs/Image topic names
  std::string l_cam_image = l_cam + "/image_rect_color/compressed";
  std::string r_cam_image = r_cam + "/image_rect_color/compressed";
  std::string d_cam_image = d_cam + "/depth_registered";
  // sensor_msgs/camea_info topic names
  std::string l_cam_info = l_cam + "/camera_info";
  std::string r_cam_info = r_cam + "/camera_info";
  std::string d_cam_info = d_cam + "/camera_info";

  // Set up subscribers to capture images and camera information
  message_filters::Subscriber<sensor_msgs::CompressedImage> l_img_sub, r_img_sub;
  message_filters::Subscriber<sensor_msgs::Image>  d_img_sub;
  message_filters::Subscriber<sensor_msgs::CameraInfo> l_info_sub, r_info_sub, d_info_sub;

  l_img_sub.subscribe(nh, l_cam_image, 1);
  r_img_sub.subscribe(nh, r_cam_image, 1);
  d_img_sub.subscribe(nh, d_cam_image, 1);
  l_info_sub.subscribe(nh, l_cam_info, 1);
  r_info_sub.subscribe(nh, r_cam_info, 1);
  d_info_sub.subscribe(nh, d_cam_info, 1);

  // Setup output directory to save dataset
  boost::filesystem::path dir(out_foldername);
  if(boost::filesystem::create_directory(dir))
  {
        ROS_INFO("Created folder to save data");
  }

  // Use time synchronizer to make sure we get properly synchronized images
  typedef sync_policies::ExactTime<sensor_msgs::CompressedImage, sensor_msgs::CompressedImage, sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::CameraInfo, sensor_msgs::CameraInfo> RetrieveZedDataPolicy;
  Synchronizer<RetrieveZedDataPolicy> sync(RetrieveZedDataPolicy(10), l_img_sub, r_img_sub, d_img_sub, l_info_sub, r_info_sub, d_info_sub);
  sync.registerCallback(boost::bind(&callback, _1, _2, _3, _4, _5, _6));

  ros::spin();
  return 0;
}
