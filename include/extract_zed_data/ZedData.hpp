#ifndef ZED_DATA_HPP
#define ZED_DATA_HPP

#include <iostream>

// ROS
#include <ros/ros.h>
#include <ros/console.h>

#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>

// A struct to hold the synchronized camera data
// Struct to store zed data
class ZedData
{
    sensor_msgs::CompressedImage::ConstPtr image_l, image_r;
    sensor_msgs::Image::ConstPtr image_d;
    sensor_msgs::CameraInfo::ConstPtr cam_info_l, cam_info_r, cam_info_d;

  public:
    ZedData(const sensor_msgs::CompressedImage::ConstPtr &l_img,
           const sensor_msgs::CompressedImage::ConstPtr &r_img,
           const sensor_msgs::Image::ConstPtr &d_img,
           const sensor_msgs::CameraInfo::ConstPtr &l_info,
           const sensor_msgs::CameraInfo::ConstPtr &r_info,
           const sensor_msgs::CameraInfo::ConstPtr &d_info) :
      image_l(l_img),
      image_r(r_img),
      image_d(d_img),
      cam_info_l(l_info),
      cam_info_r(r_info),
      cam_info_d(d_info)
    {}

    void saveData(uint32_t frame, const std::string &out_foldername );
};

#endif
