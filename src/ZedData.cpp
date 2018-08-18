#include "extract_zed_data/ZedData.hpp"

#include <string>
#include <iostream>

// OpenCv
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

#include "extract_zed_data/DepthProcessor.hpp"
#include <camera_calibration_parsers/parse.h>

using namespace std;

DepthProcessor zed_depth;

void ZedData::saveData(uint32_t frame, const std::string &out_foldername = "")
{
  ROS_INFO("Saving data at frame: %05i ", frame);
  std::stringstream frame_stream;
  frame_stream << std::setw(6) << std::setfill('0') << frame;
  std::string frame_name = std::string(frame_stream.str().c_str());

  // save left camera sensor_msgs/CompressedImage
  if (image_l != NULL)
  {
    cv_bridge::CvImagePtr left_img_ptr;
    try
    {
      ROS_INFO("Saved Left Camera Image");
      left_img_ptr = cv_bridge::toCvCopy(image_l, "bgr8");
      cv::imwrite(out_foldername + "left_" + frame_name + ".jpg", left_img_ptr->image);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
    }
  }

  // save right camera sensor_msgs/CompressedImage
  if (image_r != NULL)
  {
    cv_bridge::CvImagePtr right_img_ptr;
    try
    {
      ROS_INFO("Received Right Camera Image");
      right_img_ptr = cv_bridge::toCvCopy(image_r, "bgr8");
      cv::imwrite(out_foldername + "right_" + frame_name + ".jpg", right_img_ptr->image);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
    }
  }

  // save depth camera sensor_msgs/Image
  if (image_d != NULL)
  {
    cv_bridge::CvImagePtr depth_img_ptr;
    try
    {
      ROS_INFO("Depth Map Received");
      depth_img_ptr = cv_bridge::toCvCopy(image_d);
      zed_depth.processDepthData(depth_img_ptr);
      zed_depth.saveDepthData(out_foldername + "depth_" + frame_name);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
    }
  }

  // save left camera parameters into a YAML file
  if (cam_info_l != NULL)
  {
      ROS_INFO("Left Camera Parameters Received");
      camera_calibration_parsers::writeCalibration(out_foldername + "zed_left_cam.yaml", "Zed Left Camera", *cam_info_l);
  }

  // save right camera parameters into a YAML file
  if (cam_info_r != NULL)
  {
    ROS_INFO("Right Camera Parameters Received");
    camera_calibration_parsers::writeCalibration(out_foldername + "zed_right_cam.yaml", "Zed Right Camera", *cam_info_r);
  }

  // save depth camera parameters into a YAML file
  if (cam_info_d != NULL)
  {
    ROS_INFO("Depth Camera Parameters Received");
    camera_calibration_parsers::writeCalibration(out_foldername + "zed_depth_cam.yaml", "Zed Depth Camera", *cam_info_d);
  }
}
