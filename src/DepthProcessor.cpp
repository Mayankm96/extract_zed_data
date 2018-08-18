#include "extract_zed_data/DepthProcessor.hpp"
#include "extract_zed_data/ImageIOpfm.hpp"

#include <string>

/* depth can be either in 32-bit float (typically in meters) or in 16-bit
** unsigned char (in mm), with openni_depth_mode.
**
** Source: http://docs.ros.org/api/depth_image_proc/html/convert__metric_8cpp_source.html
*/
void DepthProcessor::processDepthData(cv_bridge::CvImagePtr depth_img_ptr)
{
  if (depth_img_ptr->encoding == "16UC1"){
    depth_vis_ = depth_img_ptr->image; // Visualization
    depth_raw_ = cv::Mat(depth_img_ptr->image.rows, depth_img_ptr->image.cols, CV_32FC1); // True depth

    // Fill in the depth image data, converting mm to m
    float bad_point = std::numeric_limits<float>::quiet_NaN ();
    for(size_t i = 0; i < depth_img_ptr->image.rows; i++)
    {
      for(size_t j = 0; j < depth_img_ptr->image.cols; j++)
      {
        uint16_t depth_vis_value = depth_img_ptr->image.at<uint16_t>(i,j);
        depth_raw_.at<float>(i,j) = (depth_vis_value == 0) ? bad_point : (float) depth_vis_value * 0.001f;
      }
    }
  }

  if (depth_img_ptr->encoding == "32FC1"){
    depth_raw_ = depth_img_ptr->image; // True depth
    depth_vis_ = cv::Mat(depth_img_ptr->image.rows, depth_img_ptr->image.cols, CV_16UC1);  // Visualization

    // Fill in the depth image data, converting m to mm
    uint16_t bad_point = 0;
    for(size_t i = 0; i < depth_img_ptr->image.rows; i++)
    {
      for(size_t j = 0; j < depth_img_ptr->image.cols; j++)
      {
        float depth_raw_value = depth_img_ptr->image.at<float>(i,j);
        depth_vis_.at<uint16_t>(i,j) = std::isnan(depth_raw_value) ? bad_point : (uint16_t)(depth_raw_value * 1000);
      }
    }
  }
}

cv::Mat DepthProcessor::returnTrueDepth()
{
  return depth_raw_;
}

cv::Mat DepthProcessor::returnDepthVis()
{
  return depth_vis_;
}

void DepthProcessor::saveDepthData(const std::string &filename, bool save_pfm)
{
  cv::imwrite(filename + ".png", depth_vis_);
  if(save_pfm)
    writeFilePFM(filename + ".pfm", depth_raw_, 1/255.0);
}

void DepthProcessor::showDepthData()
{
  cv::imshow("Depth_Visualization", depth_vis_);
  cv::waitKey(0);
}
