#ifndef DEPTHPROCESSOR_HPP
#define DEPTHPROCESSOR_HPP

// OpenCv
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

class DepthProcessor
{
  cv::Mat depth_raw_;
  cv::Mat depth_vis_;

  public:
    void processDepthData(cv_bridge::CvImagePtr depth_img_ptr);
    cv::Mat returnTrueDepth();
    cv::Mat returnDepthVis();
    void saveDepthData(const std::string &filename, bool save_depth_pfm = false);
    void showDepthData();
};

#endif
