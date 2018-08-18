// ROS
#include <ros/ros.h>
#include <ros/package.h>
#include <ros/console.h>
#include <rosbag/bag.h>
#include <rosbag/view.h>

// OpenCv
#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <cv_bridge/cv_bridge.h>

#include <boost/foreach.hpp>
#include <boost/filesystem.hpp>

#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>

#include "extract_zed_data/DepthProcessor.hpp"
#include <camera_calibration_parsers/parse.h>

using namespace std;

DepthProcessor zed_depth;
bool save_depth_pfm;

/*
 *  Reads a bag file comprising of zed camera data and saves the file
 *  contents
 *
 *  Params:
 *      bag_filename:     type: string    description: file path to bag file
 *      out_foldername:   type: string    description: output path to save bag contents
 *      image_ns:         type: string    description: image namespace
 */
void loadBag(const std::string &bag_filename, const std::string &out_foldername = "", const std::string &image_ns = "/zed")
{
  rosbag::Bag bag;
  // Open the bag file
  bag.open(bag_filename, rosbag::bagmode::Read);
  ROS_INFO("Opened %s", bag_filename.c_str());

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

  // Image topics to load
  std::vector<std::string> topics;
  topics.push_back(l_cam_image);
  topics.push_back(r_cam_image);
  topics.push_back(d_cam_image);
  topics.push_back(l_cam_info);
  topics.push_back(r_cam_info);
  topics.push_back(d_cam_info);

  rosbag::View view(bag, rosbag::TopicQuery(topics));

  uint32_t  bag_size = view.size();
  ROS_INFO("Bagfile loaded. Size: %d", bag_size);

  // Load all messages into our zed camera dataset
  uint32_t frame = 0, msg_num = 0;
  std::string frame_name;
  BOOST_FOREACH(rosbag::MessageInstance const m, view)
  {
      // topics.size()+1 since right_caminfo published at twice the rate in bagfile
      if (msg_num % (topics.size()+1) == 0)
      {
        frame ++;
        ROS_INFO("Processing timestamp: %05i ", frame);
        std::stringstream frame_stream;
        frame_stream << std::setw(6) << std::setfill('0') << frame;
        frame_name = std::string(frame_stream.str().c_str());
      }

      // save CompressedImage published in message at "../left/image_rect_color/compressed"
      if (m.getTopic() == l_cam_image || ("/" + m.getTopic() == l_cam_image))
      {
        sensor_msgs::CompressedImage::ConstPtr l_img = m.instantiate<sensor_msgs::CompressedImage>();
        if (l_img != NULL)
        {
          cv_bridge::CvImagePtr left_img_ptr;
          try
          {
            ROS_INFO("Received Left Camera Image");
            left_img_ptr = cv_bridge::toCvCopy(l_img, "bgr8");
            cv::imwrite(out_foldername + "left_" + frame_name + ".jpg", left_img_ptr->image);
          }
          catch (cv_bridge::Exception& e)
          {
            ROS_ERROR("cv_bridge exception: %s", e.what());
          }
        }
      }

      // save CompressedImage published in message at "../right/image_rect_color/compressed"
      if (m.getTopic() == r_cam_image || ("/" + m.getTopic() == r_cam_image))
      {
        sensor_msgs::CompressedImage::ConstPtr r_img = m.instantiate<sensor_msgs::CompressedImage>();
        if (r_img != NULL)
        {
          cv_bridge::CvImagePtr right_img_ptr;
          try
          {
            ROS_INFO("Received Right Camera Image");
            right_img_ptr = cv_bridge::toCvCopy(r_img, "bgr8");
            cv::imwrite(out_foldername + "right_" + frame_name + ".jpg", right_img_ptr->image);
          }
          catch (cv_bridge::Exception& e)
          {
            ROS_ERROR("cv_bridge exception: %s", e.what());
          }
        }
      }

      // save Image published in message at "../depth/depth_registered"
      if (m.getTopic() == d_cam_image || ("/" + m.getTopic() == d_cam_image))
      {
        sensor_msgs::Image::ConstPtr d_img = m.instantiate<sensor_msgs::Image>();
        if (d_img != NULL)
        {
          cv_bridge::CvImagePtr depth_img_ptr;
          try
          {
            ROS_INFO("Depth Map Received");
            depth_img_ptr = cv_bridge::toCvCopy(d_img);
            zed_depth.processDepthData(depth_img_ptr);
            zed_depth.saveDepthData(out_foldername + "depth_" + frame_name, save_depth_pfm);
          }
          catch (cv_bridge::Exception& e)
          {
            ROS_ERROR("cv_bridge exception: %s", e.what());
          }
        }
      }

      // save left camera parameters into a YAML file
      if (m.getTopic() == l_cam_info || ("/" + m.getTopic() == l_cam_info))
      {
        sensor_msgs::CameraInfo::ConstPtr l_info = m.instantiate<sensor_msgs::CameraInfo>();
        if (l_info != NULL)
        {
            ROS_INFO("Left Camera Parameters Received");
            camera_calibration_parsers::writeCalibration(out_foldername + "zed_left_cam.yaml", "Zed Left Camera", *l_info);
        }
      }

      // save right camera parameters into a YAML file
      if (m.getTopic() == r_cam_info || ("/" + m.getTopic() == r_cam_info))
      {
        sensor_msgs::CameraInfo::ConstPtr r_info = m.instantiate<sensor_msgs::CameraInfo>();
        if (r_info != NULL)
        {
          ROS_INFO("Right Camera Parameters Received");
          camera_calibration_parsers::writeCalibration(out_foldername + "zed_right_cam.yaml", "Zed Right Camera", *r_info);
        }
      }

      // save depth camera parameters into a YAML file
      if (m.getTopic() == d_cam_info || ("/" + m.getTopic() == d_cam_info))
      {
        sensor_msgs::CameraInfo::ConstPtr d_info = m.instantiate<sensor_msgs::CameraInfo>();
        if (d_info != NULL)
        {
          ROS_INFO("Depth Camera Parameters Received");
          camera_calibration_parsers::writeCalibration(out_foldername + "zed_depth_cam.yaml", "Zed Depth Camera", *d_info);
        }
      }

      msg_num ++;
  }
  bag.close();
}

int main( int argc, char** argv )
{
  //Start ROS ----------------------------------------------------------------
  ros::init(argc, argv, "extract_rosbag_zed");
  ros::NodeHandle nh("~");

  // rosbag file
  std::string bag_filename, out_foldername;
  // read node paramters
  nh.param<std::string>("bag_filename", bag_filename, "/media/mayankm/Data/Fount_Landing/dataset/depth_sync_2017-11-21-10-51-39.bag");
  nh.param<std::string>("out_foldername", out_foldername, "/mnt/MM_Seagate_BUP/Fount_Landing/dataset/");
  nh.param<bool>("save_depth_pfm", save_depth_pfm, false);

  ROS_WARN("Reading Bag file: %s", bag_filename.c_str());
  ROS_WARN("Saving data to folder: %s", out_foldername.c_str());
  ROS_WARN("Saving depth pfm data: %d", save_depth_pfm);

  boost::filesystem::path dir(out_foldername);
	if(boost::filesystem::create_directory(dir))
  {
		ROS_INFO("Created folder %s to save data", out_foldername.c_str());
	}

  loadBag(bag_filename, out_foldername);

  ROS_INFO("Processing Complete");
  return 0;
}
