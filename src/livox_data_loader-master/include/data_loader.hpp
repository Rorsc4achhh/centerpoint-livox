#pragma once
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <opencv2/opencv.hpp>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "sensor_msgs/msg/compressed_image.hpp"
#include <visualization_msgs/msg/marker_array.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <pcl_conversions/pcl_conversions.h>
#include "ws_msgs/msg/bbox_array.hpp"
#include <chrono>
#include <filesystem>
#include <fstream>
#include <vector>
#include <string>
#include <cstdlib>
#include "yaml-cpp/yaml.h"
#include <iostream>
#include <fstream>
#include <chrono>


typedef pcl::PointXYZ Point;
typedef pcl::PointXYZI PointI;
typedef pcl::PointCloud<PointI> PointICloud;
typedef PointICloud::Ptr PointICloudPtr;
typedef PointICloud::ConstPtr PointICloudConstPtr;

class DataLoader : public rclcpp::Node
{
public:
  enum class PublisherType 
  { 
      POINT_CLOUD_FRONT = 0,
      POINT_CLOUD_LEFT_FRONT = 1,
      POINT_CLOUD_LEFT_REAR = 2,
      POINT_CLOUD_RIGHT_REAR = 3,
      POINT_CLOUD_RIGHT_FRONT = 4,
      POINT_CLOUD_TELE = 5,
      IMAGE_FRONT =  6,
      IMAGE_LEFT_FRONT = 7,
      IMAGE_LEFT_REAR = 8,
      IMAGE_RIGHT_REAR = 9, 
      IMAGE_RIGHT_FRONT = 10,  
      IMAGE_TELE = 11,
      LABEL = 12
  };

public:
    DataLoader();
    ~DataLoader();

private:
  std::string getPath(PublisherType publisher_type);
  std::vector<std::string> getFilenames(PublisherType publisher_type);
  void setFilenames(PublisherType publisher_type, std::vector<std::string> file_names);
  void onTimerCallback();
  void initFilePath(std::string data_path);
  void createPublishersDataFileNames();

  void loadAscPts(char* filename,pcl::PointCloud<pcl::PointXYZI> & cloud);

  std::string matType2encoding(int mat_type);
  void convertImageToMsg(sensor_msgs::msg::Image & msg, const std::string path );
  void convertPclToPointcloud2(sensor_msgs::msg::PointCloud2 & msg,const std::string path);
  void convertLabelToMsg(ws_msgs::msg::BboxArray& boxs, const std::string path);
  size_t file_index_;

  rclcpp::TimerBase::SharedPtr timer_;

  builtin_interfaces::msg::Time stamp_;

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_point_cloud_front_;   // point clouds publisher
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_point_cloud_left_front_;   // point clouds publisher
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_point_cloud_left_rear_;   // point clouds publisher
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_point_cloud_right_rear_;   // point clouds publisher
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_point_cloud_right_front_;   // point clouds publisher
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_point_cloud_tele_;   // point clouds publisher
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_image_front_;     // 
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_image_left_front_;     // 
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_image_left_rear_;     // 
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_image_right_rear_;     // 
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_image_right_front_;     // 
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_image_tele_;     // 
  rclcpp::Publisher<ws_msgs::msg::BboxArray>::SharedPtr publisher_gt_;
  std::string data_path_;
  std::string label_path_;
  std::string yaml_path_ ="/home/txy/wspace/param/config.yaml";

  std::vector<std::string> file_names_point_cloud_front_;
  std::vector<std::string> file_names_point_cloud_left_front_;
  std::vector<std::string> file_names_point_cloud_left_rear_;
  std::vector<std::string> file_names_point_cloud_right_rear_;
  std::vector<std::string> file_names_point_cloud_right_front_;
  std::vector<std::string> file_names_point_cloud_tele_;

  std::vector<std::string> file_names_image_front_;
  std::vector<std::string> file_names_image_left_front_;
  std::vector<std::string> file_names_image_left_rear_;
  std::vector<std::string> file_names_image_right_rear_;
  std::vector<std::string> file_names_image_right_front_;
  std::vector<std::string> file_names_image_tele_;

  std::vector<std::string> file_names_gt_label_;

  std::string path_point_cloud_front_;
  std::string path_point_cloud_left_front_;
  std::string path_point_point_cloud_left_rear_;
  std::string path_point_cloud_right_rear_;
  std::string path_point_cloud_right_front_;
  std::string path_point_cloud_tele_;

  std::string path_image_front_;
  std::string path_image_left_front_;
  std::string path_image_left_rear_;
  std::string path_image_right_rear_;
  std::string path_image_right_front_;
  std::string path_image_tele_;

  std::vector<std::string> class_names_ = {};

  std::string path_gt_label_;
  sensor_msgs::msg::CompressedImage::SharedPtr cp_msg_;
};


