#pragma once
#include <algorithm>
#include <queue>
#include <numeric>
#include <pcl/point_cloud.h>
#include <pcl_conversions/pcl_conversions.h>
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include "geometry_msgs/msg/pose_array.hpp"
#include "ws_msgs/msg/cluster_array.hpp"
#include "ws_msgs/msg/bbox.hpp"
#include "ws_msgs/msg/bbox_array.hpp"
#include <pcl/filters/passthrough.h>
#include "centerpoint.h"
#include "process.h"
#include "postprocess.h"
#include "yaml-cpp/yaml.h"
#include <deque>
#include <chrono>


#include "visualization_msgs/msg/marker_array.hpp"


using Bbox = ws_msgs::msg::Bbox;
using bboxArray = ws_msgs::msg::BboxArray;
static size_t BoxFeature = 7;
using Clock = std::chrono::high_resolution_clock;
class Detection : public rclcpp::Node
{

public:
    Detection( const std::string& name_space,const rclcpp::NodeOptions& options=rclcpp::NodeOptions());
    Detection(const rclcpp::NodeOptions& options=rclcpp::NodeOptions());
    
    // Detection(const std::string& name_space, const rclcpp::NodeOptions& options);
    // ~Detection();

private:

    std::ofstream output_file_;


    void cloudCallbak(const sensor_msgs::msg::PointCloud2::ConstPtr &input);
    void convertBboxToLineList(const ws_msgs::msg::Bbox& bbox, visualization_msgs::msg::Marker& marker, int id, const rclcpp::Time& stamp);
 
    void makeOutput(std::vector<Box> &out_detections,rclcpp::Time& stamp);

    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;

    rclcpp::Publisher<ws_msgs::msg::BboxArray>::SharedPtr pub_bbox_array_;

    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;


    rclcpp::Clock::SharedPtr clock_;

    std::unique_ptr<CenterPoint> centerpoint_ = nullptr;

    void loadParam(std::string & param_path);

    bboxArray bbox_array_;

    Clock::time_point m_sync_start_time_;

    bool use_onnx_ = false;
    std::string rpn_file_;
    std::string centerpoint_config_;
    std::string file_name_;
    std::string param_path_ = "/home/chong/ros2_ws/src/centerpoint-livox/cfgs/centerpoint.yaml";

    int pub_count_ = 0;

    int sub_count_ = 0;


};


