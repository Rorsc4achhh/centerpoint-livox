#include <iostream>
#include <chrono>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include "pcl_conversions/pcl_conversions.h"
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

using namespace std::chrono_literals;

class PointCloudPublisher : public rclcpp::Node
{
public:
    PointCloudPublisher() : Node("point_cloud_publisher"), frame_number(1)
    {
        // 创建一个Publisher，发布PointCloud2消息到名为"/livox/point_cloud_front"的topic
        publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/livox/point_cloud_front", 10);

        // 创建一个定时器，每秒钟发布一次PointCloud2消息
        timer_ = this->create_wall_timer(0.35s, std::bind(&PointCloudPublisher::publishPointCloud, this));
    }

private:
    void publishPointCloud()
    {
        // 创建一个PointCloud对象
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);

        // 生成文件名
        std::ostringstream oss;
        oss << "/home/chong/Desktop/pre/test_frame/frame_" << std::setw(1) << frame_number << ".pcd";
        std::string path_pcd = oss.str();

        // 从PCD文件中加载点云数据
        if (pcl::io::loadPCDFile<pcl::PointXYZI>(path_pcd, *cloud) == -1)
        {
            RCLCPP_ERROR(this->get_logger(), "Failed to load PCD file: %s", path_pcd.c_str());
            return;
        }

        RCLCPP_INFO(this->get_logger(), "Publishing: %s", path_pcd.c_str());

        // 创建一个PointCloud2消息
        sensor_msgs::msg::PointCloud2::UniquePtr cloud_msg(new sensor_msgs::msg::PointCloud2);
        pcl::toROSMsg(*cloud, *cloud_msg);
        cloud_msg->header.frame_id = "livox";  // 设置坐标系
        cloud_msg->header.stamp = this->now();     // 设置PointCloud2消息的时间戳

        // 发布PointCloud2消息到"/livox/point_cloud_front"的topic
        publisher_->publish(std::move(cloud_msg));

        // Increment the frame number and wrap around if needed
        frame_number = (frame_number % 1636) + 1;
    }

    int frame_number;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PointCloudPublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
