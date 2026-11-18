#pragma once

#include <QObject>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

using PointCloud = pcl::PointCloud<pcl::PointXYZ>;
using PointCloudPtr = PointCloud::Ptr;

class OdinSlamNode : public QObject, public rclcpp::Node
{
    Q_OBJECT
public:
    OdinSlamNode(const std::string &node = "odin_slam_node");

public slots:
    void run_slam();
    void stop_slam();
    void save_result();

private:
    void on_receive_cloud(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msg);

private:
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr m_cloud_sub;
    PointCloudPtr m_acc_cloud;
    bool m_acc_flag = false;
    int m_save_counter = 1;
};