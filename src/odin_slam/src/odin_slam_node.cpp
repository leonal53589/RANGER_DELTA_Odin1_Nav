#include "odin_slam_node.hpp"
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/io/pcd_io.h>

#include <filesystem>

OdinSlamNode::OdinSlamNode(const std::string &node)
    : rclcpp::Node(node)
{
    m_cloud_sub = create_subscription<sensor_msgs::msg::PointCloud2>(
        "/odin1/cloud_slam", rclcpp::SensorDataQoS(),
        std::bind(&OdinSlamNode::on_receive_cloud, this, std::placeholders::_1)
    );

    this->declare_parameter("save_pcd_path", std::string("/tmp/odin_slam_result.pcd"));
    this->declare_parameter("cropbox.min_x", -10.0);
    this->declare_parameter("cropbox.min_y", -10.0);
    this->declare_parameter("cropbox.min_z", -10.0);
    this->declare_parameter("cropbox.max_x", 10.0);
    this->declare_parameter("cropbox.max_y", 10.0);
    this->declare_parameter("cropbox.max_z", 10.0);
    this->declare_parameter("voxel.leaf_size", 0.1);
    this->declare_parameter("sor.mean_k", 50);
    this->declare_parameter("sor.stddev_mul_thresh", 1.0);

    m_acc_cloud = std::make_shared<PointCloud>();
    m_acc_cloud->width = 0;
    m_acc_cloud->height = 1;
    m_acc_cloud->is_dense = true;
}

void OdinSlamNode::run_slam()
{
    m_acc_cloud->clear();

    m_acc_flag = true;
    RCLCPP_INFO(this->get_logger(), "SLAM started. Cloud cleared.");
}

void OdinSlamNode::stop_slam()
{
    m_acc_flag = false;
    RCLCPP_INFO(this->get_logger(), "SLAM stoped.");
}

void OdinSlamNode::save_result()
{
    if (m_acc_cloud->empty())
        return;

    // 1. CropBox Filter
    double min_x = this->get_parameter("cropbox.min_x").as_double();
    double min_y = this->get_parameter("cropbox.min_y").as_double();
    double min_z = this->get_parameter("cropbox.min_z").as_double();
    double max_x = this->get_parameter("cropbox.max_x").as_double();
    double max_y = this->get_parameter("cropbox.max_y").as_double();
    double max_z = this->get_parameter("cropbox.max_z").as_double();
    
    PointCloudPtr cloud_crop = std::make_shared<PointCloud>();
    pcl::CropBox<pcl::PointXYZ> crop_box;
    crop_box.setMin(Eigen::Vector4f(min_x, min_y, min_z, 1.0));
    crop_box.setMax(Eigen::Vector4f(max_x, max_y, max_z, 1.0));
    crop_box.setInputCloud(m_acc_cloud);
    crop_box.filter(*cloud_crop);

    if (cloud_crop->empty())
    {
        RCLCPP_WARN(this->get_logger(), "Cloud empty after CropBox filter. Stop processing.");
        return;
    }

    // 2. VoxelGrid Filter
    double leaf_size = this->get_parameter("voxel.leaf_size").as_double();
    
    PointCloudPtr cloud_voxel = std::make_shared<PointCloud>();
    pcl::VoxelGrid<pcl::PointXYZ> voxel_grid;
    voxel_grid.setLeafSize(leaf_size, leaf_size, leaf_size);
    voxel_grid.setInputCloud(cloud_crop);
    voxel_grid.filter(*cloud_voxel);

    if (cloud_voxel->empty())
    {
        RCLCPP_WARN(this->get_logger(), "Cloud empty after VoxelGrid filter. Stop processing.");
        return;
    }

    // 3. StatisticalOutlierRemoval Filter
    int mean_k = this->get_parameter("sor.mean_k").as_int();
    double stddev = this->get_parameter("sor.stddev_mul_thresh").as_double();
    
    PointCloudPtr cloud_sor = std::make_shared<PointCloud>();
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setMeanK(mean_k);
    sor.setStddevMulThresh(stddev);
    sor.setInputCloud(cloud_voxel);
    sor.filter(*cloud_sor);

    if (cloud_sor->empty())
    {
        RCLCPP_WARN(this->get_logger(), "Cloud empty after SOR filter. Stop processing.");
        return;
    }

    std::filesystem::path pcd_path = this->get_parameter("save_pcd_path").as_string();
    std::filesystem::create_directories(pcd_path);
    pcd_path /= std::to_string(m_save_counter) + ".pcd";
    if (pcl::io::savePCDFileBinary(pcd_path, *cloud_sor) == 0)
    {
        RCLCPP_INFO(this->get_logger(), "Successfully saved filtered point cloud to: %s", pcd_path.c_str());
        m_save_counter++;
        m_acc_cloud->clear();
    }
    else
        RCLCPP_ERROR(this->get_logger(), "Failed to save PCD file to: %s", pcd_path.c_str());
}

void OdinSlamNode::on_receive_cloud(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msg)
{
    if (m_acc_flag)
    {
        PointCloudPtr frame = std::make_shared<PointCloud>();
        pcl::fromROSMsg(*msg, *frame);

        PointCloudPtr no_nan_cloud = std::make_shared<PointCloud>();
        pcl::Indices indices;
        pcl::removeNaNFromPointCloud(*frame, *no_nan_cloud, indices);

        // 对单帧进行体素滤波降采样
        double leaf_size = this->get_parameter("voxel.leaf_size").as_double();
        PointCloudPtr filtered_frame = std::make_shared<PointCloud>();
        pcl::VoxelGrid<pcl::PointXYZ> voxel_grid;
        voxel_grid.setLeafSize(leaf_size, leaf_size, leaf_size);
        voxel_grid.setInputCloud(no_nan_cloud);
        voxel_grid.filter(*filtered_frame);

        if (!filtered_frame->empty())
            *m_acc_cloud += *filtered_frame;
    }
}