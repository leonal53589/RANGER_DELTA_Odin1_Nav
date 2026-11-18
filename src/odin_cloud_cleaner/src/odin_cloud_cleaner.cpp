#include <cmath>
#include <limits>
#include <optional>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <geometry_msgs/msg/transform_stamped.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>

#include <nav_msgs/msg/odometry.hpp>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/common/transforms.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <rclcpp/logging.hpp>
#include <rclcpp/qos.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <tf2/exceptions.h>
#include <tf2/time.h>
#include <tf2_ros/buffer.hpp>
#include <tf2_ros/transform_listener.hpp>

class OdinCloudCleaner : public rclcpp::Node
{
public:
    using PointCloud = pcl::PointCloud<pcl::PointXYZRGB>;
    using SyncPolicy = message_filters::sync_policies::ApproximateTime<
        sensor_msgs::msg::PointCloud2,
        nav_msgs::msg::Odometry>;

    OdinCloudCleaner(const std::string &name = "odin_cloud_cleaner")
        : rclcpp::Node(name)
        , m_tf_buffer(get_clock())
        , m_tf_listener(m_tf_buffer)
    {
        m_cloud_sub.subscribe(this, "/odin1/cloud_slam", rclcpp::SensorDataQoS().get_rmw_qos_profile());
        m_odom_sub.subscribe(this, "/odin1/odometry_highfreq", rclcpp::SensorDataQoS().get_rmw_qos_profile());

        m_sync = std::make_shared<message_filters::Synchronizer<SyncPolicy>>(
            SyncPolicy(100),
            m_cloud_sub,
            m_odom_sub
        );
        m_sync->registerCallback(std::bind(
            &OdinCloudCleaner::cloud_callback,
            this,
            std::placeholders::_1,
            std::placeholders::_2
        ));

        // m_raw_cloud_sub = create_subscription<sensor_msgs::msg::PointCloud2>(
        //     "/odin1/cloud_raw", 10,
        //     std::bind(&OdinCloudCleaner::raw_cloud_callback, this, std::placeholders::_1)
        // );

        m_cloud_pub = create_publisher<sensor_msgs::msg::PointCloud2>("/cloud_cleaned", rclcpp::SensorDataQoS());
    }

private:
    Eigen::Matrix4f tf_to_matrix(const geometry_msgs::msg::TransformStamped &transform_msg) const
    {
        Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();

        Eigen::Quaternionf quaternion(
            transform_msg.transform.rotation.w,
            transform_msg.transform.rotation.x,
            transform_msg.transform.rotation.y,
            transform_msg.transform.rotation.z
        );
        transform.block<3, 3>(0, 0) = quaternion.toRotationMatrix();
        transform(0, 3) = transform_msg.transform.translation.x;
        transform(1, 3) = transform_msg.transform.translation.y;
        transform(2, 3) = transform_msg.transform.translation.z;

        return transform;
    }

    Eigen::Matrix4f odom_pose_to_matrix(const nav_msgs::msg::Odometry::ConstSharedPtr &odom_msg) const
    {
        Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();

        Eigen::Quaternionf quaternion(
            odom_msg->pose.pose.orientation.w,
            odom_msg->pose.pose.orientation.x,
            odom_msg->pose.pose.orientation.y,
            odom_msg->pose.pose.orientation.z
        );
        transform.block<3, 3>(0, 0) = quaternion.toRotationMatrix();
        transform(0, 3) = odom_msg->pose.pose.position.x;
        transform(1, 3) = odom_msg->pose.pose.position.y;
        transform(2, 3) = odom_msg->pose.pose.position.z;

        return transform;
    }

    std::optional<Eigen::Matrix4f> lookup_transform_matrix(
        const std::string &target_frame,
        const std::string &source_frame
    )
    {
        try
        {
            geometry_msgs::msg::TransformStamped transform_msg =
                m_tf_buffer.lookupTransform(target_frame, source_frame, tf2::TimePointZero);
            return tf_to_matrix(transform_msg);
        }
        catch (const tf2::TransformException &ex)
        {
            RCLCPP_WARN_THROTTLE(
                get_logger(),
                *get_clock(),
                2000,
                "Transform %s <- %s lookup failed: %s",
                target_frame.c_str(),
                source_frame.c_str(),
                ex.what()
            );
            return std::nullopt;
        }
    }

    void cloud_callback(
        const sensor_msgs::msg::PointCloud2::ConstSharedPtr &cloud_msg,
        const nav_msgs::msg::Odometry::ConstSharedPtr &odom_msg
    )
    {
        // auto start = std::chrono::high_resolution_clock::now();

        PointCloud::Ptr raw_cloud(new PointCloud());
        pcl::fromROSMsg(*cloud_msg, *raw_cloud);

        if (raw_cloud->points.empty())
        {
            RCLCPP_WARN(get_logger(), "Receive a null point cloud!");
            return;
        }

        PointCloud::Ptr no_nan_cloud(new PointCloud());
        std::vector<int> index;
        pcl::removeNaNFromPointCloud(*raw_cloud, *no_nan_cloud, index);

        if (no_nan_cloud->points.empty())
        {
            RCLCPP_WARN(get_logger(), "Receive a null point cloud!");
            return;
        }

        Eigen::Matrix4f odom_T_imu = odom_pose_to_matrix(odom_msg);

        std::optional<Eigen::Matrix4f> imu_T_lidar = lookup_transform_matrix("odin_imu_link", "odin_lidar_link");
        if (!imu_T_lidar)
            return;

        Eigen::Matrix4f odom_T_lidar = odom_T_imu * *imu_T_lidar;

        PointCloud::Ptr cloud_in_lidar(new PointCloud());
        pcl::transformPointCloud(*no_nan_cloud, *cloud_in_lidar, odom_T_lidar.inverse());

        pcl::CropBox<pcl::PointXYZRGB> crop_box;
        crop_box.setInputCloud(cloud_in_lidar);
        crop_box.setMin(Eigen::Vector4f(
            std::nextafter(-1.0f, 1.0f),
            std::nextafter(-1.0f, 1.0f),
            std::numeric_limits<float>::lowest(),
            1.0f
        ));
        crop_box.setMax(Eigen::Vector4f(
            std::nextafter(1.0f, 0.0f),
            std::nextafter(1.0f, 0.0f),
            std::nextafter(0.0f, -1.0f),
            1.0f
        ));
        crop_box.setNegative(true);

        PointCloud cloud_in_lidar_filtered;
        crop_box.filter(cloud_in_lidar_filtered);

        if (cloud_in_lidar_filtered.points.empty())
        {
            RCLCPP_WARN(get_logger(), "No points left after CropBox filter");
            return;
        }

        PointCloud cloud_in_odom_filtered;
        pcl::transformPointCloud(cloud_in_lidar_filtered, cloud_in_odom_filtered, odom_T_lidar);

        sensor_msgs::msg::PointCloud2 out_msg;
        pcl::toROSMsg(cloud_in_odom_filtered, out_msg);
        out_msg.header = cloud_msg->header;

        // auto finish = std::chrono::high_resolution_clock::now();
        // std::chrono::duration<double, std::milli> diff = finish - start;
        // RCLCPP_INFO(get_logger(), "duration:%f", diff.count());

        m_cloud_pub->publish(out_msg);
    }

    message_filters::Subscriber<sensor_msgs::msg::PointCloud2> m_cloud_sub;
    message_filters::Subscriber<nav_msgs::msg::Odometry> m_odom_sub;
    std::shared_ptr<message_filters::Synchronizer<SyncPolicy>> m_sync;

    tf2_ros::Buffer m_tf_buffer;
    tf2_ros::TransformListener m_tf_listener;

    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr m_cloud_pub;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<OdinCloudCleaner>());
    rclcpp::shutdown();
    return 0;
}
