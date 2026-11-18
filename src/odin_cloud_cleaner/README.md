# odin_cloud_cleaner

`odin_cloud_cleaner` 是一个 ROS 2 点云清理节点，用于对留形Odin1空间记忆模组的 SLAM 点云做轻量预处理，并剔除机器人本体及其下方的杂散点，输出更适合后续建图、重定位或局部代价地图分析的清理后点云。

## 主要功能

- **时间同步**：基于 `message_filters::ApproximateTime` 同步订阅 SLAM 点云和高频里程计。
- **NaN 点剔除**：使用 `pcl::removeNaNFromPointCloud` 移除点云中的无效 NaN 点。
- **坐标变换**：结合高频里程计位姿（`odom -> odin_imu_link`）与 TF 变换（`odin_imu_link <- odin_lidar_link`），将点云从 `odom` 坐标系转换到雷达坐标系（`odin_lidar_link`）。
- **本体裁切（CropBox 负向滤波）**：在雷达坐标系下剔除机器人车体及下方区域内的点（保留边界及框外点）：
  - `x`: `[-1.0, 1.0)`
  - `y`: `[-1.0, 1.0)`
  - `z`: `(-inf, 0.0)`
- **坐标还原与发布**：将过滤后的点云重新变换回 `odom` 坐标系并发布。
- **RGB 色彩保留**：采用 `pcl::PointXYZRGB` 格式处理，完整透传 SLAM 点云的 RGB 颜色通道。

> [!NOTE]
> 原实现中的体素栅格下采样（`0.05m`）已被移除，降采样操作移交至下游模块（如重定位、代价地图等）按需统一处理，以保留最高点云细节。

## 输入

| 类型 | Topic / TF | 消息类型 | QoS | 说明 |
| --- | --- | --- | --- | --- |
| 点云 | `/odin1/cloud_slam` | `sensor_msgs/msg/PointCloud2` | `SensorDataQoS` | 留形 Odin1 SLAM 点云（`header.frame_id: odom`）。 |
| 里程计 | `/odin1/odometry_highfreq` | `nav_msgs/msg/Odometry` | `SensorDataQoS` | 高频里程计（400 Hz），提供 `odom -> odin_imu_link` 位姿。 |
| TF | `odin_imu_link <- odin_lidar_link` | `geometry_msgs/msg/TransformStamped` | - | 雷达本体坐标系到 IMU 坐标系的外参变换。 |

当前节点没有声明外部可配置的 ROS 参数，上述 topic、坐标系及滤波范围均在源码中固定。

## 输出

| Topic | 消息类型 | QoS | 说明 |
| --- | --- | --- | --- |
| `/cloud_cleaned` | `sensor_msgs/msg/PointCloud2` | `SensorDataQoS` | 清理后的点云，`header` 沿用输入点云的 `header`（`frame_id` 为 `odom`）。 |

## 构建与运行

在工作空间根目录构建：

```bash
colcon build --packages-select odin_cloud_cleaner --symlink-install
```

运行节点：

```bash
ros2 run odin_cloud_cleaner odin_cloud_cleaner
```
