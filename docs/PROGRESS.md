# ODIN1 建图进度（2026-11-18）

## 已实测

- 设备 `2207:0019` 经 Realtek Hub 以 `5000M`（USB 3.0，5 Gbit/s）连接。ROS 2 Humble 驱动正常出流，点云约 10.2 Hz，IMU 和高频里程计各约 400 Hz。
- Qt 窗口采集约 15 秒，成功保存 [maps/1.pcd](maps/1.pcd)，已验证包含 23,453 个有效 XYZ 点。当前 `custom_map_mode: 0`，主机在 `odom` 坐标系下累加点云。
- 待验证手持移动时的拼接精度、漂移和设备 SLAM 模式的回环效果。

## RViz2 实时查看

在项目根目录执行 `source install/setup.bash`，再执行 `ros2 launch odin_slam slam_bringup.launch.py`。默认会启动驱动、建图控制窗口和 RViz2。

RViz2 的 `Fixed Frame` 设为 `odom`。在 `slam_current` 中订阅 `/odin1/cloud_slam`，把 `Reliability Policy` 设为 `Best Effort`，与驱动匹配。启用 `slam_deacy`（配置中的名称）也要设为 `Best Effort`，可用 `Decay Time` 保留最近几秒的点云。Qt 窗口点「运行 SLAM」后会累加完整点云，当前代码在保存时输出 PCD，RViz2 显示的是实时帧或按时间保留的帧。

## 查看已保存的地图

在项目根目录用 `pcl_viewer maps/1.pcd` 直接打开。也可以在加载了 `install/setup.bash` 的终端把 PCD 发布给 RViz2。

```bash
ros2 run pcl_ros pcd_to_pointcloud --ros-args \
  -p file_name:="$PWD/maps/1.pcd" -p tf_frame:=odom -p publishing_period_ms:=1000
```

在 RViz2 中设 `Fixed Frame` 为 `odom`，添加 `PointCloud2`，话题选 `/cloud_pcd`。已验证该话题发布了全部 23,453 个点，RViz2 窗口显示待实测。 [ROS 2 Humble 的 PCD 发布工具](https://github.com/ros-perception/perception_pcl/blob/humble/pcl_ros/tools/pcd_to_pointcloud.cpp)

建图节点每次启动都从 `1.pcd` 开始编号，再次保存前先把现有地图另存。
