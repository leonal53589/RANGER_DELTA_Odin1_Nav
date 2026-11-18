# ODIN1 建图与导航进度（2026-11-18）

## 已实测

- 设备 `2207:0019` 经 Realtek Hub 以 `5000M`（USB 3.0，5 Gbit/s）连接。ROS 2 Humble 驱动正常出流，点云约 10.2 Hz，IMU 和高频里程计各约 400 Hz。
- Qt 窗口采集约 15 秒，成功保存 [maps/1.pcd](../maps/1.pcd)，已验证包含 23,453 个有效 XYZ 点。当前 `custom_map_mode: 0`，主机在 `odom` 坐标系下累加点云。已确认 RViz2 实时查看正常。
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

## RANGER DELTA 导航计划

- **已确认驱动可编译。** 松灵公开了专用的 ROS 2 `ranger_ros2/air_delta` 分支。`agx_bringup` 已在本机 Ubuntu 22.04 + Humble 的临时工作区编译成功，并通过启动文件解析。官方启动配置订阅 `/cmd_vel`，轮式里程计发布到 `/agx/wheel_odom_raw`。实车 CAN 和运动反馈待验证。 [官方 ROS 2 驱动](https://github.com/agilexrobotics/ranger_ros2/tree/air_delta)
- **阶段 1 已实现。** `odin_nav_sim` 用 Humble Nav2、测试栅格地图和模拟底盘跑 `/navigate_to_pose`，规划器为 `SmacPlanner2D`，控制器为 `RegulatedPurePursuit`。模拟底盘接收 `/cmd_vel`，发布 `/odom` 和 `odom → base_link`，固定 `map → odom`。
- **雷达安装前可接底盘。** 接 USB 转 CAN，确认前进、转弯、自旋、横移和超时停车。核对 `/agx/wheel_odom_raw`、`/motion_mode_feedback` 和 `odom → base_link`。该版本会广播底盘 TF，且里程计速度未填 `linear.y`。先验证前进、转弯和自旋，再评估全向控制。 [驱动里程计源码](https://github.com/agilexrobotics/ranger_ros2/blob/fb96124fa68c49bab03d8ddd2294c2afd45af925/agx_bringup/include/global_state.hpp#L499-L588)
- **安装后接定位和避障。** 标定 `base_link → imu`，接真实地图、实时点云和 `map → odom`。确认地图、车体和点云对齐后，低速验证首个目标点。`/odom` 及每段 TF 都由单一来源发布。

### 运行阶段 1 仿真

在项目根目录执行：

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-select odin_nav_sim --symlink-install
source install/setup.bash
ros2 launch odin_nav_sim navigation_sim.launch.py
```

RViz2 启动后，等 `Navigation` 显示 `active`。用 `Nav2 Goal` 在隔墙右侧点下并拖出目标朝向，可以看绿色路径从上方通道绕过去。再发一个远处目标，在运动中点 `Cancel` 可验收停车。仿真使用固定地图对齐，所以面板的 `Localization: inactive` 属于预期。`mode_switch_delay` 默认为 `0.35` 秒。

**仿真验收通过。** 在 RViz2 发送 `(5.60 m, 0.00 m, 90°)`，底盘从上方通道绕墙，到点位置误差约 `0.07 m`、朝向误差约 `3.2°`。返回目标在运动中取消，约 `0.8 s` 后速度归零。另实测换模式期间保持静止，指令超时及切换中收到零速度均停车。

本机 RViz2 在退出时返回 `-11`。地图显示、发送目标和取消均已实测通过。仅启动导航时可加 `launch_rviz:=false`。
