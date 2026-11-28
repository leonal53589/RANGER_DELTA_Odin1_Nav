# Ranger Delta + ODIN1 实验室导航

ROS 2 Humble / Ubuntu 22.04。GLIM 离线构建实验室参考点云，双体素 4-DoF VGICP 将 ODIN1 实时扫描对齐地图，发布 `map → odom`。停车时用多帧固定结构确认多解，行驶中用 CAN 轮式里程计短时衔接。Nav2 使用 SmacPlanner2D 规划和 MPPI 跟踪，动态体素层清理短时障碍残影，末端按纵横误差低速补位并复核停稳。

## 构建

在仓库根目录执行。已有工作区使用 `--symlink-install` 时可沿用原构建方式。

```bash
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash
```

- 首次检出时，用 `vcs import src < dependencies.repos` 获取固定版本的松灵消息和车体模型。[源码清单](dependencies.repos)
- 工程依赖 Nav2（含 MPPI、voxel grid）、TF2、RViz2、PCL、Qt5、OpenCV、libusb、Boost、Eigen3、OpenMP 和可由 CMake 找到的 `small_gicp`。已配置 rosdep 的环境可用 `rosdep install --from-paths src --ignore-src --rosdistro humble -y` 安装已登记的依赖。本机的 `small_gicp` 位于 `~/WorkingSpace/glim_ws/install`，其他安装位置可通过 `CMAKE_PREFIX_PATH` 指定。[构建配置](src/odin_nav_sim/CMakeLists.txt) · [ODIN1 驱动说明](src/odin_ros_driver/README.md)

## 运行

| 任务 | 命令 |
|---|---|
| 全场导航，停车检查定位 | `ENABLE_CONTROL=false ./scripts/run_glim_navigation.sh` |
| 全场导航，允许底盘运动 | `./scripts/run_glim_navigation.sh` |
| ODIN 设备定位与工位 A 返航 | `./scripts/run_real_navigation.sh`，工位闭环需配置参考帧 |
| 导航仿真 | `ros2 launch odin_nav_sim navigation_sim.launch.py` |
| ODIN 设备采图 | `./scripts/capture_odin_map.sh` |

GLIM 入口默认使用 ROS 域 49，设备定位入口使用域 44。首次部署需备齐 GLIM 参考 PCD 和 ODIN 设备地图。地图路径、双终端命令和 RViz 操作见[实车导航指南](docs/NAVIGATION_GUIDE.md)。

实车前进巡航上限为 0.60 m/s，转向上限为 0.60 rad/s。GLIM 精停配置检查地图估计相对路径末点的 2.2 cm、0.05 rad 容差。实体到位精度以现场尺量为准。

## 项目结构

| 目录 | 职责 |
|---|---|
| `src/odin_ros_driver`、`src/odin_slam` | ODIN1 通讯、点云发布、设备采图与保存 |
| `src/ranger_delta_bringup` | CAN 反馈和命令、轮式里程计、车体模型和雷达外参 |
| `src/odin_nav_sim` | VGICP 与设备定位入口、代价地图、路径选择、控制器和末级速度保护 |
| `maps/`、`scripts/` | 地图配置、启动脚本及标定工具 |

[文档索引](docs/README.md) · [数学与代码原理](docs/NAVIGATION_MATHEMATICS_AND_PRINCIPLES.md#glim-runtime) · [实车测试结果](docs/TEST_RESULTS.md#glim-navigation)
