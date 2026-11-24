# Ranger Delta + ODIN1 导航

基于 Ubuntu 22.04、ROS 2 Humble 和 Nav2。ODIN1 通过 USB 发布点云和定位数据。Ranger Delta 通过 SocketCAN 接收运动指令并回传底盘状态。RViz 用于查看地图、代价地图和路径，以及发送单点或途经点目标。

## 项目结构

| 目录 | 职责 |
|---|---|
| `src/odin_ros_driver` | ODIN1 驱动，发布实时点云、SLAM 点云和设备位姿 |
| `src/odin_slam` | 累加 SLAM 点云，保存时做裁剪、体素降采样和离群点剔除 |
| `src/ranger_delta_bringup` | 底盘 CAN 收发、轮式里程计、车体 URDF 和雷达外参 |
| `src/odin_nav_sim` | 实车与仿真启动、定位桥、Nav2 配置、行为树、底盘控制适配和速度保护 |
| `src/odin_cloud_cleaner`、`src/vendor` | 可选的点云预处理，以及固定版本的松灵驱动和车体模型 |
| `maps`、`scripts` | 场地地图、采图与导航入口 |

## 数据流与算法

```text
ODIN1 重定位位姿 + 雷达外参 + 轮式里程计 → map → odom → base_link → lidar
静态栅格 + 实时点云 → 代价地图 → SmacPlanner2D → 路径 → MPPI / 末端停车
/cmd_vel_nav → 速度平滑 → /cmd_vel → 速度保护 → /cmd_vel_guarded → 底盘 CAN
```

- **定位。** 设 $T^A_B$ 将 $B$ 系坐标变换到 $A$ 系。定位桥在共同采样时刻，用外参逆变换将 ODIN1 雷达位姿换算为车体位姿并投影到平面，再求 $T^{map}_{odom}=T^{map}_{base}(T^{odom}_{base})^{-1}$。底盘反馈节点积分轮速和转向反馈，发布 `odom → base_link`。 [定位桥](src/odin_nav_sim/scripts/odin_localization.py)
- **规划与跟踪。** 全局代价地图叠加静态栅格与实时障碍，局部代价地图在 `odom` 下随车滚动。点云回波标记障碍，射线清除经过的动态栅格。SmacPlanner2D 在全局代价地图搜索，MPPI 用 Ackermann 模型采样控制序列，按路径、目标和障碍代价对样本加权。行驶时满足 $|\omega|\leq |v|/R_{\min}$，当前 $R_{\min}=0.50\,\mathrm{m}$。 [实车参数](src/odin_nav_sim/config/nav2_real.yaml)
- **遇阻与到点。** 单点和途经点共用遇阻逻辑。近段受阻先停等，远段受阻且近段有效时可切换到同向替代路径，持续无路则结束目标。途经点在近折返处停稳转向。末端按同一时刻的 TF 计算路径末点与车体的距离、航向误差，轮速和误差持续达标 0.3 秒后报到。 [遇阻逻辑](src/odin_nav_sim/src/obstacle_navigation.cpp) · [控制器](src/odin_nav_sim/src/delta_pursuit_controller.cpp)
- **速度保护。** 根据指令速度和实测轮速检查近处点云。平移方向的检查距离采用 $d=m+v t_r+v^2/(2a)$，其中 $m$ 为距离余量，$t_r$ 为反应时间，$a$ 为配置的减速度。近障命中或点云、里程计、TF、速度指令超时会输出零速。 [速度保护节点](src/odin_nav_sim/scripts/navigation_velocity_guard.py)

## 构建

依赖已就绪后，在加载了 ROS 2 Humble 的终端进入仓库根目录构建。

```bash
colcon build
```

新工作区需要准备以下依赖。

- **第三方源码。** `dependencies.repos` 固定两个松灵仓库的提交。 `ranger_ros2` 提供 `agx_bringup` 的底盘消息，`ugv_gazebo_sim` 提供 Ranger Delta 的 URDF 和网格。安装 `python3-vcstool` 后执行 `vcs import src < dependencies.repos`，源码会放到 `src/vendor/`。 [源码版本清单](dependencies.repos)
- **ROS 2 与系统库。** 加载 `/opt/ros/humble/setup.bash`。项目使用 Nav2（含 MPPI）、RViz2、TF2，以及 PCL、Qt5、OpenCV 等开发库。ODIN1 驱动需要 `libusb-1.0-0-dev`，松灵源码用到 `libboost-dev`。已配置 rosdep 时，可用 `rosdep install --from-paths src --ignore-src --rosdistro humble -y` 安装包清单中声明的依赖。 [ODIN1 驱动依赖与 USB 权限](src/odin_ros_driver/README.md)

已有工作区若使用 `--symlink-install` 构建，增量构建可继续沿用该选项。

## 运行

在新终端从仓库根目录执行 `source install/setup.bash`。

| 任务 | 仓库根目录下的命令 |
|---|---|
| 导航仿真 | `ros2 launch odin_nav_sim navigation_sim.launch.py` |
| 采图 | `./scripts/capture_odin_map.sh` |
| 实车导航 | `./scripts/run_real_navigation.sh` |

- **采图。** 在 Qt 窗口启动 SLAM，低速绕行后停稳、暂停并保存。RViz 的 Fixed Frame 设为 `odom`。采集目录包含点云 `1.pcd` 和设备重定位地图 `map.bin`。
- **实车。** 在启动脚本中设置地图目录、雷达外参和 CAN 接口。导航加载二维栅格 `site_nav.yaml`、`site_nav.pgm`，重定位加载同批次的 `map.bin`。移动地图目录后更新 `odin_relocalization.yaml` 中的设备地图绝对路径。脚本默认启用底盘控制，停车联调时将脚本中的 `enable_control` 改为 `false`。 [实车启动脚本](scripts/run_real_navigation.sh) · [CAN 与停车验收](docs/PROGRESS.md#终端-1-启动导航)
