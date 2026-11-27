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

`scripts/calib/` 用于调整雷达安装位置后重新采集和求解外参。到点时间 A/B 模式使用 `build_goal_time_test.sh` 和配套补丁，`replay_parking_goal_checker.py` 可离线复核到点判定。针对历史录包的一次性试验放在本机 `local_experiments/`，Git 忽略该目录。

普通导航使用 11 月 24 日的设备重定位地图与栅格。11 月 27 日的 GLIM 全场点云用于离线定位对照。[停车定位结果与数据索引](docs/TEST_RESULTS.md#glim-validation)

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

### 工位 A 精确返航

实车脚本默认使用标定后的 ODIN1 外参运行普通导航。`station_reference` 默认留空，仓库中的 `station_A.npz` 用于历史仿真回归。要启用工位 A 精确返航，先用当前地图和外参重采参考帧。

用 `enable_control=false` 启动实车导航。车停在工位 A 的目标位置和朝向、重定位稳定后，在另一个终端从仓库根目录采集：

```bash
source install/setup.bash
export ROS_DOMAIN_ID=44 ROS_LOCALHOST_ONLY=1 PYTHONNOUSERSITE=1
REF="$PWD/log/stations/station_A_calibrated.npz"
ros2 run odin_nav_sim parking_check capture "$REF"
```

把新文件的绝对路径填到实车脚本的 `station_reference`，启用底盘控制并重启。执行 `ros2 run odin_nav_sim return_to_station "$REF"`，Nav2 先到工位附近，再以 A 的两面固定墙做末端对准。实测对照可用 `--mode baseline` 单独跑进近，用 `--mode align` 单独跑补位。以同一组地面胶带和车身标记尺量实体停点，验收目标为位置误差 ≤3 cm、航向误差 ≤5°。[停车实验记录](docs/TEST_RESULTS.md#21-实车右侧左侧返航到当前-a--11-月-26-日)

### 工位停车复核（单 ODIN1，只读）

在普通实车导航已启动、车停在实体目标工位并朝向所需方向时，另开终端保存参考帧。路径由自己指定。每个参考文件对应一张静态地图和一个工位位姿。

```bash
cd /home/leona/WorkingSpace/navigation2/project_slam
source install/setup.bash
export ROS_DOMAIN_ID=44 ROS_LOCALHOST_ONLY=1 PYTHONNOUSERSITE=1
REF="$PWD/log/stations/A.npz"
ros2 run odin_nav_sim parking_check capture "$REF"
ros2 run odin_nav_sim parking_check watch "$REF"
```

`capture` 等待轮式里程计连续静止至少 1 秒，按点云时间取 TF，保存底盘系参考点云、地图指纹和工位地图位姿。已有文件会保留。做重复到位试验时，每次返航应发送同一组地图目标，以采参考时打印的 `nav_map_m_rad` 为准；在 RViz 反复手点会引入额外的目标位置和角度误差。运行 `watch` 后再发 RViz 单点目标；每个新观察到的 `NavigateToPose` 成功目标停稳后记录一次。去程停在别处会记录 `outside_station_view`。需要手动测一次时可单独执行 `ros2 run odin_nav_sim parking_check check "$REF"`。也可将参考帧绝对路径填入实车脚本的 `parking_reference`，下次启动时自动监听，两种监听方式选一种。

验证右偏补位决策时，待重定位就绪并停稳，运行 `ros2 run odin_nav_sim parking_check decide "$REF"`。程序采集本次启动后的三帧静止点云，根据横偏中位数和极差输出 JSON。右偏至少 10 cm 且极差小于 2 cm 时建议单次车身左移，步长上限 5 cm。其余输出为跳过或中止，并附原因。该模式只读取话题并打印结果。

输出保存在参考文件旁的 `A.npz.jsonl`，每行对应一次检查。`accepted=true` 表示点云估计通过质量检查。终端的工位偏差以**采参考帧时的车头为 +X、车左为 +Y、逆时针为正角**。`station_offset_m_rad` 是车停在工位的哪个方向，单位为米、弧度。`nav_offset_m_rad` 是同一坐标系下的导航 TF 估计。可按 `cloud_stamp_ns`、`goal_id` 配对卷尺记录。参考文件需与实车使用同一静态地图；若清理 `log/` 后仍需保留工位，采集时选用其他持久目录。

复核的初步适用范围为导航估计距工位 ≤0.50 m、朝向差 ≤30°。注册使用三个航向初值；任一不收敛、双向 8 cm 重叠率低于 65%、归一化雅可比奇异值比低于 0.03、解之间相差超过 2.5 cm 或 2°，均拒绝输出工位点云偏差。超过初值修正范围也会拒绝。这些门槛用于实测筛选，实体到位精度用独立尺量评价。
