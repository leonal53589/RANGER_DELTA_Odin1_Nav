# 实车导航使用指南

以下命令均在工作区根目录执行。全场导航使用 11 月 27 日的 GLIM 图，ODIN1 通过 USB 提供点云，底盘通过 500 kbit/s 的 SocketCAN 提供轮式里程计。构建依赖见[仓库首页](../README.md#构建)。

## 选择入口

| 用途 | 入口 | 地图和 ROS 域 |
|---|---|---|
| 全场导航、RViz 任意目标 | `./scripts/run_glim_navigation.sh` | GLIM 栅格和定位 PCD，默认域 49 |
| ODIN 设备定位、按参考帧返回工位 A | `./scripts/run_real_navigation.sh` | 11 月 24 日设备图与栅格，脚本固定域 44 |
| 无底盘仿真 | `ros2 launch odin_nav_sim navigation_sim.launch.py` | 仿真底盘和测试栅格 |
| ODIN 设备采图 | `./scripts/capture_odin_map.sh` | Qt 中运行 SLAM、暂停并保存，输出 `1.pcd` 与 `map.bin` |

采图脚本生成设备地图。GLIM 参考 PCD 和配套二维栅格来自单独的离线建图流程。两种实车入口共用 USB 和 CAN，切换前先用 Ctrl+C 结束上一组启动进程。[全场地图验证](TEST_RESULTS.md#glim-validation)

## 第一次启动 GLIM 导航

**1. 检查地图和底盘。** 参考点云和设备地图需要单独部署，新检出时先将对应批次文件放到下列路径。设备 YAML 中的 `relocalization_map_abs_path` 也要指向本机的 `map.bin`。设备地图提供配准初值。

```bash
ls -lh maps/glim_20261127/{glim_reference.pcd,site_nav.pgm,site_nav.yaml,localization.yaml}
ls -lh maps/odin_20261124_192624_a0FBCc/{map.bin,odin_relocalization.yaml}
ip -details link show can0
```

若接口为 DOWN，执行 `sudo ip link set dev can0 up type can bitrate 500000`。启动底盘和 ODIN1，保持停车。

**2. 终端一启动定位和 RViz。** 第一次联调用反馈模式，脚本会加载工作空间。

```bash
ENABLE_CONTROL=false ROS_DOMAIN_ID=49 ./scripts/run_glim_navigation.sh
```

**3. 终端二检查观测。** 以下命令均为读取。等待 `accepted=true` 的停车帧，检查 TF 持续更新，再在 RViz 对照车头、墙和障碍。单条拒绝记录可结合后续帧判断。

```bash
source install/setup.bash
export ROS_DOMAIN_ID=49 ROS_LOCALHOST_ONLY=1 PYTHONNOUSERSITE=1
timeout 8s ros2 topic echo /vgicp_localization/status
timeout 5s ros2 run tf2_ros tf2_echo map base_link
ros2 topic echo /navigation_velocity_guard/state --qos-durability transient_local --once
```

停车帧出现 `ambiguous_fit` 时，`/vgicp_localization/status` 的 `candidate_review` 记录候选位姿、验证重合率、分区残差和拒绝原因。`winner` 是单帧通过复核的候选下标，`consistent=true` 表示连续至少 5 帧支持同一个候选。这个字段用于离线和现场对照，导航是否有正式定位仍看主字段 `accepted` 和 `map → odom`。定位节点参数 `review_ambiguity` 控制旁路诊断字段。正式停车恢复通过时会出现 `accepted=true`、`reason=stationary_recovery`，随后可用 `tf2_echo` 检查 TF。 [恢复与回放记录](TEST_RESULTS.md#vgicp-stationary-recovery)

**4. 实车执行目标。** 结束终端一的反馈模式，再运行下列命令。脚本默认启用运动控制。RViz 设 `Fixed Frame = map`，点 **Nav2 Goal**，在可通行区域按下鼠标并拖出终点朝向。途经点使用 Navigation 2 面板的 **Waypoint / Nav Through Poses Mode**，依次添加目标后启动 Navigate Through Poses。需要中止时取消当前任务。

```bash
ROS_DOMAIN_ID=49 ./scripts/run_glim_navigation.sh
```

## RViz 和日志怎么看

实车前进巡航和角速度上限分别为 0.60 m/s、0.60 rad/s。近目标会减速，精停容差为 2.2 cm、0.05 rad，停稳连续复核 0.55 s。需要调整车速时，在实车 YAML 中同步修改 MPPI、速度平滑器和底盘限幅，重启后生效。[实车参数](../src/odin_nav_sim/config/nav2_real.yaml) · [速度对照](TEST_RESULTS.md#controller-speed)

| 想确认的事情 | 看哪里 |
|---|---|
| 地图中原来有什么，现场又新增了什么 | 分别开关 Static Map、Global Costmap、Local Costmap。静态层来自栅格，动态层来自实时点云。 |
| 机器人为什么改变路线 | 对照 Planner Candidates（`/plan`）与 Last Controller Plan（`/controller_server/accepted_plan`），后者是实际交给控制器的路径。 |
| 定位当前是否得到点云支持 | `/vgicp_localization/status` 的 `stamp_ns`、`accepted`、`stationary`、`reason`。有 TF 时仍可能在沿用上一帧有效校正。 |
| 为什么停车 | 先看导航动作是否仍在执行，再对照 `/cmd_vel`、`/cmd_vel_guarded`、`/navigation_velocity_guard/state` 和 `/odom`。 |
| 是否达到实体目标 | 保存原目标、`/controller_server/tracking_endpoint`、有效停车定位帧与卷尺读数。分别计算地图误差和实体误差。 |

[RViz 配置](../src/odin_nav_sim/rviz/real_navigation.rviz) · [代码原理](NAVIGATION_MATHEMATICS_AND_PRINCIPLES.md#glim-runtime) · [实车验收记录](TEST_RESULTS.md#glim-navigation)

## 工位 A 精确返航

本节使用域 44 的 ODIN 设备定位入口。`station_reference` 默认留空，仓库中的 `station_A.npz` 用于历史仿真回归。要启用工位 A 精确返航，使用与当前地图、外参和目标朝向一致的参考帧。

在 `scripts/run_real_navigation.sh` 内设置 `enable_control=false` 后启动。车停在工位 A 的目标位置和朝向、重定位稳定后，在另一个终端从仓库根目录采集：

```bash
source install/setup.bash
export ROS_DOMAIN_ID=44 ROS_LOCALHOST_ONLY=1 PYTHONNOUSERSITE=1
REF="$PWD/local_experiments/stations/station_A_calibrated.npz"
ros2 run odin_nav_sim parking_check capture "$REF"
```

把新文件的绝对路径填到实车脚本的 `station_reference`，启用底盘控制并重启。执行 `ros2 run odin_nav_sim return_to_station "$REF"`，Nav2 先到工位附近，再以 A 的两面固定墙做末端对准。实测对照可用 `--mode baseline` 单独跑进近，用 `--mode align` 单独跑补位。以同一组地面胶带和车身标记尺量实体停点，验收目标为位置误差 ≤3 cm、航向误差 ≤5°。[停车实验记录](TEST_RESULTS.md#21-实车右侧左侧返航到当前-a--11-月-26-日)

## 工位停车复核（单 ODIN1，只读）

在普通实车导航已启动、车停在实体目标工位并朝向所需方向时，另开终端保存参考帧。路径由自己指定。每个参考文件对应一张静态地图和一个工位位姿。

```bash
cd /home/leona/WorkingSpace/navigation2/project_slam
source install/setup.bash
export ROS_DOMAIN_ID=44 ROS_LOCALHOST_ONLY=1 PYTHONNOUSERSITE=1
REF="$PWD/local_experiments/stations/A.npz"
ros2 run odin_nav_sim parking_check capture "$REF"
ros2 run odin_nav_sim parking_check watch "$REF"
```

`capture` 等待轮式里程计连续静止至少 1 秒，按点云时间取 TF，保存底盘系参考点云、地图指纹和工位地图位姿。已有文件会保留。做重复到位试验时，每次返航应发送同一组地图目标，以采参考时打印的 `nav_map_m_rad` 为准；在 RViz 反复手点会引入额外的目标位置和角度误差。运行 `watch` 后再发 RViz 单点目标；每个新观察到的 `NavigateToPose` 成功目标停稳后记录一次。去程停在别处会记录 `outside_station_view`。需要手动测一次时可单独执行 `ros2 run odin_nav_sim parking_check check "$REF"`。也可将参考帧绝对路径填入实车脚本的 `parking_reference`，下次启动时自动监听，两种监听方式选一种。

验证右偏补位决策时，待重定位就绪并停稳，运行 `ros2 run odin_nav_sim parking_check decide "$REF"`。程序采集本次启动后的三帧静止点云，根据横偏中位数和极差输出 JSON。右偏至少 10 cm 且极差小于 2 cm 时建议单次车身左移，步长上限 5 cm。其余输出为跳过或中止，并附原因。该模式只读取话题并打印结果。

输出保存在参考文件旁的 `A.npz.jsonl`，每行对应一次检查。`accepted=true` 表示点云估计通过质量检查。终端的工位偏差以**采参考帧时的车头为 +X、车左为 +Y、逆时针为正角**。`station_offset_m_rad` 和 `nav_offset_m_rad` 分别为点云与导航 TF 的估计，单位为米和弧度。按 `cloud_stamp_ns`、`goal_id` 配对卷尺记录。参考帧与导航需使用同一静态地图，保存在 `local_experiments/stations/`。

复核的初步适用范围为导航估计距工位 ≤0.50 m、朝向差 ≤30°。注册使用三个航向初值；任一不收敛、双向 8 cm 重叠率低于 65%、归一化雅可比奇异值比低于 0.03、解之间相差超过 2.5 cm 或 2°，均拒绝输出工位点云偏差。超过初值修正范围也会拒绝。这些门槛用于实测筛选，实体到位精度用独立尺量评价。
