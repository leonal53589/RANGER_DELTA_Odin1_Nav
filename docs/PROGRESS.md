# ODIN1 建图与导航进度

## 当前阶段 · 2026-11-28

全场入口使用 GLIM 地图、双体素 4-DoF VGICP 和 Nav2 精停。实车静止冷启动已验证多帧恢复。弱约束路段可用轮式里程计限时、限距衔接。控制器前进和转向上限调至 0.60 m/s、0.60 rad/s，仿真的三种到点动作和遇障停车回归通过，现场反馈加速效果良好。全场实体 ≤3 cm、≤5° 仍待跨区域独立尺量。[启动](NAVIGATION_GUIDE.md) · [冷启动复测](TEST_RESULTS.md#vgicp-stationary-recovery) · [控制器对照](TEST_RESULTS.md#controller-speed)

## 历史阶段 · 2026-11-24

以下按当时的设备地图、外参和程序版本保留。

- 新场地地图已通过短程实车导航和重复绕障。控制器按 Ranger Delta 的 0.50 m 最小行驶转弯半径跟踪路径，普通导航使用 10 cm 到点容差。按[实车启动步骤](#终端-1-启动导航)完成停车验收后，再开启底盘控制。
- 同一次定位会话的实车 A/B 中，时间对齐版从 **10.79 s 缩短到 7.19 s** 报到，停稳用时从 **11.50 s 缩短到 7.47 s**。这版仍在实验入口验证。最新对齐轮尺量距 B **2.94 cm**，后续优先核对实体目标到地图的换算以及跨会话定位偏差。
- 实车脚本默认加载 `maps/odin_20261124_192624_a0FBCc` 的 5 cm 栅格和同次采集的 `map.bin`。新图已通过本轮实车导航测试。栅格下缘碰到 `Y = −10 m` 的 PCD 裁剪线，上方窄口在当前代价图中与主通道断开。
- 本地录包和调试日志已移出工作区，以下旧日志链接可在恢复 `log/` 后查看。

### 本轮导航可靠性改动

1. **先看能否停车。** 实车速度链为 `/cmd_vel_nav → /cmd_vel → /cmd_vel_guarded → 底盘 CAN`。末级按 40 Hz 检查原始点云、轮速、地图定位和速度指令。车周围已观测到的近障以及输入超时会触发零速。定位检查扣除定位桥的 TF 前置时间。状态变化发布到 `/navigation_velocity_guard/state`。
2. **再看能否恢复任务。** 单点与途经点共用等待和绕行逻辑。全局和局部代价地图使用体素层，0.12–1.35 m 的回波标障，地面回波也可沿三维射线清障。未被射线穿过的已标体素会保留。场地栅格由全局静态层提供。
3. **最后验收实体精度。** 固定同一实体 B 和地图目标，直行、左转后接近、右转后接近各做 5 轮。每轮记动作结果、原目标、控制器接受的路径末点（`/controller_server/tracking_endpoint`）、停稳时刻的地图 TF、R/F 到两条基准线的四个读数。用两标记中点相对示教中点的距离衡量实体误差，同时记录两标记连线的角度变化。按组统计成功次数、中位和最大误差，并注明卷尺单读数约 ±0.5 cm。物理停车距离、盲区与上述精度数据待本版实车复验。

**隔离仿真结果（2026-11-24）**

- 1.20 m 通道在第 6 秒移开障碍，只留下地面回波。原配置在第 12.5 秒仍有 14 个致命格并最终失败，新配置同一时刻剩 0 格，约 17.6 秒到达。保持障碍或只让射线从高桌沿下方穿过，对照区域的 16 个致命格均保留。
- 注入位置固定、航向往复 ±8.6° 的故障，约 12 秒触发等待，约 33 秒返回失败。正常起步掉头并在终点反向对准成功。行为树按轮式位移在路径出发方向的投影检查净进展，转向只按角误差的最佳改善给予有限时间。
- 动态绕行比较前方最多 3.5 m 的候选路径，明显不同的绕行在停车后连续两次规划一致才接受。三轮横向摆动障碍均绕行到达，每轮控制器接受初始路径和一条绕行路径。在 0.60 m 窄口，重复跟踪失败会间隔重试；一次对照中控制器接收路径由 16 次降至 5 次，约 24 秒退出。0.70 m 和 1.20 m 直通道均通过。
- 实车 RViz 默认显示 `Last Controller Plan`（`/controller_server/accepted_plan`），保留最近一次接受的路径。勾选 `Planner Candidates`（`/plan`）可对照候选路径。现场仍需复验 ODIN1 的地面覆盖、遮挡和制动距离。

**隔离仿真复验**（在项目根目录运行）

```bash
source install/setup.bash
ROS_DOMAIN_ID=87 ROS_LOCALHOST_ONLY=1 PYTHONNOUSERSITE=1 \
  launch_test src/odin_nav_sim/test/obstacle_scenarios.launch.py
```

实车先用下方停车验收入口启动。对照平滑速度、受保护速度、轮速与保护原因。低速制动测试使用软障碍物，分别记录首次有效近点、保护输出零速和轮速持续归零的时刻。以最差实测反应时间及减速度核对配置。ODIN1 安装高度 1.125 m，低箱子、贴身盲区和车后方的覆盖需要现场单独测量。CAN 断线时需要核对底盘自身的失联停车行为。

## 已实测

- 设备 `2207:0019` 经 Realtek Hub 以 `5000M`（USB 3.0，5 Gbit/s）连接。ROS 2 Humble 驱动正常出流，点云约 10.2 Hz，IMU 和高频里程计各约 400 Hz。
- Qt 窗口采集约 15 秒，初版样图包含 23,453 个有效 XYZ 点。当前 `custom_map_mode: 0`，主机在 `odom` 坐标系下累加点云。已确认 RViz2 实时查看正常。
- 待验证手持移动时的拼接精度、漂移和设备 SLAM 模式的回环效果。

## RViz2 实时查看

在项目根目录执行 `source install/setup.bash`，再执行 `ros2 launch odin_slam slam_bringup.launch.py`。默认会启动驱动、建图控制窗口和 RViz2。

RViz2 的 `Fixed Frame` 设为 `odom`。在 `slam_current` 中订阅 `/odin1/cloud_slam`，把 `Reliability Policy` 设为 `Best Effort`，与驱动匹配。启用 `slam_deacy`（配置中的名称）也要设为 `Best Effort`，可用 `Decay Time` 保留最近几秒的点云。Qt 窗口点「运行 SLAM」后会累加完整点云，当前代码在保存时输出 PCD，RViz2 显示的是实时帧或按时间保留的帧。

## 查看已保存的地图

在项目根目录指定要查看的采图文件。也可用同一文件向 RViz2 发布点云。

```bash
PCD="$PWD/maps/odin_20261124_192624_a0FBCc/1.pcd"
pcl_viewer "$PCD"
# 或者在已加载工作空间的终端发布
ros2 run pcl_ros pcd_to_pointcloud --ros-args \
  -p file_name:="$PCD" -p tf_frame:=odom -p publishing_period_ms:=1000
```

在 RViz2 中设 `Fixed Frame` 为 `odom`，添加 `PointCloud2`，话题选 `/cloud_pcd`。早期的 15 秒样图曾用该话题验证 23,453 个点。 [ROS 2 Humble 的 PCD 发布工具](https://github.com/ros-perception/perception_pcl/blob/humble/pcl_ros/tools/pcd_to_pointcloud.cpp)

建图节点每次启动都从 `1.pcd` 开始编号，再次保存前先把现有地图另存。

## RANGER DELTA 导航计划

- **松灵依赖已纳入工作空间。** `agx_bringup` 位于 `src/vendor/ranger_ros2`，由 [dependencies.repos](../dependencies.repos) 固定在已验证的 `air_delta` 提交。官方启动配置订阅 `/cmd_vel`，轮式里程计发布到 `/agx/wheel_odom_raw`。 [官方 ROS 2 驱动](https://github.com/agilexrobotics/ranger_ros2/tree/air_delta)
- **实车收帧已通过。** `can0` 在 `500000 bit/s` 下连续收到状态、运动和模式反馈。4 秒 `candump` 抓到 3,418 帧，发送帧数为 0。
- **阶段 1 已实现。** `odin_nav_sim` 用 Humble Nav2、测试栅格地图和模拟底盘跑 `/navigate_to_pose`，规划器为 `SmacPlanner2D`，控制器为 `RegulatedPurePursuit`。模拟底盘接收 `/cmd_vel`，发布 `/odom` 和 `odom → base_link`，固定 `map → odom`。
- **雷达安装前可接底盘。** 接 USB 转 CAN，确认前进、转弯、自旋、横移和超时停车。核对 `/agx/wheel_odom_raw`、`/motion_mode_feedback` 和 `odom → base_link`。该版本会广播底盘 TF，且里程计速度未填 `linear.y`。先验证前进、转弯和自旋，再评估全向控制。 [驱动里程计源码](https://github.com/agilexrobotics/ranger_ros2/blob/fb96124fa68c49bab03d8ddd2294c2afd45af925/agx_bringup/include/global_state.hpp#L499-L588)
- **安装后接定位和避障。** 标定 `base_link → imu`，接真实地图、实时点云和 `map → odom`。确认地图、车体和点云对齐后，低速验证首个目标点。`/odom` 及每段 TF 都由单一来源发布。

### 阶段 2 底盘反馈回放

`ranger_delta_bringup` 使用松灵 `agx_bringup` 的消息类型。节点发布 `/system_status`、`/chassis_motion_feedback`、`/motion_mode_feedback`、`/agx/wheel_odom_raw` 和可选的 `odom → base_link`。默认回放 `candump -L` 日志，在本机 ROS 域 `117` 运行。回放只读文件，实车反馈模式也仅接收 CAN 数据。位姿是根据底盘反馈积分得到的估计，行驶方向和尺度仍待实车验收。

在项目根目录编译底盘包及其依赖。新检出工作空间时先按 [README](../README.md#构建) 导入松灵源码。回放参数填写实际保存的 `candump -L` 文件路径。

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-up-to ranger_delta_bringup --symlink-install
source install/setup.bash
ros2 launch ranger_delta_bringup feedback.launch.py \
  replay_file:=/path/to/delta-capture.log
```

回放只有约 4 秒。先在另一个加载了相同环境的终端执行 `export ROS_DOMAIN_ID=117 ROS_LOCALHOST_ONLY=1`，再用 `ros2 topic echo /agx/wheel_odom_raw` 或 `ros2 run tf2_ros tf2_echo odom base_link` 等待数据。

**离线验收通过。** 真实抓包回放得到 191 条系统状态、191 条运动反馈、191 条模式反馈和 190 条里程计。待机电压解码为 `26.5 V`，静止位姿和 TF 均正确。回放未打开 CAN 套接字。

**实车静止验收通过。** 只启动 `feedback.launch.py source:=socketcan` 接收数据。4 秒内收到 190 条里程计及持续的状态和运动模式反馈，`odom → base_link` 可查询，位移、转角和速度反馈均为零。CAN 发送计数增量为 0。

### 阶段 2 导航底盘接口

`chassis.launch.py` 订阅 `/cmd_vel`（`geometry_msgs/msg/Twist`），将反馈里程计发布到 Nav2 使用的 `/odom`，并发布 `odom → base_link`。速度转换支持前后阿克曼、斜移、自旋和停车，发送前会检查状态、反馈时效和模式切换是否完成。默认控制开关 `enable_control` 为 `false`，只能在启动时设置。回放模式禁止启用控制。

加载上面的工作区环境后，可在回放域只检查接口：

```bash
ros2 launch ranger_delta_bringup chassis.launch.py \
  replay_file:=/path/to/delta-capture.log
```

离线验收确认 `/cmd_vel` 有一个订阅者、零发布者，`/odom` 和 TF 连续发布，两个节点都未打开 CAN 套接字。通过 10 项协议与控制逻辑测试。真实前进、转向、自旋、停车及里程计方向验收等装车后再进行。

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

## 真机运控测试与舵轮诊断记录工具 (2026-11-19)

### 1. 实测反馈与版本回退
- 用户在真机上复测横移，发现向左横移时 3 个轮子依然未转过来，确认单纯的指令延时未解决根因。
- **按用户要求完成代码回退**：将控制逻辑 [`commands.py`](../src/ranger_delta_bringup/ranger_delta_bringup/commands.py) 和 [`command_node.py`](../src/ranger_delta_bringup/ranger_delta_bringup/command_node.py) 完全回退至初始稳定版本，保持原有控制流不变。

### 2. 舵轮状态监测与诊断记录工具 (`ranger_delta_wheel_monitor`)
为了精准排查各舵轮电机转角、是否卡阻以及是否到位，新增了全套舵轮实时诊断与记录工具：
1. **轮子朝向状态自动判定**:
   - **竖向 (纵向/直行)**: 角度在 0° 或 180° 附近（允许 ±15° 偏差）。
   - **横向 (平移/横移)**: 角度在 +90° 或 -90° 附近（允许 ±15° 偏差，即 75°~105°）。
   - **斜向 (倾斜/过渡)**: 其余角度。
2. **多源数据监听与解析**:
   - 解析 CAN `0x271` 四轮转角（`W1 右前`, `W2 右后`, `W3 左后`, `W4 左前`）。
   - 解析 CAN `0x281` 四轮驱动转速。
   - 解析 CAN `0x291` 运动模式与切换状态。
   - 解析 CAN `0x265~0x268` 转向舵机低速驱动器状态（电压、温度、使能状态、故障报警位）。
3. **独立终端交互看板 (CLI Live-TUI)**:
   - 可通过 `ros2 run ranger_delta_bringup ranger_delta_wheel_monitor` 启动。
   - 终端实时高亮刷新 4 轮角度、指向状态标签（`[ 竖向 ]` / `[ 横向 ]` / `[ 斜向 ]`）、目标到位状态以及舵电机健康度。
   - 按 `Ctrl+C` 退出时自动输出**诊断总结报告**，统计每个轮子在测试期间达到的最小/最大角度，明确指出哪几个轮子未转到横向。
4. **CSV 时序数据自动记录**:
   - 自动在 `logs/wheel_monitor/wheel_record_YYYYMMDD_HHMMSS.csv` 中按帧记录各轮转角、状态与指令，便于离线复盘与波形绘制。
5. **UI 控制面板集成**:
   - [`control_ui.py`](../src/ranger_delta_bringup/ranger_delta_bringup/control_ui.py) 集成 4 轮状态看板，并提供“⏺ 开始录制轮子数据 / ⏹ 停止录制并生成诊断”一键操作按钮。

## Ranger Delta 舵轮限位机理分析与官方手册剖析 (2026-11-19)

### 1. 官方文档分析 (`aen0mytav3n9e1wx#60c244ac`)
结合松灵官方用户手册深入分析：
1. **底盘构型差异（三轮三转 Delta vs 四轮四转 Air）**:
   - Ranger Delta 是 **三轮三转** 三角形底盘（W1 右前、W2 右后、W3 左后，无 W4），转向电机传动结构受三角形连杆几何约束。
   - 松灵开源 ROS 2 驱动仓库 `ranger_ros2` 的 `air_delta` 分支由 Ranger Air（四轮 360° 独立舵轮）直接复用至 Delta，未对 Delta 底盘的连杆限位做针对性运动学转换。
2. **官方遥控与灯效对“横行”的定义**:
   - 手册第 2.1 节状态指示明确写道：`横行模式运行（斜行角度90时）：向左运行时，左侧两个灯灯条显示黄色，向右运行时，右侧两个灯灯条显示黄色`。
   - 手册第 2.3 节遥控说明明确写道：`SWD拨到下为斜移模式：左摇杆控制速度，右摇杆控制转角（最大角度90°即为横移）`。
   - **关键设计**: 官方对横向运动的控制逻辑是“**舵角保持 90° 横向，通过正反向驱动速度区分向左还是向右**”，舵轮并不需要旋转到 -90°。

### 2. 真机转角限位实测验证 (`ranger_delta_steer_verifier`)
- 下发 `+45.0°` (CAN `+785 mrad`): 实测所有三轮精准停在 `-29.9° / -30.0°`，受到连杆和固件的负向偏转硬限位截断。
- 下发 `-90.0°` (CAN `-1571 mrad`): 实测所有三轮精准到位 `+89.9°`，正向横置完全畅通。
- CAN 0x111 转向指令符号与实车物理转角相反（下发负值对应舵角正向旋转）。

### 3. 运控映射修复与闭环
- 在 [`commands.py`](../src/ranger_delta_bringup/ranger_delta_bringup/commands.py) 中重构 Crab 横移模式运动学映射：
  - 纯向右横移 ($v_y < 0$): 舵角设为 `-pi/2` (CAN `-1571`，轮子转到 `+90°`)，速度下发正值。
  - 纯向左横移 ($v_y > 0$): 舵角同样设为 `-pi/2` (CAN `-1571`，轮子转到 `+90°`)，速度下发负值（反向驱动）。
- 彻底杜绝向左横移时向舵轮下发 `+1571` 撞击 `-30°` 极限导致的斜向窜动，实现向左向右均保持横向纯平移。

## ODIN1 实车导航复测（2026-11-21）

- 上一版地图已验证 ODIN1 重定位和实车导航。定位桥发布 `map → odom`，底盘发布 `odom → base_link`，局部和全局代价地图均接入 `/odin1/cloud_raw`。
- 双轮一侧为车头 `+X`，单轮在车尾。雷达外参初值为 `xyz=(0.195, 0, 0.823) m`、`rpy=(0, 0, 0)`。三轮等速自旋的理论中心在 `base_link` 前方约 43 mm，实际旋转中心和安装外参仍待独立测量。 [车体模型](../src/ranger_delta_bringup/urdf/ranger_delta.urdf.xacro)
- 绕椅子调试用居中路径、0.22 m 前视距离和 0.04 m/s 线速度上限，分两段走完约 2.30 m，并在 0.10 m 到点容差内停稳。标准 RViz 入口加载 [实车默认参数](../src/odin_nav_sim/config/nav2_real.yaml)，由 Nav2 根据当前代价地图规划。
- 上一版地图省略 `odin_reference_pcd` 后，4 次目标有 3 次成功，1 次报 `Failed to make progress`。启用参考 PCD 时曾出现 ODIN1 报重定位成功、定位桥仍未发布 `map → odom`，以及行驶中 TF 过期。参考 PCD 复核在回放中拦截过约 13 cm 的错误校正，但实时点云与旧图重叠不足时也会阻止 TF 更新。

## 场地地图验收（2026-11-24）

- 11 月 21 日的试验地图曾通过停车重定位和短距离导航。本轮已用全场采集的 [5 cm 栅格配置](../maps/odin_20261124_192624_a0FBCc/site_nav.yaml) 和同批次设备地图替换。

### 终端 1 启动导航

2026-11-24 重新固定 ODIN1。当前 `base_link → lidar` 平移为 `(0.210, 0, 1.125) m`，三个安装角均为 `0 rad`。项目 `base_link` 原点在地面平面，`sensor_z` 直接使用量得的离地高度。

底盘和 ODIN1 上电，先关闭之前单独启动的 ODIN1、底盘反馈和导航节点。用 `ip -details link show can0` 检查 CAN 接口，未启用时运行 `sudo ip link set dev can0 up type can bitrate 500000`。

实车重定位需要同次采集的本地 `map.bin`。换电脑时把它复制到地图目录，并把 `odin_relocalization.yaml` 里的 `relocalization_map_abs_path` 更新为本机绝对路径。参考点云 `1.pcd` 只在启用复核时需要。

```bash
cd /home/leona/WorkingSpace/navigation2/project_slam
source /opt/ros/humble/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=44 ROS_LOCALHOST_ONLY=1 PYTHONNOUSERSITE=1
MAP_DIR="$PWD/maps/odin_20261124_192624_a0FBCc"

ros2 launch odin_nav_sim real_navigation.launch.py \
  map:="$MAP_DIR/site_nav.yaml" \
  odin_relocalization_config:="$MAP_DIR/odin_relocalization.yaml" \
  sensor_x:=0.210 sensor_y:=0.0 sensor_z:=1.125 \
  sensor_roll:=0.0 sensor_pitch:=0.0 sensor_yaw:=0.0 \
  can_interface:=can0 enable_control:=false launch_rviz:=true
```

### 终端 2 停车验收

```bash
cd /home/leona/WorkingSpace/navigation2/project_slam
source install/setup.bash
export ROS_DOMAIN_ID=44 ROS_LOCALHOST_ONLY=1 PYTHONNOUSERSITE=1

timeout 5s ros2 topic echo /system_status --once
timeout 10s ros2 run tf2_ros tf2_echo map base_link
ros2 lifecycle get /controller_server
ros2 topic echo /navigation_velocity_guard/state --qos-durability transient_local --once
ros2 topic info /cmd_vel_guarded --verbose
```

把车停在新图采图起点附近，朝向尽量与采图时一致。确认底盘状态正常、TF 持续更新、控制器为 `active`。实车 RViz 默认叠加静态地图、全局和局部代价地图以及全局路径。可单独勾选两张代价地图，观察障碍出现或消失后路径是否变化。核对地图对齐时，在 `map` 固定坐标系下添加 `/odin1/cloud_raw`，对照现场的墙线和门框。

停车验收通过后，停止上一轮导航，在项目根目录运行 `./scripts/run_real_navigation.sh`。脚本默认启用底盘控制和 RViz，地图目录、雷达外参和启动选项都在脚本开头修改。用 **Nav2 Goal** 先测空地短直行、到点朝向和取消停车，再跑一组绕障路线。随后用未写入静态图的可移动物体验证实时标记与清除，记录 `/plan`、`/cmd_vel`、`/odom`、TF 和两张代价地图。出现进度超时时，先对照定位和实际运动，再调整控制参数。

默认 `goal_profile:=navigation` 用于通行，路径末端容差为 10 cm、0.20 rad。需要停车对齐时可用 `goal_profile:=precision`，路径末端容差为 6 cm、0.10 rad，规划搜索容差为 3.5 cm。验收时同时记录 RViz 原目标和栅格路径末点。网格中心量化与规划容差都会影响末点位置。

实车行驶段由 Nav2 MPPI 生成速度指令，使用 Ackermann 模型、0.50 m 最小转弯半径和 2.8 秒预测。在 `nav2_real.yaml` 的 `FollowPath.tracking` 下调整 MPPI 参数。启动日志会打印 `Using nav2_mppi_controller::MPPIController for path tracking`。原地对正、末端停车和补位仍由底盘适配控制器处理。

实车 **Start Nav Through Poses** 会先预规划整条路线，在中间途经点附近识别接近折返的路径连接。去程与回程夹角达到 135° 时，当前段在该点结束，车停稳并对准出段方向后继续。当前段的终点由停车检查器确认，普通途经点仍可连续通过。出现 `Stopping at waypoint ... to align with the next segment` 表示识别到折返。

末端对准期间保持终点朝向，航向进入容差后输出零速。位置和航向用同一时刻的 TF 复核，轮式反馈低于停车阈值并持续满足条件 0.3 秒才报到点。停稳后位置持续超限才以最高 5 cm/s 补位，末端最多尝试两次修正。控制器日志记录补位和停稳验收。正常导航默认启用此流程，`goal_time_ab:=true` 仍使用两套原实验检查器。

补位时，若航向达标且沿车身轴线移动可进入内圈，控制器优先短距离直进或倒车。整段扫掠区域需通过局部代价地图和 0.35 秒内的原始点云检查。起步前覆盖不足时沿路径补位，直移途中失去净空依据时停车。当前单台前向 ODIN1 的原始点云无法直接确认车尾净空，车尾覆盖不足时会沿路径补位。

自旋定位处理可在实车启动命令中加 `odin_spin_guard:=true` 显式启用，默认关闭。它在底盘反馈确认原地自旋时用轮式里程计保持位姿连续，限制 ODIN1 的位置和航向修正，重新平移后逐步恢复融合。运行状态可在 `odin_localization` 日志中查看。

### 常见中断

- **缺少 `map` 或出现 TF 外推。** 分别查 `odom → base_link` 和 `map → odom`。ODIN1 的 `relocalization success` 后还要看到定位桥的 `Publishing map -> odom`。TF 恢复连续更新后再发新目标。静态 `map → odom` 入口只供停车联调。
- **无路径或拐弯贴近障碍。** 在 RViz 检查全局代价地图的目标和沿途通道，必要时分段点通道中央。要复查低速跟踪，可调低 `FollowPath.tracking.vx_max` 后重启，再对照 `/plan`、`/cmd_vel_nav` 和 **Collision Check Arc**。
- **膨胀区与禁行区。** 全局、局部软膨胀半径为 `0.55 / 0.50 m`。按现场测得最大外伸不超过 **28 cm**，两张代价地图统一使用 `robot_radius: 0.30`、`footprint_padding: 0.0`。Humble 静态地图复跑中，约 `(1.89, 0.00) m` 与 `(1.88, -0.02) m` 的两个旧禁行起点从代价 **253** 降至 **186**，均能生成路径。实车通行仍待复测，局部弧线碰撞需结合当时的实时点云复核。
- **单点和途经点遇阻。** 近段受阻或跟踪失败先让行 2 秒。只有远段失效时，若当前前方 0.60 m 路径仍然有效，可在 0.25 秒内换上方向接近且通过代价检查的新路径。其余情况停车重规划。连续受阻的基础期限为 20 秒，沿有效替代路线实际转向收敛时可获得有限额外时间（最多约 10.5 秒）。轮式里程计沿出发方向推进 12 cm 且跟踪持续至少 0.8 秒后才重置受阻计时。
- **`Failed to make progress`。** 当前 12 秒内需平移超过 0.08 m 或转动超过 0.12 rad。对照 `/cmd_vel_nav`、`/cmd_vel`、`/odom` 和 `/motion_mode_feedback`，核对控制器请求、限速后指令、底盘反馈和切换状态。 [Nav2 配置](../src/odin_nav_sim/config/nav2_real.yaml)

## 自旋定位处理实车 A/B（2026-11-21）

- **排查结果。** 左右各三次约 25° 的自旋中，地图车位偏移 2.7–4.0 cm，卷尺估测的车体中心位移在约 1 cm 以内。逐帧点云对照显示定位偏差主要在转动中累积。固定坐标偏移换起始朝向后将其中一组误差从 1.67 cm 放大到 3.53 cm RMS，因此采用按底盘自旋状态限制定位修正的方案。自旋复测（`log/site_spin_repeats_20261121_204529_lJtXFe/limited_spin/summary.log`） · 时间对照（`log/site_spin_timeline_20261121_215243_bDNInz/timeline.log`） · 换朝向验证（`log/site_raw_heading_holdout_20261121_214202_grrLXP/holdout_results.log`）
- **回放和旁路。** 回放中，整轮回正的位置误差从 5.51 cm 降到 0.40 cm。实车先用独立话题并行验证，两段自旋接直行的停车点云误差分别从 6.06 cm、6.41 cm 降到 4.70 cm、4.87 cm。离线回放（`log/site_spin_guard_replay_20261121_220201_IybJgq/closure_results.log`） · 旁路对照（`log/site_spin_shadow_20261121_221025_i2nSLV/pointcloud_comparison.log`）

`odin_nav_sim` 编译和 4 项定位回归测试通过。闭环测试分别设置 `odin_spin_guard:=false` 和 `odin_spin_guard:=true`。两次实车起点相差 0.61 cm、2.59°，使用点云配准将目标换算到同一物理位置。测试用自旋速度上限为 0.10 rad/s，平滑器线速度上限为 0.04 m/s。这组速度限制只用于本次试验。两组均完成首个导航目标，随后在下一个目标途中取消并停稳。

| 指标 | 关闭 | 开启 |
|---|---:|---:|
| 自旋中定位估计的最大平移 | 4.76 cm | 0.01 cm |
| 到点位置误差（点云配准） | 17.49 cm | 14.42 cm |
| 到点航向误差（点云配准） | 0.61° | 6.03° |
| 取消到轮速低于停车阈值 | 0.33 s | 0.26 s |
| 取消后额外行驶（轮式里程计） | 6.9 mm | 6.1 mm |

**结论。** 开启后自旋位姿摆动减小，本轮到点位置误差下降，同时末端航向误差增大。当前只有一对闭环样本，开关默认关闭。后续在同一路线交替复测，重点比较末端航向误差和位置重复性。测试包和日志保存在本机 `log/`。闭环点云对照（`log/site_spin_guard_ab_20261121_222003_rLvHg1/pointcloud_ab.log`） · 关闭组（`log/site_spin_guard_ab_20261121_222003_rLvHg1/off/tests.log`） · 开启组（`log/site_spin_guard_ab_20261121_222003_rLvHg1/on/tests.log`）

## Ranger Delta 导航闭环迭代（2026-11-22）

- 控制器复用 RPP 路径跟踪，在碰撞预测前应用底盘 0.50 m 最小行驶转弯半径。预览按可执行弧线采样。Nav2 跟踪与辅助动作统一通过速度平滑器，实车命令上限为 0.05 m/s、0.15 rad/s。仿真按同样的曲率规则限幅。
- 首轮同目标 2/3 成功。失败轮在末端反复切换行驶和自旋，舵轮切换期间底盘几乎停住，最后报进度超时。加入路径对准、跟踪、末端对准三阶段的切换回差后，同一短目标在同一定位会话内实车 3/3 成功，仿真 3/3 成功。实车 TF 估计相对原目标的停车误差为 2.7–4.2 cm、2.1–4.6°。 失败组（`log/site_nav_iteration_fZeE2g/real_RmGBWg/trials.log`） · 最终实车复测（`log/site_nav_iteration_fZeE2g/final_real_6k8rxze2/trials.log`） · 仿真复测（`log/site_nav_iteration_fZeE2g/final_sim_kvlhg932/trials.log`）
- 三趟弧线段的命令曲率峰值为 2.00 m⁻¹，底盘反馈约 1.99 m⁻¹。原始点云对照显示，后两趟停车点与第一趟相距 1.3 cm、4.4 cm。跨大角度的起终点配准两趟通过质量检查，估计距原目标约 7.9 cm、5.5 cm，另一趟重叠不足。 底盘和原始点云对照（`log/site_nav_iteration_fZeE2g/results_summary.log`）
- 最初停车位后方 42.5 cm 的近邻占用来自静态图，局部图最近实时占用约 97.5 cm。保存的 PCD 在对应栅格中含有离地约 0.74 m 的点，保留原占用。沿验证过的前向路线平移约 20 cm 后再开始导航。 代价地图对照（`log/site_nav_iteration_fZeE2g/site_obstacles.log`）
- 复核一次停车重启，原始点云间相差约 3 mm，地图车位却改变约 17 cm、3.7°。精确停车档仍要核实点云与场地地图的对齐，跨重启对照不能只看 TF。 停车重启对照（`log/site_nav_iteration_fZeE2g/restart_check.log`）
- 合成点云注入测试中，全局图的测试区域从 0 个致命占用格增至 4 个，局部图增至 6 个。收到穿过同一位置的背景回波后，两张图的新增占用均清除。实车搬动物体后的清除仍待验收。 障碍清除对照（`log/site_nav_iteration_fZeE2g/costmap_probe_result.log`）
- 同一实车绕障目标 `(1.30, -0.75) m` 两次成功。直线路径中心到致命占用最短约 38 cm，两次规划均绕行约 1.32 m，规划路径中心最小距离约 62 cm。点云显示两次起点相差约 1 cm、停车点相差约 9.4 cm。TF 给出的停车点间距约 0.5 cm，距原目标均约 8 cm。结束后被动检查 CAN，轮速全部为零。 绕障对照（`log/site_nav_iteration_fZeE2g/detour_summary.log`） · 停车检查（`log/site_nav_iteration_fZeE2g/final_stop.log`）

## ODIN1 停车重定位复验（2026-11-22）

- 在绕障终点附近保持车辆静止，仅重启 ODIN1 驱动五次。每次约 8.3–8.4 秒获得同步地图位姿和原始点云。约 −14° 朝向下，十组重启配对的最大地图车位差为 2.46 cm、0.27°，点云配准的最大位置差为 3.3 mm。单轮静止记录最长 21 秒，地图车位峰值变化不超过 1.6 mm。 现场复验（`log/odin_relocalization_repeat_FSKsQk/live_comparison.log`）
- 旧录包的六组停车重启对照覆盖约 −97°、−67°、−12° 和 −2° 四种朝向。最大地图车位差为 17.11 cm、3.71°，同组点云差为 2.7 mm、0.02°。其余五组地图位置差为 1.54–5.62 cm。旧样本的场地位置和运行过程也有变化，尚需在同一物理点改变朝向并重复启动，以验证朝向的影响。 旧录包复核（`log/odin_relocalization_repeat_FSKsQk/archived_restarts.log`）
- 对现场同一帧点云离线扰动地图位姿。沿地图 X 注入 ±17 cm 会触发项目现有 PCD 分数门槛，沿 Y 注入同样偏移仍可通过。该场景的参考图匹配对不同方向误差的敏感度不同，后续需用方向约束复核重定位。 参考图扰动对照（`log/odin_relocalization_repeat_FSKsQk/reference_sensitivity.log`）

### 地图 Y 方向的点云约束

- 从参考点云邻域估计平面法向，并用点到平面残差检查 X、Y 和航向的局部约束。现场停车位的最弱方向主要沿地图 Y。同一帧沿 Y 正向注入 17 cm 后，仍有约 92% 的点距参考点云不超过 5 cm。旧录包四个朝向的同类扰动也全部通过原有分数门槛，其中一个朝向的原始位姿未通过。 多朝向离线对照（`log/odin_direction_check_hRpVDR/geometry.log`）
- 尝试单独匹配法向主要沿 Y 的平面。五轮停车采集的 15 帧中，原始位姿下只有 0.7%–2.8% 的选中点在 5 cm 内找到同类地图点。当前场景中该筛选缺乏稳定对应，暂未接入运行。 定向平面筛选对照（`log/odin_direction_check_hRpVDR/directional_score.log`）

### 同一点换朝向实车复验

- 以同一旋转中心附近的原朝向、左转约 25°、右转约 25° 分别做三次停车重定位，9 次成功。三组内的最大地图位置差分别为 **3.62、2.82、3.88 cm**，原始点云组内最大位置差分别为 2.2、1.4、2.7 mm。回到原朝向再重定位也成功，点云估计车体距起始位置约 2 mm、朝向差 0.59°。 实车同点复测（`log/odin_direction_check_hRpVDR/field_comparison.log`）
- 从左转朝向回到原朝向的连续转向中，地图位姿估计平移 8.94 cm，原始点云配准估计平移 1.45 cm，两者平移向量相差 10.05 cm，航向变化仅差 0.03°。三个朝向的离线扰动都复现了地图 Y 正向偏移 17 cm 仍通过原始 PCD 门槛。停车重启在本场景内可重复，转动过程的平移估计仍有明显偏差。
- 默认实车启动实际触发了 Humble 参数解析错误，因为底盘节点直接读取带别名的 YAML。已移除别名并保持控制器与底盘均使用 0.50 m 最小行驶转弯半径。直接参数读取与默认启动下的实车转向均已通过。试验结束后被动接收 CAN，车体速度和末次复核的轮速均为零。 转向记录（`log/odin_direction_check_hRpVDR/field.log`） · 停车反馈（`log/odin_direction_check_hRpVDR/standstill.log`）

## 旋转平移偏差分解（2026-11-22）

- 同一会话录制 12 段左右自旋及两组约 30 cm 直行往返，按原始点云时间戳对齐两段 ODIN1 TF、轮式里程计和底盘模式。固定全局地图校正重算后，转向平移 RMS 仍为 **4.00 cm**，原始值为 **4.12 cm**。10 段全局校正增量不到 0.04 cm，两段分别为 1.86 cm、2.32 cm。典型回转在 1.03、2.10、2.79 秒出现 **1.70、3.59、4.52 cm** 的位置偏差，同时全局校正增量接近零。 时间对齐与来源分解（`log/odin_spin_source_GFsJZf/analysis.log`）
- 回放现有自旋保护后，12 段转向的相对平移 RMS 从 **4.11 cm 降到 2.05 cm**，航向 RMS 均为 0.13°。两组直行往返回位误差却分别从 1.18、0.19 cm 增至 6.97、2.40 cm。连续全程最终，两种回放都留下约 8.4 cm 的位置偏差。结束后停车重启，点云几乎重合，地图位置改变约 9.5 cm，并回到接近试验初始的对齐。 保护回放汇总（`log/odin_spin_source_GFsJZf/aggregate.log`） · 重启前后点云核对（`log/odin_spin_source_GFsJZf/startup_anchor_check.log`）
- 用原始点云估计车体自旋的真实平移。本地样本能拟合出约 4 cm 的旋转中心偏心，在旧录包的两个朝向上分别改善和恶化误差，现有证据不足以直接采用固定偏心补偿。 旋转中心留出对照（`log/odin_spin_source_GFsJZf/physical_pivot_check.log`）
- 短绕障现场 A/B 中，两组均先完成相同的约 24° 左转与回转，再导航到经点云换算的同一物理目标。两组均到达。关闭与开启保护时，TF 估计的到点位置误差分别为 7.7、8.2 cm；沿途点云分段配准给出的估计分别为 **13.7–14.0、20.8–21.5 cm**。两组停车点实测点云间距为 8.2 cm，TF 给出的间距为 3.1 cm。两组物理起点相差 4.1 cm、2.7°，这对样本保留起点差异的影响。结束后 CAN 反馈确认车体和各轮均停稳。 绕障现场对照（`log/odin_spin_source_GFsJZf/ab_results.log`） · 沿途点云配准（`log/odin_spin_source_GFsJZf/detour_chain_results.log`） · 停车反馈（`log/odin_spin_source_GFsJZf/standstill.log`）

## ODIN1 WIWC 接口与固定外参离线验证（2026-11-22）

- 回放限定为日志报告的驱动 **0.13.1**、SoC **V0.14.1**、SLAM **V0.13.0**、MCU **V1.5.2**，时钟模式为 2，TF 补发关闭。本机与上游 v0.13.1 的取逆方向一致。三个含 WIWC 的录包中，**2,762/2,762** 条可配对的 `imu → lidar` TF 可由 `twist.covariance` 前 16 项按行还原矩阵并取逆复现。TF 与标准里程计同戳，匹配的 WIWC 原始戳相对 TF 的中位提前量约 **97.5 ms**。按原始戳插值替换旧缓存后，车体 XY 的逐帧差在三包内分别为 **0.50、0.62、0.55 mm RMS**，最大 **8.95 mm**。固件内部如何生成这组时变矩阵仍需版本对应的定义。 回放日志（`log/odin_interface_replay/verified.log`） · [驱动 v0.13.1](https://github.com/manifoldsdk/odin_ros_driver/blob/v0.13.1/include/host_sdk_sample.h)
- 官方文档给出的固定 `imu → lidar` 矩阵在训练包将 12 段转向的平移 RMS 从 **4.11 降到 1.54 cm**，全程闭合误差从 **8.42 降到 2.76 cm**，两组约 30 cm 直行往返的闭合 RMS 从 **0.78 降到 0.55 cm**。各单程仍约 **4.6–5.1 cm**。直接冻结训练会话起始 WIWC 值时，自旋 RMS 为 **4.68 cm**。 [官方矩阵定义](https://github.com/ManifoldTechLtd/wiki/blob/master/docs/odin_series/odin1/5.%20Data%20output_.md) · 端点数据（`log/odin_interface_replay/endpoints.csv`）
- 外部会话中，官方固定矩阵在 A、B 朝向的自旋 RMS 分别由 **5.01 降到 1.94 cm**、**2.59 降到 2.10 cm**，另一会话由 **6.53 降到 2.97 cm**。B 朝向整组闭合从 **2.86 增至 3.90 cm**，三组外部直行单程均有小幅退化。轮式增量在训练会话闭合误差为 **0.25 cm**，但自旋模式的软件模型将平移设为零，训练自旋航向 RMS 为 **1.34°**。原始点云配准提供同传感器的几何参考。官方固定矩阵暂列为旁路候选，实车沿用原生链，`odin_spin_guard` 默认关闭。 复算脚本（`log/odin_interface_replay/replay.py`）

## 新 B 点卷尺停车对照（2026-11-22）

- 同一定位会话交替执行三轮短直行、三轮约 ±24° 自旋回正后短直行，**6/6 导航成功**。共用一个实体示教目标，重启后按车辆留在上一轮停车位的假设换算该目标。六轮收到的全局规划末点一致，距原指令目标约 **0.9 cm**，返程规划约 **17–22 cm**。卷尺实测沿目标车头方向提前 **4.1–7.4 cm** 停车。 六轮数据（`log/parking_pairs_EqthIh/six_trial_summary.json`）
- 直行组、转向组的实体停车位置最大两两间距分别为 **1.75 cm、1.94 cm**，相对各自示教方向的卷尺与 TF 平面差分别为 **0.98 cm、1.34 cm RMS**，六轮最大差 **1.53 cm**。当前短路线和样本量下，转向后的端点误差未出现一致增大。测点间距读数在约 **20.6–22.5 cm** 之间，横向和航向差需结合尺量误差解释。 逐轮卷尺与 TF 对照（`log/parking_pairs_EqthIh/six_trial_summary.json`）
- 第三轮直行实测纵向提前 **7.4 cm**，停稳后 TF 距路径末点 **7.48 cm**。在线读取到点容差为 **6 cm**。 [Nav2 1.1.20 控制器](https://raw.githubusercontent.com/ros-navigation/navigation2/1.1.20/nav2_controller/src/controller_server.cpp)

## Nav2 到点时间戳离线复现（2026-11-22）

- 用 TF2 的 10 秒缓存按录包接收顺序回放 **995 个采样时刻**。在实际报到点的 6 个时刻，现有查询路径均复现通过。改为按车体位姿的时间戳转换末点，5 轮仍通过，第 3 轮直行算得 **7.66 cm**，原查询路径为 **5.95 cm**。该轮末点转换用的 TF 比车体位姿早 **3.72 秒**。其余四轮在路径时间戳超出 TF 缓存后按 Humble 规则回退到最新 TF。 [复算脚本](../scripts/replay_parking_goal_checker.py) · 六轮到点快照（`log/parking_pairs_EqthIh/goal_checker_replay/reached_log_comparison.json`）
- 三种合成 TF 场景通过。用本机安装的 `SimpleGoalChecker` 实测离线构造的 **1,990 次**判定，与复算完全一致。录包重建以 rosbag 收件顺序和控制器日志为准，控制器未在现场记录内部查询结果。下一步在仿真中给到点判定增加只读记录，采样车体、末点、TF 时间戳和两套判定，再评估时间对齐改动。 逐时刻记录（`log/parking_pairs_EqthIh/goal_checker_replay/checker_cycles.csv`） · 本机插件复核（`log/parking_pairs_EqthIh/goal_checker_replay/native/result.log`）

## Nav2 到点时间对齐闭环仿真（2026-11-22）

- 用 Nav2 1.1.20 的独立诊断程序、项目 `DeltaPursuitController` 和模拟底盘，直接给 `FollowPath` 发送 22 cm 直线路径。两套判定共用 **6 cm** XY 容差。每个控制周期同时记录车体位姿、两个转换后末点、三个 TF 时间戳、两套距离及实际使用的判定。独立 DDS 域和仿真专用运动话题与实车隔离。 仿真配置（`log/goal_checker_sim_Bflixb/controller.yaml`） · 插桩（`log/goal_checker_sim_Bflixb/instrument.py`）
- 共 **12 轮**闭环且均成功。无修正时，两种方法都停在误差约 **5.98 cm** 处。注入 **−1.7 cm** 的纵向地图修正，原逻辑三轮均在真实仿真误差 **7.68 cm** 时报到，时间对齐三轮均走到约 **6.00 cm**。反向注入 **+1.7 cm** 后，两种方法都停在 **5.9 cm** 左右。原逻辑两轮均已停车但约 **4.9 秒** 后才报到，等待结束时原查询因规划时刻变换移出 TF 缓存而回退到最新 TF。时间对齐版在首次满足到点条件时返回成功。 自动验证（`log/goal_checker_sim_Bflixb/verification.log`） · 控制周期与停点汇总（`log/goal_checker_sim_Bflixb/verified_summary.json`）

## 可切换的 Nav2 到点时间对齐测试（2026-11-22）

- 用独立 overlay 编译 Nav2 Humble 1.1.20。仿真启动参数 `goal_time_mode` 默认 `stock`，`original` 和 `aligned` 共用 **6 cm** XY 容差、0.10 rad 航向容差和 0.04 m/s 速度。对齐版用当前车体 TF 的时间转换全局路径末点。该时刻缺少 TF 时，本周期暂不判到点。两组日志均记录两套距离及实际判定，方便复查同一周期。当前验证范围为模拟底盘和可控的纵向地图修正。 [构建脚本](../scripts/build_goal_time_test.sh) · [控制器改动](../scripts/nav2_goal_time_test/controller.patch)
- 完整 Nav2 `NavigateToPose` 在默认栅格图上跑短直行。**8/8 对照成功**，每轮仅规划一次。无修正时两组距路径末点均约 **5.78 cm**。注入 **−1.7 cm** 修正时，原版两次停在 **7.56–7.58 cm**，对齐版两次停在 **5.86–6.00 cm**。注入 **+1.7 cm** 修正时，两组均停在 **5.9 cm** 左右，原版停后约 **6.53 s** 才报到，对齐版约 **0.01 s**。另测默认仿真导航成功，未加载 overlay 时测试模式会在启动节点前报错。实车停车精度仍待用 B 点尺量复验。 逐轮结果（`log/goal_time_e2e_B4ZqbL/verified_summary.json`）

在项目根目录执行。测试切组时先退出上一组，再用相同起点和目标运行另一组。下例的 RViz 起点为 `(0.05, 0.05)`，目标为 `(0.25, 0.05)`，车头沿地图 +X。测试 overlay 在单独的仿真终端加载。

```bash
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select odin_nav_sim
./scripts/build_goal_time_test.sh
source install/local_setup.bash
source build/nav2_goal_time_test/install/local_setup.bash
export ROS_DOMAIN_ID=136 ROS_LOCALHOST_ONLY=1
ros2 launch odin_nav_sim navigation_sim.launch.py \
  initial_x:=0.05 initial_y:=0.05 \
  goal_time_mode:=aligned goal_time_shift_x:=-0.017
```

## B 点实车到点判定对照预检（2026-11-22）

- 实车启动增加可选的 `goal_time_ab:=true`，需要独立 overlay、`goal_profile:=precision` 和 ODIN1 重定位。两套目标检查器使用同样的 6 cm、0.10 rad 容差。实验目标的行为树显式选择检查器，切组可以共用一次定位会话。独立仿真已按目标验证两种选择，注入 −1.7 cm 修正时分别停在 **7.57 cm**、**5.87 cm**。 原判定（`log/parking_goal_time_ab_pjuDwu/sim_original/result.json`） · 对齐判定（`log/parking_goal_time_ab_pjuDwu/sim_aligned/result.json`）
- 实车两种检查器均成功加载，底盘反馈正常，ODIN1 重定位成功。启动后的停车位读数比上次尺量对应的停车记录相差约 **31 cm、10°**，旧 B 的实体基准需现场复核。只读预规划发现当前车位落在全局代价 **253** 的栅格，离最近静态占用格约 **35 cm**，三个倒车候选均未通过。未发运动目标，预检已停止并保存录包。9,462 条底盘反馈均为零速。 起点定位（`log/parking_goal_time_ab_pjuDwu/anchor_now.json`） · 障碍来源（`log/parking_goal_time_ab_pjuDwu/obstacle_source.json`） · 停车核对（`log/parking_goal_time_ab_pjuDwu/standstill.json`）
- 新尺量确认车相对上轮停车位实际后移约 **36 cm**、转了约 **10°**。同一个实体 B 点现在约在车头前方 **41 cm**。用尺量和当前 TF 换算后，B 的地图坐标与旧会话相差 **8.65 cm**。从当前位姿直接规划仍被静态膨胀区阻止。把起点沿车头虚拟前移 **16 或 18 cm** 时，规划成功，剩余约 **28–29 cm**，全局路线距最近占用格约 **49 cm**。预检已结束并保存录包，15,165 条底盘反馈均为零速。等待人工直前并停稳后复查实际路线。 目标换算（`log/parking_goal_time_forward_xkEOg6/goal_for_session.json`） · 虚拟起点对照（`log/parking_goal_time_forward_xkEOg6/virtual_stages.json`） · 停车核对（`log/parking_goal_time_forward_xkEOg6/standstill.json`）

### 前移后的原判定首轮

- 停车点云确认人工直前 **16.54 cm**，左右偏移约 0.74 cm。用前移前后的点云把同一实体 B 换算到新定位会话。检查通过的前进规划长 **28.18 cm**，全局和局部路线到最近占用格分别为 **50.2 cm、69.2 cm**。原判定实车成功，观测到的峰值线速度指令 **0.03 m/s**，全程无倒车。 目标换算（`log/parking_goal_time_b_Zig9Ij/anchor_registration.json`） · 首轮结果（`log/parking_goal_time_b_Zig9Ij/original-1_stopped.json`）
- 同一真实控制周期并排计算两种结果。对齐判定首次满足 6 cm 及朝向条件在 **6.089 s**，当时原判定给出 **9.62 cm**，其变换比车体位姿早约 **6.04 s**。原判定在 **10.039 s** 报到。停稳时 TF 距指令目标约 **5.88 cm**，点云配准估计沿目标朝向提前 **9.10 cm**，实体偏差待卷尺复核。首个对齐判定通过的记录来自原版同一路径运行，尚未发对齐版实车目标。 同周期对照（`log/parking_goal_time_b_Zig9Ij/original-1_timing.json`） · 停车点云（`log/parking_goal_time_b_Zig9Ij/original-1_cloud_pose.json`）

### 时间对齐实车首轮及初步对照

- 原判定首轮卷尺读数为 `R_x=58.0、R_y=−9.8、F_x=79.0、F_y=−13.5 cm`。相对实体 B 约提前 **3.52 cm**、横偏 **−3.62 cm**，二维距离 **5.05 cm**。卷尺与从试验起点累积的点云目标误差估计存在差异。 首轮尺量（`log/parking_goal_time_b_Zig9Ij/original-1_comparison.json`）
- 两轮之间会话中断，重启用停车点云转接原 B。底盘按现场许可倒回约 **16.65 cm**。时间对齐首轮前进导航成功，发目标后约 **5.99 s** 报到，实际采用的到点距离为 **5.90 cm**。同一周期原查询距末点 **6.97 cm**，用的是约 **5.96 s** 前的变换。 对齐轮结果（`log/parking_goal_time_b_Zig9Ij/aligned-1_stopped.json`）
- 直接配准两轮停车点云，两处实体停点相距约 **0.63 cm**、航向差约 **5.74°**。两轮起点相差约 **2.75 cm、11.05°**。各轮路径末点相对实体 B 换算值的偏移也不同，暂用同周期的判定差异判断时间对齐是否生效。 两轮点云直接配准（`log/parking_goal_time_b_Zig9Ij/pair_cloud_comparison.json`）
- 对齐轮卷尺读数为 `R_x=55.7、R_y=−8.3、F_x=75.3、F_y=−14.5 cm`，相对 B 约提前 **6.51 cm**、横偏 **−4.00 cm**，二维距离 **7.64 cm**，航向差 **−5.49°**。两轮中点按卷尺相距 **3.01 cm**，按点云相距 **0.63 cm**。同一对车身标记按尺量得到的间距在示教和两轮测试中分别为 **22.50、21.32、20.56 cm**。本次实车确认了时间对齐对报到判定的影响，实体停车精度仍需在同一定位会话和相近起点、朝向下重复对照。 两轮尺量汇总（`log/parking_goal_time_b_Zig9Ij/paired_tape_summary.json`）

### 同一定位会话的 A/B 预检

- 对照程序记录指令目标、规划末点、两套判定首次通过和底盘停稳。用上轮录包验证，原判定在发目标后 **9.47 s** 停稳、**10.04 s** 报到，对齐轮在 **6.26 s** 停稳、**5.99 s** 报到。停稳以连续底盘反馈判定，线速度及角速度门槛为 **0.005 m/s、0.02 rad/s**。
- `enable_control:=false` 时用点云核对当前停车位与上一轮相差约 **0.26 cm**。虚拟倒退 **16–18 cm** 的两条返回路线均通过规划，最近占用格距规划路径中心约 **49–50 cm**。实际对照准备在同一定位会话里交替测试，并用轮式里程计和停车点云核对起点。等待静态尺量读数及现场确认。 停车点云核对（`log/parking_fixed_session_tol2U6/restart_registration.json`） · 虚拟路线（`log/parking_fixed_session_tol2U6/static_route_preview.json`）
- 收到当前停车位的静止复测 `R_x=55.6、R_y=−8.3、F_x=75.5、F_y=−14.5 cm`。与前一组相比，单项最大变化 **0.2 cm**，标记中点变化 **0.05 cm**，计算航向变化约 **0.25°**。现场确认 R–F 中点与底盘外壳平面中心重合，换算时按平面偏移 **(0, 0) cm** 处理。两组坐标推得的标记间距为 **20.56、20.84 cm**。 静止复测数据（`log/parking_fixed_session_tol2U6/tape_calibration.json`）
- 试验启动时清理了静态预检遗留的 ODIN1 进程，重启后完成原判定第 1 轮。轮式反馈后退 **17.67 cm**，前进规划长 **24.62 cm**，规划末点距指令目标 **2.10 cm**。原判定和时间对齐首次满足条件分别在 **9.94 s** 和 **7.04 s**，本轮采用原判定，**9.94 s** 报到，约 **10.03 s** 停稳。停车 TF 距计划末点 **5.99 cm**，距指令目标 **7.71 cm**。本轮结束后定位进程退出，录包已补建索引。停车尺量用于这轮记录，下一对试验从新会话内重新交替测量。 轮式及停车数据（`log/parking_fixed_session_tol2U6/original-1_stopped.json`） · 判定时间对照（`log/parking_fixed_session_tol2U6/original-1_timing.json`）
- 该轮尺量 `R_x=54.5、R_y=−10.2、F_x=76.0、F_y=−15.0 cm`，相对 B **提前 6.50 cm、横偏 −5.22 cm**，二维距离 **8.34 cm**。本次实车同周期结果可以用于复核到点时间问题，实体停车位置作为单独样本。 首轮尺量（`log/parking_fixed_session_tol2U6/original-1_comparison.json`）
- 第二次尝试以临时用户服务启动定位，原判定轮成功。两套判定在同一次运行中首次满足条件分别为 **10.29 s、7.59 s**，底盘约 **10.60 s** 停稳。尺量距实体 B **3.89 cm**，主要来自横偏。定位服务在停车后再次退出，录包已补建索引，本轮继续作为单独样本。下一对改为一次连续运行完成，两轮均自动保存停车点云，最后一次停车再用卷尺复核。 原判定轮记录（`log/parking_matched_session_iffokf/original-1_timing.json`） · 停车尺量（`log/parking_matched_session_iffokf/original-1_comparison.json`）

### 同会话连续 A/B 第 1 对

- 从三次旧录包日志确认，会话中断源于在录包写入期间读取 SQLite 数据库，录包进程报 `database is locked`，随后监督进程关闭导航。改为两轮结束、录包正常关闭后统一读取。单次负向底盘反馈触发的中止也已定位，回放未见倒车指令，后续继续拦截反向指令，并按实际后移累计距离监控。 录包锁冲突（`log/parking_paired_continuous_YlEDfR/active/record.log`）
- 一次 ODIN1 重定位完成两轮，两版的指令目标和规划末点一致。起点点云配准相差 **0.86 cm、1.68°**。原判定 **10.79 s** 报到、**11.50 s** 停稳，对齐版 **7.19 s** 报到、**7.47 s** 停稳。对齐版报到时，用旧变换计算距末点 **6.77 cm**，时间对齐后为 **6.00 cm**。 原判定日志（`log/parking_ab_final_i7GQ3A/original-1_timing.json`） · 对齐版日志（`log/parking_ab_final_i7GQ3A/aligned-1_timing.json`）
- 两轮停车点云直接比较，标记中点相差约 **1.14 cm**，航向差约 **4.48°**。对齐轮尺量为 `R_x=61.0、R_y=−10.5、F_x=82.6、F_y=−12.8 cm`，距实体 B **2.94 cm**，其中纵向提前 **0.30 cm**、横偏 **−2.93 cm**，航向差 **+5.98°**。同期 TF 给出的纵向提前量为 **5.23 cm**，仍需复核实体 B 到地图坐标的换算和停车位姿。连续对照的原判定轮只采了点云，实体距离以对齐轮的尺量为准。录包正常关闭，最后 40 条底盘反馈均为零速。 对照汇总（`log/parking_ab_final_i7GQ3A/pair-1_comparison.json`） · 停车尺量（`log/parking_ab_final_i7GQ3A/aligned-1_comparison.json`）
