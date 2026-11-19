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

- **松灵依赖已纳入工作空间。** `agx_bringup` 位于 `src/vendor/ranger_ros2`，由 [dependencies.repos](../dependencies.repos) 固定在已验证的 `air_delta` 提交。官方启动配置订阅 `/cmd_vel`，轮式里程计发布到 `/agx/wheel_odom_raw`。 [官方 ROS 2 驱动](https://github.com/agilexrobotics/ranger_ros2/tree/air_delta)
- **实车收帧已通过。** `can0` 在 `500000 bit/s` 下连续收到状态、运动和模式反馈。4 秒 `candump` 抓到 3,418 帧，发送帧数为 0。
- **阶段 1 已实现。** `odin_nav_sim` 用 Humble Nav2、测试栅格地图和模拟底盘跑 `/navigate_to_pose`，规划器为 `SmacPlanner2D`，控制器为 `RegulatedPurePursuit`。模拟底盘接收 `/cmd_vel`，发布 `/odom` 和 `odom → base_link`，固定 `map → odom`。
- **雷达安装前可接底盘。** 接 USB 转 CAN，确认前进、转弯、自旋、横移和超时停车。核对 `/agx/wheel_odom_raw`、`/motion_mode_feedback` 和 `odom → base_link`。该版本会广播底盘 TF，且里程计速度未填 `linear.y`。先验证前进、转弯和自旋，再评估全向控制。 [驱动里程计源码](https://github.com/agilexrobotics/ranger_ros2/blob/fb96124fa68c49bab03d8ddd2294c2afd45af925/agx_bringup/include/global_state.hpp#L499-L588)
- **安装后接定位和避障。** 标定 `base_link → imu`，接真实地图、实时点云和 `map → odom`。确认地图、车体和点云对齐后，低速验证首个目标点。`/odom` 及每段 TF 都由单一来源发布。

### 阶段 2 底盘反馈回放

`ranger_delta_bringup` 使用松灵 `agx_bringup` 的消息类型。节点发布 `/system_status`、`/chassis_motion_feedback`、`/motion_mode_feedback`、`/agx/wheel_odom_raw` 和可选的 `odom → base_link`。默认回放 `candump -L` 日志，在本机 ROS 域 `117` 运行。回放只读文件，实车反馈模式也仅接收 CAN 数据。位姿是根据底盘反馈积分得到的估计，行驶方向和尺度仍待实车验收。

在项目根目录编译底盘包及其依赖。新检出工作空间时先按 [README](../README.md#三编译工作空间) 导入松灵源码。回放参数填写实际保存的 `candump -L` 文件路径。

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
- **按用户要求完成代码回退**：将控制逻辑 [`commands.py`](file:///home/leona/WorkingSpace/navigation2/project_slam/src/ranger_delta_bringup/ranger_delta_bringup/commands.py) 和 [`command_node.py`](file:///home/leona/WorkingSpace/navigation2/project_slam/src/ranger_delta_bringup/ranger_delta_bringup/command_node.py) 完全回退至初始稳定版本，保持原有控制流不变。

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
   - [`control_ui.py`](file:///home/leona/WorkingSpace/navigation2/project_slam/src/ranger_delta_bringup/ranger_delta_bringup/control_ui.py) 集成 4 轮状态看板，并提供“⏺ 开始录制轮子数据 / ⏹ 停止录制并生成诊断”一键操作按钮。

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
- 在 [`commands.py`](file:///home/leona/WorkingSpace/navigation2/project_slam/src/ranger_delta_bringup/ranger_delta_bringup/commands.py) 中重构 Crab 横移模式运动学映射：
  - 纯向右横移 ($v_y < 0$): 舵角设为 `-pi/2` (CAN `-1571`，轮子转到 `+90°`)，速度下发正值。
  - 纯向左横移 ($v_y > 0$): 舵角同样设为 `-pi/2` (CAN `-1571`，轮子转到 `+90°`)，速度下发负值（反向驱动）。
- 彻底杜绝向左横移时向舵轮下发 `+1571` 撞击 `-30°` 极限导致的斜向窜动，实现向左向右均保持横向纯平移。
