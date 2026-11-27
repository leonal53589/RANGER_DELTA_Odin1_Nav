# A 点返航精度复验

## 2026-11-25 五轮结果修订

地面两条固定基准线定义坐标，+X 向前，面朝 +X 时 +Y 向左。车身上 R 靠后、F 靠前，返回同一个实体 A 时始终沿原胶带读数。A 的 R 为 (22.8, −4.1) cm，F 为 (43.5, −8.0) cm，中点为 (33.15, −6.05) cm，R→F 方位为 −10.67°，标距为 21.06 cm。胶带坐标和 ROS 的 map 坐标各有自己的原点和方向。

| 轮次 | 路线 | 尺量中点误差 cm | 尺量航向变化 ° | Nav2 停车 TF 误差 cm | ODIN 原生链换算底盘距出发 A 约 cm |
|---|---|---:|---:|---:|---:|
| 1 | 右 | 25.58 | +10.14 | 9.29 | 23.8 |
| 2 | 右 | 26.25 | +9.61 | 9.41 | 24.6 |
| 3 | 左 | 4.50 | −10.37 | 5.86 | 4.5 |
| 4 | 左 | 18.13（标距异常） | +36.32（标距异常） | 3.66 | 7.6 |
| 5 | 左 | 6.70 | −11.65 | 4.23 | 7.8 |

现场已确认第 3、5 轮的 F_y 分别为 −14.3、−16.3 cm。第 4 轮保留原读数，按其计算标距为 16.86 cm，单列为量测异常。五轮返航 action 全部成功。按单轮距离 ≤10 cm 且航向绝对误差 ≤10° 的试验标准，停车 TF 为 2/5，卷尺四轮可评样本为 0/4。其中两轮实体位置达标；21 cm 标距下，几毫米的横向读数变化即可影响航向临界判定。

## 交给实车测试智能体的任务

请在工作空间 `/home/leona/WorkingSpace/navigation2/project_slam` 进行 ODIN 自旋位置保护开关对照试验。检验右侧返航时，保护是否让提供给 Nav2 的停车位置比实体停点更靠近目标。右侧历史两轮实体误差约 26 cm，停车时原生底盘位置与导航 TF 相距约 15 cm。本轮以卷尺为独立判据。

### 1. 锁定起止条件

- 沿用同一张 11 月 24 日地图、同一实体 A 和原胶带。机器人若已经离开 A，沿用原目标，记录实际起步位置即可。若胶带或标记已移动，请现场人员重新建立一次实体 A，停车量两遍，同时记录两秒稳定的地图位姿，再为整批试验锁定新目标。
- 去程先到右侧离位点 D，地图位姿为 (1.00 m, −1.10 m, 0°)。返航目标沿用 A_nav = (−0.2020 m, −0.1520 m, −0.0431 rad)。每轮从实际停稳的 D 预规划到固定的 A。记录 D 的实际位姿、最后接近 A 的方向和最后一段 accepted_plan。若换边或离位点差异明显，保留该轮并单列。
- 先做三对，共六次返航，开关顺序为 ON/OFF、OFF/ON、ON/OFF。每轮用新目录；每轮按同一流程重启导航并等重定位稳定，即使相邻两轮取值相同也如此。换会话时记录车辆未动期间重启前后的地图位姿差；若与旧 A 所在的地图明显错位，暂停该轮校验定位。操作者确认现场可试车、可急停后，才发送首个目标。

### 2. 准备目录和实车启动

下面各段是独立步骤。终端 1 在项目目录设置本轮标签，创建目录后启动导航。

```bash
cd /home/leona/WorkingSpace/navigation2/project_slam
source install/setup.bash
export ROS_DOMAIN_ID=44 ROS_LOCALHOST_ONLY=1 PYTHONNOUSERSITE=1
MAP_DIR="$PWD/maps/odin_20261124_192624_a0FBCc"
GUARD=true                  # OFF 轮改成 false
ROUND=01                    # 每轮更新
mkdir -p "$PWD/log"
RUN_DIR=$(mktemp -d "$PWD/log/arrival_r${ROUND}_guard_${GUARD}_XXXXXX")
echo "$RUN_DIR"
ip -details link show can0
ros2 launch odin_nav_sim real_navigation.launch.py \
  map:="$MAP_DIR/site_nav.yaml" \
  odin_relocalization_config:="$MAP_DIR/odin_relocalization.yaml" \
  sensor_x:=0.210 sensor_y:=0.0 sensor_z:=1.125 \
  sensor_roll:=0.0 sensor_pitch:=0.0 sensor_yaw:=0.0 \
  can_interface:=can0 enable_control:=true launch_rviz:=true \
  goal_profile:=navigation goal_time_ab:=false odin_spin_guard:="$GUARD" \
  |& tee "$RUN_DIR/launch.log"
```

检查 can0 已 UP 且为 500000 bit/s；若未启用，先处理 CAN 再启动。只运行一套实车 launch。保留当前速度、容差、碰撞检查和地图参数。spin guard 是定位节点启动时的只读参数。每轮用 `ros2 param get /odin_localization odin_spin_guard` 核对实际值，并记录 `ros2 param get /odin_localization odin_reference_pcd` 的结果，所有轮次一致。

### 3. 先录包，后发送两个独立目标

在终端 2、3 都进入同一工作空间，source 和 export 同终端 1。终端 2 将 RUN_DIR 设为终端 1 打印的完整路径，在发目标前启动录制。

```bash
ros2 bag record --include-hidden-topics -o "${RUN_DIR:?请先设置本轮目录}/bag" \
  /tf /tf_static /odin1/tf /odin1/tf_static /odin1/odometry \
  /odom /motion_mode_feedback /chassis_motion_feedback \
  /plan /controller_server/accepted_plan /controller_server/tracking_endpoint \
  /cmd_vel /cmd_vel_guarded /navigation_velocity_guard/state \
  /navigate_to_pose/_action/status /follow_path/_action/status
```

终端 3 核对 `map → base_link` 连续更新、车头与墙线和现场一致，轮速为零，Nav2 已 active。在该终端定义两个目标和预规划函数。`use_start: false` 让规划器使用车辆此刻的实际位置。

```bash
D_POSE='{header: {frame_id: map}, pose: {position: {x: 1.0, y: -1.10}, orientation: {w: 1.0}}}'
A_POSE='{header: {frame_id: map}, pose: {position: {x: -0.2020, y: -0.1520}, orientation: {z: -0.0215483, w: 0.9997678}}}'
plan_to() {
  ros2 action send_goal /compute_path_to_pose nav2_msgs/action/ComputePathToPose \
    "{goal: $1, planner_id: GridBased, use_start: false}"
}
plan_to "$D_POSE"
```

确认规划成功、返回路径非空且走廊与现场相符，再运行下段发 D。

```bash
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: $D_POSE}"
```

等待 D 成功、底盘停稳，执行 `plan_to "$A_POSE"` 从实际 D 预检到 A。确认规划成功且路径非空，随后单独运行返航命令。

```bash
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: $A_POSE}"
```

每轮使用新的录包目录，录包持续到返航停稳后的量测窗口结束。若程序自动发目标，检查预规划确实有路径、action 结果确实为 SUCCEEDED；超时应取消目标并等取消确认。失败、急停、定位跳变或路线有人时，先结束本轮并确认停车，保留日志和失败原因。

### 4. 停车量测

返航 action 结束后，先确认无活动目标，检查新鲜轮速的平面线速度和角速度都已持续接近零，保持录包至少 5 秒，记录这一段的统一停车时间窗，再停止录包。确认静止才请现场人员接近。每轮向现场人员发：

> 请在原胶带上量相同标记。按 R_x、R_y、F_x、F_y 顺序报带符号的厘米读数，重新放尺量两遍；读完退出路线后回复「已离开」。下一轮等你确认后开始。

用两点中点与 A 中点的距离作位置误差。用 atan2(F_y−R_y, F_x−R_x) 减 A 的 −10.67° 并回绕到 ±180° 作带符号航向误差。保留两遍原数；若标距与现场直接量的 R–F 中心距不符，趁车还停着复量。将距离、航向、两项同时达标分别记列。

### 5. 交付数据和判断

- 每轮记录开关实值、实际 D、进近方向、返航状态、耗时、是否经过恢复、原目标、最后实际跟踪末点、停车导航 TF、ODIN 换算的停车底盘位姿、四个卷尺读数和 bag 路径。失败轮进入导航成功率的分母。结束后用 ros2 bag info 确认所需动态话题有消息。
- 在同一停车时间窗取多个带时间戳的样本。原生底盘链按 `T_map_odinMap × T_odinMap_lidar × inverse(T_base_lidar)` 计算，其中设备 TF 来自 /odin1/tf，安装外参来自 /tf_static。先做三维变换再取 x、y、yaw。/odin1/odometry 的子系是 imu。导航底盘位姿来自公共 /tf 的 `map → odom → base_link`。注意定位桥对 map→odom 的时间戳有 0.30 s 前置；分析运动中分叉时按源时间匹配。停稳后报中位数和波动范围。
- 每对至少给出三个量：实体距 A、导航 TF 与原生底盘的平面间距、实体航向误差。分别列 ON、OFF 及配对差，同时检查原地转向期间 OFF 是否伴随较大平移估计。保留接近方向和重定位初始偏差。若三对中实体和软件间距均一致改善，且收益超过重复放尺波动，再换左侧路线复验。若只改善软件间距，继续检查外参、地图配准和停车逻辑。向我报告证据后再决定日常导航默认值。
