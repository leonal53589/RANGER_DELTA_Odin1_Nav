# Ranger Delta 旋转中心与 ODIN1 6-DoF 外参实车标定记录

**实测日期**：2026-11-26  
**本机归档** `local_experiments/field_archive/calib_20261126_165306_n26ilO/`，含原始录包与 `calibration_result.json`。

**采集与求解脚本**：[`calib_runner.py`](../scripts/calib/calib_runner.py) · [`solve_calibration.py`](../scripts/calib/solve_calibration.py)

---

## 1. 标定目标与坐标系定义

为使全图任意位置点云配准、轮式自旋与实地卷尺验收在同一物理刚体坐标系下自洽，本次实验联合标定两项几何量：

1. **底盘真实物理旋转中心（`base_link` 地面投影原点）**
   - 在 [`ranger_delta_description.urdf`](../src/vendor/agilex_models/ugv_gazebo_sim/ranger_delta/ranger_delta_description/urdf/ranger_delta_description.urdf) 中，双轮转向轴位于 $x=+0.194\,\text{m},\ y=\pm 0.169\,\text{m}$，单轮转向轴位于 $x=-0.1835\,\text{m},\ y=0$。
   - 实车受轮胎形变、三轮负载与舵角零偏影响，原地 `Spin` 时的物理瞬时旋转中心不一定与 URDF 几何中心或车身侧面胶带 `R/F` 中点重合。本标定直接以**原地 `Spin` 的实测刚体圆心**作为真实 `base_link` 地面投影。
2. **底盘至雷达 6-DoF 刚体外参（`base_link -> lidar`）**
   - 采用 ROS URDF 标准外旋 `XYZ` / 内旋 `ZYX` 欧拉角约定：
     $$T_{\text{base}\leftarrow\text{lidar}} = \begin{bmatrix} R_z(\text{yaw}) R_y(\text{pitch}) R_x(\text{roll}) & [x,\ y,\ z]^T \\ 0 & 1 \end{bmatrix}$$
   - 将实时点云 $\mathbf{p}_L$ 变换至底盘地面坐标系的公式为 $\mathbf{p}_B = R_{BL}\mathbf{p}_L + [x, y, z]^T$。

---

## 2. 标定理论与免靶标流水线

无需相机或棋盘格标定板，仅利用实验室水平地面、四周墙体及底盘可控运动（经由 `/cmd_vel_nav → velocity_smoother → navigation_velocity_guard → 底盘` 链路执行）：

1. **地面平面拟合 $\to$ $(\text{roll},\ \text{pitch},\ z)$**
   - 在多个停点采集静止 `/odin1/cloud_raw`（10 帧叠加，体素 $2\,\text{cm}$ 降采样），选取地面区域 $x \in [0.35, 4.0]\,\text{m},\ |y|<2.2\,\text{m}$，用 Huber/soft-L1 鲁棒最小二乘拟合地面 $z = ax + by + c$，得地面法向量 $\mathbf{n}_L = [-a, -b, 1]^T / \|\cdot\|$ 与雷达离地垂直高度 $z = -c \cdot n_{L,z}$。
   - 令 $R_y(\text{pitch})R_x(\text{roll})\mathbf{n}_L = [0, 0, 1]^T$，解得：
     $$\text{roll} = \operatorname{atan2}(n_{L,y}, n_{L,z}),\quad \text{pitch} = \operatorname{atan2}(-n_{L,x}, \sqrt{n_{L,y}^2 + n_{L,z}^2})$$
     （因 $R_z(\text{yaw})$ 绕 $Z$ 轴旋转，不改变地面法向量，因此 `roll` 与 `pitch` 可先于 `yaw` 精确解耦求解。）
2. **多角度原地自旋 $\to$ 物理圆心与传感器水平偏置 $\mathbf{r}_L = [r_{Lx}, r_{Ly}]^T$**
   - 依次执行 `0° → +90° → -90° → 0°` 原地慢速自旋（`0.16 rad/s`），记录连续 `/odin1/cloud_raw` 与 4 个静止停点，并由人工在每个停点沿固定地面轴测量 `R_x, R_y, F_x, F_y`。
   - 对点云先按 `(roll, pitch, z)` 整平并截取垂直结构（$z \in [0.18, 2.10]\,\text{m}$），通过连续关键帧链接与全局局部地图多轮 Point-to-Plane ICP 对齐，求解整平坐标系下的雷达轨迹 $(X_{L,k}, Y_{L,k}, \Psi_{L,k})$。
   - 按刚体自旋模型（分每段独立圆心消除 360° 累计轮胎搓动，并用 `soft_l1` 抑制通道远端特征稀疏段外点）：
     $$\begin{bmatrix} X_{L,k} \\ Y_{L,k} \end{bmatrix} = \mathbf{c}_{W,\text{seg}} + R(\Psi_{L,k}) \begin{bmatrix} r_{Lx} \\ r_{Ly} \end{bmatrix}$$
   - 同步由 4 组卷尺 `R/F` 读数按 $M_k = \mathbf{c}_{\text{ground}} + R(\theta_k)\mathbf{r}_{RF}$ 拟合出 `R/F` 中点与物理自旋圆心的固定偏置。
3. **开阔通道纯直行 $\to$ `sensor_yaw` 与底盘直线航向校准**
   - 转向开阔通道（前方 $>5.5\,\text{m}$ 净空），纯直行命令 `vx=0.15 m/s, wz=0` 行驶约 $1.8\,\text{m}$（全程录包）。
   - 用 Point-to-Plane ICP 对起终点点云配准得到雷达位移 $\Delta \mathbf{p}_L$ 及实际航向漂移 $\Delta\theta_{\text{drv}}$，扣除自旋杠杆臂影响得到真实旋转中心的移动向量 $\Delta \mathbf{c}_L$。
   - 由弧弦几何（并用 400 Hz `/odin1/imu` 去偏置积分对 $\Delta\theta_{\text{drv}}$ 交叉校验），底盘正前方 $+X$ 在雷达系中的方位为 $\phi = \operatorname{atan2}(\Delta c_{L,y}, \Delta c_{L,x}) - \Delta\theta_{\text{drv}}/2$，故 **`sensor_yaw` $= -\phi$**，并把 $\mathbf{r}_L$ 旋回 `base_link` 系得：
     $$\begin{bmatrix} \text{sensor\_x} \\ \text{sensor\_y} \end{bmatrix} = R_z(\text{sensor\_yaw}) \begin{bmatrix} r_{Lx} \\ r_{Ly} \end{bmatrix}$$

---

## 3. 复现操作步骤

启动标准实车节点（仅提供驱动、`/odom`、速度平滑与安全守护，不发送 Nav2 目标）：

```bash
cd /home/leona/WorkingSpace/navigation2/project_slam
./scripts/run_real_navigation.sh
```

在另一终端加载环境并按安全确认节奏采集数据（每次运动前必须确认人员退出行车范围）：

```bash
cd /home/leona/WorkingSpace/navigation2/project_slam
source install/setup.bash
export ROS_DOMAIN_ID=44 ROS_LOCALHOST_ONLY=1 PYTHONNOUSERSITE=1
CAL_DIR=$(mktemp -d "$PWD/log/calib_$(date +%Y%m%d_%H%M%S)_XXXXXX")

# 阶段一：原地自旋（每站停稳后人工量 R_x, R_y, F_x, F_y）
python3 scripts/calib/calib_runner.py capture --out "$CAL_DIR/stop_0.npz" --frames 10
python3 scripts/calib/calib_runner.py spin --angle-deg 90.0  --speed 0.16 --bag "$CAL_DIR/spin_0_to_p90/bag"
python3 scripts/calib/calib_runner.py capture --out "$CAL_DIR/stop_1_p90.npz" --frames 10
python3 scripts/calib/calib_runner.py spin --angle-deg -180.0 --speed 0.16 --bag "$CAL_DIR/spin_p90_to_m90/bag"
python3 scripts/calib/calib_runner.py capture --out "$CAL_DIR/stop_2_m90.npz" --frames 10
python3 scripts/calib/calib_runner.py spin --angle-deg 90.0  --speed 0.16 --bag "$CAL_DIR/spin_m90_to_0/bag"
python3 scripts/calib/calib_runner.py capture --out "$CAL_DIR/stop_3_0.npz" --frames 10

# 阶段二：面向开阔通道纯直行 1.8 m
python3 scripts/calib/calib_runner.py spin --angle-deg -90.0 --speed 0.16 --bag "$CAL_DIR/spin_stage2_m90/bag"
python3 scripts/calib/calib_runner.py capture --out "$CAL_DIR/drive_start.npz" --frames 10
python3 scripts/calib/calib_runner.py drive --dist 1.80 --speed 0.15 --bag "$CAL_DIR/drive_straight/bag"
python3 scripts/calib/calib_runner.py capture --out "$CAL_DIR/drive_end.npz" --frames 10

# 离线求解（需先将各停点卷尺数据写入 $CAL_DIR/tape.json）
OPENBLAS_NUM_THREADS=1 python3 scripts/calib/solve_calibration.py --calib-dir "$CAL_DIR"
```

---

## 4. 本次实车标定结果与关键诊断（2026-11-26）

### 4.1 标定前（名义值）与标定后 6-DoF 外参对照

| 参数 | 原名义配置 (`run_real_navigation.sh`) | 本次实测标定值 | 差值 / 物理诊断说明 |
|---|---:|---:|---|
| **`sensor_x`** | `+0.2100 m` | **`+0.1754 m`** | 差 `-3.46 cm`（雷达光学原点位于壳体内部，且相对物理自旋中心偏后） |
| **`sensor_y`** | `0.0000 m` | **`+0.0025 m`** | 偏心仅 `+2.5 mm`，基本位于底盘纵向中心线上 |
| **`sensor_z`** | `+1.1250 m` | **`+1.1241 m`** | 6 停点标准差 `3.73 mm`，与名义值吻合至 `0.9 mm` |
| **`sensor_roll`** | `0.000°` | **`-0.049°`** (`-0.00085 rad`) | 雷达相对物理地面基本无横滚安装误差 |
| **`sensor_pitch`** | `0.000°` | **`+0.759°`** (`+0.01325 rad`) | 雷达实际存在约 $+0.76^\circ$ 前倾（开阔通道三站实测 $+0.67^\circ\text{–}+0.74^\circ$） |
| **`sensor_yaw`** | `0.000°` | **`-1.507°`** (`-0.02631 rad`) | 雷达朝向相对真实直行车头存在约 $-1.51^\circ$ 偏角 |

### 4.2 多重独立传感器交叉验证结果

1. **物理旋转中心与 R/F 标记偏置（卷尺 4 点闭合）**
   - 四个停点（`0° → +90° → -90° → 0°`）实测卷尺坐标：
     - `stop_0`: `R=(47.5, -8.7), F=(68.5, -9.9) cm`（标距 `21.03 cm`，航向 `-3.27°`）
     - `stop_1_p90`: `R=(86.9, -1.8), F=(88.7, 18.2) cm`（标距 `20.08 cm`，航向 `+84.86°`）
     - `stop_2_m90`: `R=(38.3, 29.0), F=(39.1, 8.0) cm`（标距 `21.02 cm`，航向 `-87.82°`）
     - `stop_3_0`: `R=(48.2, -9.8), F=(69.0, -10.0) cm`（标距 `20.80 cm`，航向 `-0.55°`）
   - 4 点刚体圆心拟合结果：物理旋转圆心在地面系为 `(63.60, 14.93) cm`，各站实际圆心仅移动 `3–6 mm`；**`R/F` 中点在自身局部系下相对物理旋转中心的固定偏置为 `(-4.51, -24.71) cm`（半径 `25.12 cm`）**，4 站拟合残差分别为 `0.36, 0.03, 0.04, 0.31 cm`。
   - 39 帧点云自旋拟合（含连续帧与静止帧）中位残差为 **`4.08 mm`**（剔除通道稀疏远景 `-75°..-86°` 退化帧后的 35 帧拟合 RMS 为 **`4.27 mm`**，给出 `sensor_x=+0.1782 m, sensor_y=+0.0015 m`，与全帧鲁棒拟合差异仅 `2.8 mm`）。

2. **IMU 重力与地面倾角对比（解开第 18 节 `1.6°` 倾角之谜）**
   - 在 6 个静止停点将 `/odin1/imu` 加速度矢量经 `/odin1/tf`（`imu -> lidar`）转换到 `lidar` 系：
     - **俯仰 `pitch`**：IMU 给出 $+0.57^\circ\text{–}+0.84^\circ$（通道内约 $+0.70^\circ$），与地面点云拟合（$+0.71^\circ\text{–}+0.76^\circ$）吻合至 $0.06^\circ$。
     - **横滚 `roll`**：地面点云拟合为 **$-0.05^\circ$**，但 IMU 经出厂 `imu -> lidar` 矩阵投影却报出 **$-1.60^\circ\text{–}-1.81^\circ$**。
     - 静止 IMU 的横滚与点云地面法向相差约 1.6°–1.8°，方向与设备姿态投影后测到的约 1.65° 倾斜一致。现有对照无法区分 IMU 本体偏差与出厂 IMU–LiDAR 外参误差。旧图配准还受高度截顶和墙面结构提取影响。[配准对照](TEST_RESULTS.md#glim-validation)

3. **直线行驶与底盘运动学交叉验证**
   - 直行段 ICP 测得雷达真实前移 **`1.7353 m`**，航向漂移 **`+0.740°`**；同期 400 Hz `/odin1/imu` 去偏置陀螺积分测得航向漂移 **`+0.635°`**（二者仅差 **$0.10^\circ$**）。
   - 同期 CAN 舵角反馈 `/steering_angles` 均值为 `[-0.90°, +2.60°, +0.90°]`（存在微小左偏残差，吻合真实行驶时的 $+0.65^\circ$ 微弱左偏弧线）；轮式里程计 `/odom` 报行进 `1.8333 m`（偏大约 `5.3%`）。

---

## 5. 上线与后续 Step 1b 使用建议

1. **在导航与 URDF 中启用新外参**：
   - 将 [`scripts/run_real_navigation.sh`](../scripts/run_real_navigation.sh) 开头参数更新为：
     ```bash
     sensor_x=0.1754
     sensor_y=0.0025
     sensor_z=1.1241
     sensor_roll=-0.00085
     sensor_pitch=0.01325
     sensor_yaw=-0.02631
     ```
   - 启动脚本已使用这组外参，`station_reference` 默认留空。更改外参会改变点云到 `base_link` 的投影和工位地图目标。用当前地图和外参重新采集参考帧后，再启用工位闭环。[采集步骤](NAVIGATION_GUIDE.md#工位-a-精确返航)
2. **全场建图进展。** 已用点云和原生 IMU 离线运行 GLIM，并以物理地面整平。新图与独立停车扫描的结果见[全场定位验收](TEST_RESULTS.md#glim-validation)。

全场 VGICP 入口另从地图配置读取匹配器外参。重新标定时同步核对启动脚本传给车体 TF 的六个值与地图配置中的 `mount_xyz`、`mount_rpy`。[GLIM 入口](../scripts/run_glim_navigation.sh) · [匹配器外参](../maps/glim_20261127/localization.yaml)
