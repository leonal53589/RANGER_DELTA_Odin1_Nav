# Odin1 留形模组单机测试与轻量建图工作空间 (project_slam)

本项目从 `26-Sentry` 导航工程中严格迁移了留形模组（Odin1）的驱动与建图代码，并针对单机手持/桌面测试进行了优化适配。

---

## 一、工作空间结构

```
project_slam/
├── src/
│   ├── odin_ros_driver/       # 留形 Odin1 官方驱动（已适配哨兵工程低负载参数与 SensorDataQoS）
│   ├── odin_slam/             # 点云累加 SLAM 节点与 Qt 控制界面（一键运行/暂停/保存 PCD）
│   └── odin_cloud_cleaner/    # 留形点云去噪与自车遮挡剔除预处理节点（备用）
├── maps/                      # 建图结果保存目录（自动生成 0.pcd, 1.pcd...）
└── README.md
```

---

## 二、首次运行环境配置（仅需配置一次）

### 1. 配置 USB 权限规则 (Udev)
留形模组通过 USB 3.0 与电脑通信，需要在系统配置 Udev 规则以获取设备读写权限：

```bash
# 1. 写入 udev 规则
sudo bash -c 'cat <<EOF > /etc/udev/rules.d/99-odin-usb.rules
SUBSYSTEM=="usb", ATTR{idVendor}=="2207", ATTR{idProduct}=="0019", MODE="0666", GROUP="plugdev"
EOF'

# 2. 重新加载并触发规则
sudo udevadm control --reload
sudo udevadm trigger
```

### 2. 硬件连接注意事项
* **务必插入电脑的原生 USB 3.0 接口**（蓝色接口）。
* 尽量**不要使用未外接供电的拓展坞/Hub**，留形内部有嵌入式计算芯片和激光器，瞬时功耗较大。
* 插入后可通过 `lsusb` 查看是否有 `2207:0019` 设备。

---

## 三、编译工作空间

本项目已经预先下载了官方预编译静态库 `liblydHostApi_amd.a` 并通过了编译。若后续修改了源码，重新编译只需：

```bash
cd /home/leona/WorkingSpace/navigation2/project_slam
colcon build --symlink-install
```

---

## 四、启动与建图测试

### 方式 A：一键全启动（推荐）
包含 **留形驱动** + **SLAM 控制窗口** + **预设 RViz2 视图**：

```bash
cd /home/leona/WorkingSpace/navigation2/project_slam
source install/setup.bash
ros2 launch odin_slam slam_bringup.launch.py
```

> **可选参数**：
> * 不弹 RViz2：`ros2 launch odin_slam slam_bringup.launch.py launch_rviz:=false`

---

### 方式 B：分终端启动（方便分别查看驱动日志与建图状态）

* **终端 1：启动传感器驱动**
  ```bash
  cd /home/leona/WorkingSpace/navigation2/project_slam
  source install/setup.bash
  ros2 launch odin_ros_driver odin1_ros2.launch.py
  ```
  *(检查输出是否提示 `Device connected` 与 `Start stream success`)*

* **终端 2：启动 SLAM 节点与控制窗口**
  ```bash
  cd /home/leona/WorkingSpace/navigation2/project_slam
  source install/setup.bash
  ros2 launch odin_slam odin_slam.launch.py
  ```

* **终端 3：单独打开预设配置的 RViz2（可选）**
  ```bash
  rviz2 -d $(ros2 pkg prefix odin_ros_driver)/share/odin_ros_driver/config/odin_ros2.rviz
  ```

---

## 五、建图操作指引

启动后会弹出一个名为 **`control_dialog`** 的小窗口，包含三个按钮：

1. **运行 SLAM**：
   * 点击后开始监听并累加 `/odin1/cloud_slam` 点云。
   * 手持留形传感器在房间或走廊内**平稳慢速移动**，绕行一圈形成闭环。
   * 在 RViz2 中可以看到点云被实时拼接在固定坐标系 `odom` 下。
2. **暂停 SLAM**：
   * 行走结束或需要暂停时点击，停止累加点云。
3. **保存结果**：
   * 点击后，程序会对累加的点云执行：
     1. `CropBox` 空间裁剪滤波
     2. `VoxelGrid` 体素降采样（默认 0.02m 分辨率）
     3. `StatisticalOutlierRemoval` 离群噪点剔除
   * 点云地图会自动保存至 `project_slam/maps/` 目录下（如 `0.pcd`, `1.pcd`）。

---

## 六、查看建图结果

安装或使用 PCL 查看器查看生成的点云地图：

```bash
# 安装 pcl-tools (若未安装)
sudo apt install -y pcl-tools

# 查看点云
pcl_viewer /home/leona/WorkingSpace/navigation2/project_slam/maps/0.pcd
```

在 `pcl_viewer` 中：
* 观察墙壁厚度（反映手持漂移大小与建图精度）。
* 观察回到起点时墙面是否有重影或断层（检验闭环效果）。

---

## 七、常用诊断命令

如果传感器未出数据，可以使用以下命令排查：

```bash
# 查看是否有 odin 话题
ros2 topic list | grep odin

# 查看高频里程计频率（正常约为 400Hz）
ros2 topic hz /odin1/odometry_highfreq

# 查看 SLAM 点云频率（正常约为 10-15Hz）
ros2 topic hz /odin1/cloud_slam

# 查看 IMU 频率（正常约为 400Hz）
ros2 topic hz /odin1/imu
```
