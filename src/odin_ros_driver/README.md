# Odin ROS 驱动

Odin 传感器模块（Manifold Tech Ltd.）的 ROS 驱动套件。

Odin1 Wiki：https://manifoldtechltd.github.io/wiki/Odin1/Cover.html

## 兼容性

- ROS 1（推荐使用 LTS 版本 Noetic）
- ROS 2（推荐使用 LTS 版本 Humble）

## 重要说明

本驱动包为点云 SLAM 应用提供核心功能，面向特定使用场景，仅供专业技术人员进行二次开发。在实际部署环境中，用户需根据具体场景进行优化和定制开发，以满足实际运行要求。

## 1. 版本

- 当前驱动版本：v0.13.1
- 所需设备固件版本：v0.13.0

## 2. 环境准备

### 2.1 操作系统要求

- ROS Noetic 和 ROS 2 Foxy：Ubuntu 20.04
- ROS 2 Humble：Ubuntu 22.04
- 暂不支持 Ubuntu 18.04
- Ubuntu 24.04 尚未获得官方支持，但经过适当修改后可能可以运行

### 2.2 依赖项

- OpenCV >= 4.2.0（推荐 4.5.5 或 4.8.0；请确保系统中只安装一个版本的 OpenCV）
- yaml-cpp
- thread
- OpenSSL
- Eigen3

### 2.3 安装依赖

#### 2.3.1 系统依赖

```shell
sudo apt update
sudo apt-get install build-essential cmake git libgtk2.0-dev pkg-config libavcodec-dev libavformat-dev libswscale-dev
```

#### 2.3.2 yaml-cpp

```shell
sudo apt update
sudo apt install -y libyaml-cpp-dev
```

#### 2.3.3 libusb

```shell
sudo apt update
sudo apt install -y libusb-1.0-0-dev
```

#### 2.3.4 OpenCV

```shell
sudo apt update
sudo apt-get install libopencv-dev
```

#### 2.3.5 ROS

- ROS Noetic：[安装说明](https://wiki.ros.org/noetic/Installation)
- ROS 2 Foxy：[安装说明](https://docs.ros.org/en/foxy/Installation/Ubuntu-Install-Debians.html)
- ROS 2 Humble：[安装说明](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html)

## 3. 编译与运行

### 3.1 创建 udev 规则

```shell
sudo vim /etc/udev/rules.d/99-odin-usb.rules
```

在 `99-odin-usb.rules` 文件中加入以下内容：

```shell
SUBSYSTEM=="usb", ATTR{idVendor}=="2207", ATTR{idProduct}=="0019", MODE="0666", GROUP="plugdev"
```

重新加载规则并重新插入设备：

```shell
sudo udevadm control --reload
sudo udevadm trigger
```

### 3.2 获取源码

```shell
git clone https://github.com/manifoldsdk/odin_ros_driver.git catkin_ws/src/odin_ros_driver
```

> 请将源码克隆到 `[ros_workspace]/src/` 目录中，否则编译时可能出错。

### 3.3 编译

#### 3.3.1 ROS 1（以 Noetic 为例）

```shell
source /opt/ros/noetic/setup.bash
./script/build_ros.sh
```

#### 3.3.2 ROS 2（以 Foxy 为例）

```shell
source /opt/ros/foxy/setup.bash
./script/build_ros2.sh
```

### 3.4 运行

#### 3.4.1 ROS 1（以 Noetic 为例）

```shell
source [ros_workspace]/devel/setup.bash
roslaunch odin_ros_driver [launch file]
```

- `odin_ros_driver`：包名
- `launch file`：Launch 文件名
- `ros_workspace`：用户的 ROS 工作空间

运行示例：

```shell
roslaunch odin_ros_driver odin1_ros1.launch
```

#### 3.4.2 ROS 2（以 Foxy 为例）

```shell
source [ros2_workspace]/install/setup.bash
ros2 launch odin_ros_driver [launch file]
```

- `odin_ros_driver`：包名
- `launch file`：Launch 文件名
- `ros2_workspace`：用户的 ROS 2 工作空间

运行示例：

```shell
ros2 launch odin_ros_driver odin1_ros2.launch.py
```

### 3.5 工作模式

工作模式可通过 `config/control_command.yaml` 中的 `custom_map_mode` 参数配置。

#### 3.5.1 里程计模式

设置 `custom_map_mode = 0` 启用里程计模式。在该模式下，`map` 坐标系与 `odom` 坐标系的位姿一致。

如果发现里程计数据发生漂移，可执行以下脚本命令动态重置算法：

```shell
./set_param.sh algo_reset 1
```

#### 3.5.2 SLAM 模式

设置 `custom_map_mode = 1` 启用 SLAM 模式。该模式在里程计模式的基础上增加了**回环检测**和**地图保存**功能，提供完整的 SLAM 系统。

驱动启动后，Odin1 会自动进行建图并缓存地图数据。场景采集完成后，请在驱动源码目录中执行以下命令，保存程序启动以来采集的全部地图数据：

```shell
./set_param.sh save_map 1
```

地图将保存到 `config/control_command.yaml` 中 `mapping_result_dest_dir` 和 `mapping_result_file_name` 参数指定的位置。如果未指定这些参数，则使用默认值。

首次保存后，可以再次执行该命令保存新地图。每次保存都会生成一个新的地图文件，连续两次保存操作之间请至少间隔 5 秒。

地图原点对应程序启动时 `odom` 坐标系的原点。

#### 3.5.3 重定位模式

设置 `custom_map_mode = 2`，并通过 `config/control_command.yaml` 中的 `relocalization_map_abs_path` 参数指定预先构建地图的绝对路径，即可启用重定位。

启动后，Odin1 会根据当前视角和指定地图开始重定位。为提高成功率，建议从原 SLAM 轨迹起始位姿的 1 米、正负 10 度范围内开始。

重定位效果很大程度上取决于环境。在特征明显的场景中，即使超出 1 米、10 度范围也可能成功匹配；而其他环境可能要求更严格的初始条件。建议在目标环境中进行测试，以确定实际可用范围。

如果初次重定位失败，系统会暂时进入备用 SLAM 模式，此状态下无法保存地图。此时可以自由移动 Odin1，系统会在后台持续尝试重定位。成功后，系统将发布 `map` 与 `odom` 坐标系之间的 TF。初始化后轻微晃动或移动设备，有助于提高重定位精度。

以下 Topic 发布在 `odom` 坐标系中：`/odin1/cloud_slam`、`/odin1/odometry`、`/odin1/odometry_highfreq` 和 `/odin1/path`。如需获取 `map` 坐标系下的数据，请应用从 `odom` 到 `map` 的 TF。

## 4. 文件结构与数据格式

### 4.1 文件结构

```text
Odin_ROS_Driver/                     # ROS 1/ROS 2 驱动包
    3rdparty/                        # 第三方库
    src/
        host_sdk_sample.cpp          # 示例源码
        yaml_parser.cpp              # 读取 YAML 参数的源码
        rawCloudRender.cpp           # RenderCloud 源码
        depth_image_ros_node.cpp     # ROS 1 深度图节点
        depth_image_ros2_node.cpp    # ROS 2 深度图节点
        pcd2depth_ros.cpp            # ROS 1 pcd2depth 源码
        pcd2depth_ros2.cpp           # ROS 2 pcd2depth 源码
        pointcloud_depth_converter.cpp # 点云深度转换源码
        cloud_reprojection_ros.cpp   # ROS 1/ROS 2 点云重投影节点源码
        cloud_reprojector.cpp        # 点云重投影核心逻辑
    lib/
        liblydHostApi_amd.a          # AMD 平台静态库
        liblydHostApi_arm.a          # ARM 平台静态库
    include/
        host_sdk_sample.h            # 示例头文件
        lidar_api_type.h             # API 数据结构头文件
        lidar_api.h                  # API 函数声明
        yaml_parser.h                # 参数文件读取头文件
        rawCloudRender.h             # RenderCloud API
        data_logger.h                # save_data 日志接口
        depth_image_ros_node.hpp     # ROS 1 深度图节点
        depth_image_ros2_node.hpp    # ROS 2 深度图节点
        pointcloud_depth_converter.hpp # 点云深度转换接口
        cloud_reprojection_ros_node.hpp # ROS 1/ROS 2 点云重投影节点
        cloud_reprojector.hpp        # 点云重投影核心类
    config/
        control_command.yaml         # 驱动控制参数文件
        calib.yaml                   # 设备标定文件；每台设备不同，连接驱动时从设备读取
    launch_ROS1/
        odin1_ros1.launch            # ROS 1 Launch 文件
    launch_ROS2/
        odin1_ros2.launch.py         # ROS 2 Launch 文件
    script/
        build_ros.sh                 # ROS 1 安装脚本
        build_ros2.sh                # ROS 2 安装脚本
    recorddata/                      # 可导入 MindCloud 的录制数据
    log/                             # 日志文件
        Driver_{timestamp}/          # 每次启动驱动生成的日志目录
            Conn_{timestamp}/        # 每次连接 Odin1 生成的日志目录
                dev_status.csv       # 设备状态日志
    README.md                        # 使用说明
    CMakeLists.txt                   # CMake 构建文件
    LICENSE                          # 许可证文件
```

### 4.2 Launch 文件

| Launch 文件名 | 说明 |
|---|---|
| `odin1_ros1.launch` | ROS 1 Odin1 基础操作演示 |
| `odin1_ros2.launch.py` | ROS 2 Odin1 基础操作演示 |

### 4.3 ROS Topic

Odin ROS 驱动的内部参数定义在 `config/control_command.yaml` 中。常用 Topic 及其控制参数如下：

| Topic | `control_command.yaml` 参数 | 详细说明 |
|---|---|---|
| `odin1/imu` | `sendimu` | IMU Topic |
| `odin1/image` | `sendrgb` | RGB 相机 Topic，由设备原始 JPEG 数据解码而来，格式为 `bgr8` |
| `odin1/image_undistort` | `sendrgbundistort` | RGB 相机去畸变 Topic，使用设备中的 `calib.yaml` 处理 |
| `odin1/image/compressed` | `sendrgbcompressed` | RGB 相机压缩 Topic，内容为设备原始 JPEG 数据 |
| `odin1/cloud_raw` | `senddtof` | 原始点云 Topic |
| `odin1/cloud_render` | `sendcloudrender` | 渲染点云 Topic，由原始点云、RGB 图像和设备中的 `calib.yaml` 处理得到 |
| `odin1/cloud_slam` | `sendcloudslam` | SLAM 点云 Topic |
| `odin1/odometry` | `sendodom` | 里程计 Topic |
| `odin1/odometry_highfreq` | `sendodom` | 高频里程计 Topic |
| `odin1/path` | `showpath` | 里程计轨迹 Topic |
| `tf` | `sendodom` | TF 树 Topic |
| `odin1/depth_img_competetion` | `senddepth` | 稠密深度图 Topic。演示功能，对算力要求较高，与 `odin1/image_undistort` 一一对应。使用数据时请直接订阅该 Topic，不要使用 `echo`。原始值已经是深度数据，无需再次转换 |
| `odin1/depth_img_competetion_cloud` | `senddepth` | 稠密深度点云 Topic。演示功能，对算力要求较高 |
| `odin1/reprojected_image` | `sendreprojection` | 点云重投影图像 Topic。使用里程计将 `cloud_slam` 投影到相机图像中，在主机端处理 |

### 4.4 数据格式

#### 4.4.1 原始点云

原始点云 `cloud_raw` 包含以下字段：

```text
float32 x             # X 轴，单位：米
float32 y             # Y 轴，单位：米
float32 z             # Z 轴，单位：米
uint8  intensity      # 反射强度，范围：0～255
uint16 confidence     # 点置信度；典型场景中的实际范围约为 0～1300，值越大越可靠
float32 offset_time   # 相对于基准时间戳的偏移，单位：秒
```

`confidence` 推荐过滤阈值为 30～35，实际使用时应根据场景调整。

如需在 PCL 中使用此自定义格式，请先定义点类型：

```cpp
/*** LS ***/
namespace ls_ros {
    struct EIGEN_ALIGN16 Point {
        float x;
        float y;
        float z;
        uint8_t intensity;
        uint16_t confidence;
        float offset_time;
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    };
}  // namespace ls_ros

POINT_CLOUD_REGISTER_POINT_STRUCT(ls_ros::Point,
      (float, x, x)
      (float, y, y)
      (float, z, z)
      (uint8_t, intensity, intensity)
      (uint16_t, confidence, confidence)
      (float, offset_time, offset_time)
)
```

然后可将 ROS `sensor_msgs::PointCloud2` 消息转换为 PCL 点云：

```cpp
pcl::PointCloud<ls_ros::Point> ls_cloud;
pcl::fromROSMsg(*msg, ls_cloud);
```

#### 4.4.2 SLAM 点云与渲染点云

SLAM 点云 `cloud_slam` 和直接渲染点云 `cloud_render` 包含以下字段：

```text
float32 x             # X 轴，单位：米
float32 y             # Y 轴，单位：米
float32 z             # Z 轴，单位：米
float32 rgb           # RGB 值
```

### 4.5 其他功能参数

| `control_command.yaml` 参数 | 详细说明 |
|---|---|
| `use_host_ros_time` | 时间同步模式：`0` 表示使用 Odin 内部系统时间作为数据时间戳（常用且推荐）；`1` 表示使用主机收到数据时的 ROS 时间（不推荐大多数用户使用）；`2` 表示通过类似 NTP 的方式将 Odin1 时间对齐到主机时间，时间戳为主机时间轴上的传感器数据接收时间 |
| `strict_usb3.0_check` | 严格检查 USB 3.0。关闭后，即使 USB 连接低于 USB 3.0 也允许连接 |
| `recorddata` | 按可导入 MindCloud(TM) 的特定格式录制数据，用于后处理。该功能会占用大量存储空间，测试数据约为每 10 分钟 9.5 GB。录制文件（IMU、图像、点云、Pose、Rotate）中每帧的时间戳与 `use_host_ros_time` 使用相同的对齐策略：在 NTP 模式（`use_host_ros_time=1` 或 `2`）下，录制时间戳为 NTP 对齐后的主机时间，而非 Odin1 开机时间 |
| `devstatuslog` | 记录设备状态。目前会将 SoC 温度、CPU 使用率、内存使用率、DTOF 传感器温度以及数据收发速率等信息保存至 `log` 目录下的 `dev_status.csv`。每次启动驱动都会创建新文件 |
| `showcamerapose` | 显示相机位姿和视场角 |
| `custom_map_mode` | 工作模式：`0` 为里程计模式，`map` 与 `odom` 位姿一致；`1` 为带回环检测的建图模式，支持保存地图；`2` 为重定位模式，需指定地图文件绝对路径，成功后输出 `map` 与 `odom` 之间的 TF |
| `custom_init_pos` | 初始化位置，当前未使用 |
| `relocalization_map_abs_path` | 重定位模式使用的地图文件绝对路径 |
| `mapping_result_dest_dir` 和 `mapping_result_file_name` | 建图模式下保存地图的路径和文件名；未指定时使用默认值 |

### 4.6 通过 ROS Service 在线调节 AE/AWB

驱动启动后会注册 4 个 ROS Service，允许在不重启驱动的情况下，从另一个终端动态调节相机的自动曝光（AE）和自动白平衡（AWB）。底层 SDK 调用与驱动主控制路径共用同一把互斥锁，因此可与正常数据流并发调用。

#### 4.6.1 Service 列表

| Service 名称 | 类型 | 用途 |
|---|---|---|
| `/odin1/get_ae` | `odin_ros_driver/srv/GetAe` | 查询当前 AE 状态 |
| `/odin1/get_awb` | `odin_ros_driver/srv/GetAwb` | 查询当前 AWB 状态 |
| `/odin1/set_ae` | `odin_ros_driver/srv/SetAe` | 设置 AE 模式及手动曝光时间、增益 |
| `/odin1/set_awb` | `odin_ros_driver/srv/SetAwb` | 设置 AWB 模式及手动 R/B 增益 |

#### 4.6.2 请求字段、范围与物理含义

`SetAe.Request`：

| 字段 | 范围 | 含义 |
|---|---|---|
| `mode` | `0`（AUTO）或 `1`（MANUAL） | `0`：由设备自动调节 AE，忽略下面两个浮点参数；`1`：设备锁定 AE 并应用指定值 |
| `exposure_time` | `0.0001`～`0.033` 秒，仅手动模式 | 每帧传感器曝光时间。时间越长画面越亮，但运动模糊越明显 |
| `gain` | `1.0`～`64.0`，仅手动模式 | 模拟增益。增益越高输出越亮，但信噪比越低 |

`SetAwb.Request`：

| 字段 | 范围 | 含义 |
|---|---|---|
| `mode` | `0`（AUTO）或 `1`（MANUAL） | `0`：由设备自动调节 AWB；`1`：设备锁定 AWB 并应用指定增益 |
| `rgain` | `0.1`～`4.0`，仅手动模式 | R 通道增益。相对 `bgain` 越大，画面越偏暖（黄/红） |
| `bgain` | `0.1`～`4.0`，仅手动模式 | B 通道增益。相对 `rgain` 越大，画面越偏冷（蓝） |

> Gr/Gb 通道由设备固定为 `1.0`，不可调节。

#### 4.6.3 响应字段

4 个 Service 均返回 `success`（`bool`）和 `rc`（`int32`），Get 类 Service 还会返回查询到的状态字段。

`GetAe.Response`：

| 字段 | 典型范围 | 含义 |
|---|---|---|
| `exposure_time` | `0.0001`～`0.033` 秒 | 当前曝光时间 |
| `gain` | `1.0`～`64.0` | 当前模拟增益 |
| `iso` | `100`～`6400` | 等效 ISO |
| `brightness` | `0`～`255` | 平均帧亮度 |
| `is_converged` | `0` 或 `1` | `1` 表示 AE 已收敛 |
| `env_lv` | `0`～`15` | 环境亮度指数，值越大表示环境越亮 |
| `fps` | 约 `10`、`14.5` 或 `29` | 当前帧率 |

`GetAwb.Response`：

| 字段 | 典型范围 | 含义 |
|---|---|---|
| `rgain` / `bgain` | `0.1`～`4.0` | R/B 通道增益 |
| `grgain` / `gbgain` | 固定为 `1.0` | 由设备固定的 Gr/Gb 增益 |
| `cct` | `2500`～`8000` K | 相关色温 |
| `ccri` | `-50`～`50` | 色温偏离指数，`0` 表示位于普朗克轨迹上 |
| `is_converged` | `0` 或 `1` | `1` 表示 AWB 已收敛 |

#### 4.6.4 `rc` 返回码

| `rc` | 含义 |
|---|---|
| `0` | 成功 |
| `400` | 设备载荷过短 |
| `401` | 设备不支持该操作码 |
| `402` | 参数长度错误 |
| `403` | **参数越界**，手动参数超出上表范围时最常见 |
| `404` | 设备端 Socket 错误 |
| `405` | 设备端 `ae_control` 无响应，请确认 `lydapp` 已运行 |
| `255`（`0xFF`） | `ae_control` 报告未知操作码 |
| `-1` | SDK 未初始化 |
| `-2`～`-5` | USB 传输异常、超时或响应格式错误 |
| `-100` | **驱动尚未打开设备**，请等待设备连接成功 |

#### 4.6.5 调用示例

ROS 2（Humble）：在一个终端启动驱动，然后在另一个终端执行：

```bash
source install/setup.bash

# 查询当前状态
ros2 service call /odin1/get_ae  odin_ros_driver/srv/GetAe
ros2 service call /odin1/get_awb odin_ros_driver/srv/GetAwb

# 将 AE 设置为自动
ros2 service call /odin1/set_ae odin_ros_driver/srv/SetAe "{mode: 0}"

# 将 AE 设置为手动，曝光时间 10 ms，增益 4.0
ros2 service call /odin1/set_ae odin_ros_driver/srv/SetAe \
  "{mode: 1, exposure_time: 0.010, gain: 4.0}"

# 将 AWB 设置为手动，rgain=1.5，bgain=2.0
ros2 service call /odin1/set_awb odin_ros_driver/srv/SetAwb \
  "{mode: 1, rgain: 1.5, bgain: 2.0}"

# 恢复自动模式
ros2 service call /odin1/set_ae  odin_ros_driver/srv/SetAe  "{mode: 0}"
ros2 service call /odin1/set_awb odin_ros_driver/srv/SetAwb "{mode: 0}"

# 查看 srv 完整定义
ros2 interface show odin_ros_driver/srv/SetAe
```

ROS 1（Noetic）：启动驱动后，在另一个终端执行：

```bash
source devel/setup.bash

# 查询当前状态
rosservice call /odin1/get_ae
rosservice call /odin1/get_awb

# 将 AE 设置为手动
rosservice call /odin1/set_ae "{mode: 1, exposure_time: 0.010, gain: 4.0}"

# 将 AWB 设置为手动
rosservice call /odin1/set_awb "{mode: 1, rgain: 1.5, bgain: 2.0}"

# 恢复自动模式（ROS 1 要求填写全部字段）
rosservice call /odin1/set_ae  "{mode: 0, exposure_time: 0.0, gain: 0.0}"
rosservice call /odin1/set_awb "{mode: 0, rgain: 0.0, bgain: 0.0}"

# 查看 srv 完整定义
rossrv show odin_ros_driver/SetAe
```

#### 4.6.6 不同场景的推荐初始参数

AE（`exposure_time`、`gain`）：

| 场景 | `exposure_time` | `gain` |
|---|---|---|
| 明亮室外 | `0.001`～`0.005` 秒 | `1.0`～`2.0` |
| 普通室内 | `0.008`～`0.015` 秒 | `2.0`～`8.0` |
| 暗光环境 | `0.020`～`0.030` 秒 | `8.0`～`32.0` |
| 极暗环境 | `0.033` 秒 | `32.0`～`64.0` |

AWB（`rgain`、`bgain`）：

| 目标色调 | `rgain` | `bgain` |
|---|---|---|
| 暖色（钨丝灯、夕阳） | `2.0`～`2.5` | `1.0`～`1.2` |
| 中性（D65 日光） | `1.5`～`1.7` | `1.8`～`2.0` |
| 冷色（阴天、荧光灯） | `1.2`～`1.4` | `2.2`～`2.6` |
| 极冷色 | `1.0` | `3.0`～`4.0` |

#### 4.6.7 注意事项

- Service 等待设备响应时最长会阻塞约 10 秒，正常情况下会在几十毫秒内返回。
- 手动模式不会在驱动或设备重启后保留，每次重新连接时都会恢复为 AUTO。
- `rc = -100` 表示驱动尚未打开设备。请等待驱动日志显示 `device connected` 后再调用。
- `exposure_time` 的有效最大值受帧周期 `1 / fps` 限制。`dtof_fps = 290` 时帧率为 29 Hz、周期约为 34 ms，因此 `0.033` 秒的上限已接近帧周期边界。

## 5. 常见问题

### 5.1 重新启动 Host SDK 时出现段错误

**错误信息**

```text
No device connected after 60 seconds
```

**解决方法**

1. 断开并重新接通 Odin 模块电源。
2. 设备重启后重新初始化 Odin SDK。

### 5.2 编译时库链接失败

**错误信息**

```text
ld: cannot find -llydHostApi
```

也可能出现符号查找错误。

**解决方法**

1. 清理此前的构建产物。

ROS 1：

```shell
rm -rf devel/ build/
```

ROS 2：

```shell
rm -rf devel/ install/ log/
```

2. 重新运行安装脚本。

### 5.3 Docker 图形界面透传失败

**错误信息**

```text
Unable to open X display
No protocol specified
```

**解决方法**

```shell
xhost + # 允许 Docker 容器透传图形界面
```

### 5.4 ROS 驱动因获取版本失败而退出

**错误信息**

```text
<ERROR><api.cpp:lidar_get_version:672>: get device version fail.
get version failed.
```

**解决方法**

设备固件版本过低，请升级到最新版本。

### 5.5 RViz 长时间无响应

**错误信息**

RViz 无响应，一段时间后终端输出：

```text
Device disconnected, waiting for reconnection...
```

**解决方法**

请重新接通 Odin 模块电源。

### 5.6 设备无响应

**错误信息**

```text
Missed ok response from device, probably wrong interaction procedure.
```

**解决方法**

请采用 5.1 节中的解决方法。

### 5.7 设备缺少外部标定文件

**错误信息**

```text
ERROR: Missing camera node 'cam_0'
```

**解决方法**

请重新插拔 USB 设备。

### 5.8 数据流启动后，ROS 驱动立即报告设备断开

**错误信息**

```text
Device ready and streams activated
Device detaching...
Wating for device reconnection...
Device disconnected, waiting for reconnection...
```

**原因**

该问题常见于 ROS 2 环境连接了复杂网络（如同时连接办公 Wi-Fi 和以太网）的情况。ROS 2 默认使用广播，复杂的网络环境可能阻塞 ROS 2 发布过程，进而导致设备断开。

**解决方法**

如果不需要跨设备通信，请将 ROS 2 通信限制在本机：

```shell
export ROS_LOCALHOST_ONLY=1
```

如果需要跨设备通信，请尽量简化网络环境。建议建立仅包含必要设备的小型局域网。

### 5.9 数据流启动后，ROS 驱动进程立即退出

**错误信息**

```text
Device ready and streams activated
[host_sdk_sample-2] process has died ......
```

**排查方法**

在 `control_command.yaml` 中设置 `sendrgb = 0`，禁用 `odin1/image` 后重试。如果驱动能够正常运行，则问题很可能是系统中安装了多个版本的 OpenCV。

**解决方法**

卸载不使用的 OpenCV 版本，仅保留一个完整版本，然后重新编译驱动并重试。

### 5.10 ROS 驱动输出 `TF_OLD_DATA ignoring data` 警告

**错误信息**

```text
[rviz2-3] Warning: TF_OLD_DATA ignoring data from the past for frame odin1_base_link at time 20.547632 according to authority Authority undetectable
[rviz2-3] Possible reasons are listed at http://wiki.ros.org/tf/Errors%20explained
[rviz2-3]          at line 294 in ./src/buffer_core.cpp
```

**原因**

这是 ROS 和 RViz 的提示，表示部分 TF 数据因时间戳冲突而被忽略。通常发生在 ROS 驱动保持运行时，用户重启了 Odin 设备电源。Odin 的内部系统时间会被重置，新数据的时间戳与 RViz 上次运行时接收的旧数据发生冲突。

**解决方法**

点击 RViz 界面底部的 Reset 按钮，重置 RViz 内部状态即可停止该警告。

### 5.11 ROS 驱动输出 `unknown cmd code: xx` 错误

**错误信息**

```text
<ERROR><api.cpp:cmd_data_deal:418>: unknow command code 21.
```

**原因**

ROS 驱动版本与设备固件版本不匹配，导致驱动无法解析新版固件中新增的数据。

**解决方法**

请确保使用最新版本的 ROS 驱动和设备固件。

### 5.12 USB 设备访问错误（`LIBUSB_ERROR_BUSY` 或 `LIBUSB_ERROR_ACCESS`）

**错误信息**

```text
libusb: error [udev_hotplug_event] ignoring udev action bind
LIBUSB_ERROR_BUSY
```

或：

```text
libusb: error [_get_usbfs_fd] libusb couldn't open USB device /dev/bus/usb/xxx/xxx, errno=13
LIBUSB_ERROR_ACCESS
```

**原因**

- `LIBUSB_ERROR_BUSY`：其他进程正在使用该 USB 设备。常见原因是同时运行了多个 ROS 驱动实例，或之前崩溃的应用仍持有设备句柄。
- `LIBUSB_ERROR_ACCESS`：当前用户无权访问 USB 设备。通常由缺少 udev 规则或用户权限不足导致。

**解决方法**

对于 `LIBUSB_ERROR_BUSY`：

1. 检查是否有其他驱动实例正在运行：

```shell
ps aux | grep host_sdk_sample
```

2. 结束现有实例：

```shell
killall host_sdk_sample
```

3. 如果问题仍然存在，请重新插拔 USB 设备以重置设备状态。

对于 `LIBUSB_ERROR_ACCESS`：

1. 为设备添加 udev 规则。创建 `/etc/udev/rules.d/99-odin.rules` 文件并写入：

```shell
SUBSYSTEM=="usb", ATTR{idVendor}=="2207", ATTR{idProduct}=="0019", MODE="0666", GROUP="plugdev"
```

2. 重新加载 udev 规则：

```shell
sudo udevadm control --reload-rules
sudo udevadm trigger
```

3. 也可以使用 `sudo` 运行驱动，但生产环境中不推荐这样做：

```shell
sudo -E ros2 launch odin_ros_driver odin_ros_driver.launch.py
```

4. 确保当前用户属于 `plugdev` 用户组：

```shell
sudo usermod -aG plugdev $USER
```

执行后需注销并重新登录，用户组变更才会生效。

### 5.13 ros2 bag 录制丢失高频 Topic（IMU / odometry_highfreq）

**现象**

使用 `ros2 bag record` 录制时，低频 Topic（cloud、image、odometry、wiwc）完整无丢失，但 `/odin1/imu`（400 Hz）和 `/odin1/odometry_highfreq`（400 Hz）会出现样本丢失。分析脚本显示消息间隔达到正常周期的 2 倍或更长，而 SDK 侧没有丢失报告，`ros2 topic hz` 等在线订阅者也观察不到丢失。

**原因**

驱动使用 `RELIABLE` QoS 发布 `/odin1/imu` 和 `/odin1/odometry_highfreq`。`ros2 bag record` 默认订阅配置为 `history = keep_last`、`depth = 10`，在 400 Hz 下只能缓冲约 25 ms 的样本。当录制端出现短暂延迟（磁盘刷新、MCAP/SQLite 数据块写入或调度抖动）时，订阅队列会溢出，DDS 在**订阅端**静默丢弃最早的样本。SDK 和发布端不受影响，因此驱动日志和 `ros2 topic hz` 都不会报告丢失。

**解决方法**

使用仓库提供的 QoS 配置文件 `script/rosbag2_qos.yaml`，增大录制端对两个高频 Topic 的订阅队列深度：

```yaml
# script/rosbag2_qos.yaml
/odin1/imu:
  reliability: reliable
  history: keep_last
  depth: 4000

/odin1/odometry_highfreq:
  reliability: reliable
  history: keep_last
  depth: 4000
```

录制时通过 `--qos-profile-overrides-path` 应用：

```shell
ros2 bag record -a \
    --qos-profile-overrides-path src/odin_ros_driver/script/rosbag2_qos.yaml \
    -o my_bag
```

也可以只录制指定 Topic：

```shell
ros2 bag record \
    --qos-profile-overrides-path src/odin_ros_driver/script/rosbag2_qos.yaml \
    -o my_bag \
    /odin1/imu /odin1/odometry_highfreq /odin1/odometry /odin1/wiwc /odin1/cloud_raw
```

**进一步优化**

如果应用上述配置后仍有丢失，通常是磁盘速度较慢，可继续尝试：

```shell
# 使用 MCAP 后端和更大的内部缓存，速度通常快于 SQLite3
ros2 bag record -s mcap --max-cache-size 1073741824 \
    --qos-profile-overrides-path src/odin_ros_driver/script/rosbag2_qos.yaml \
    -o my_bag \
    /odin1/imu /odin1/odometry_highfreq ...

# 增大内核 UDP Socket 缓冲区。默认值仅为 208 KB，
# 这是 400 Hz RELIABLE 流量中常见的隐藏瓶颈
sudo sysctl -w net.core.rmem_max=33554432
sudo sysctl -w net.core.wmem_max=33554432
```

**ROS 1 是否有相同问题？**

没有。ROS 1 使用基于 TCP 的发布/订阅，发布端和订阅端各自只有一个 `queue_size` 参数，不存在 ROS 2 中发布端与订阅端 QoS 配置不匹配的问题。本驱动的 ROS 1 路径已将 IMU 和 `odometry_highfreq` 的发布队列设为 `queue_size = 4000`，详见 `include/host_sdk_sample.h` 中 `initialize_publishers` 的 ROS 1 分支。`rosbag record` 使用的 TCP 传输本身也提供可靠传输。因此 ROS 1 不会出现这种特定的丢帧现象，无需额外配置。

### 5.14 Ubuntu 24.04（ROS 2 Jazzy）下 RViz 无法转换点云或 TF

**现象**

在 Ubuntu 24.04（ROS 2 Jazzy）中，将 RViz 的 **Fixed Frame** 设为 `odom` 后，位于 `lidar` 坐标系的 `cloud_raw` 无法显示，RViz 会报告 `odom -> imu -> lidar` 链的变换或外推错误。相同配置在 Ubuntu 22.04（Humble）中可以正常工作。

**原因**

设备点云的时间戳通常比里程计时间戳超前约 100 ms。ROS 2 Jazzy 的 tf2 会严格按时间戳查询，且不允许外推。当 TF 仅按里程计频率和里程计时间戳发布时，tf2 找不到覆盖点云时间戳的变换，因此查询失败。

**解决方法**

在 `config/control_command.yaml` 中将 `tf_extra_publish_rate` 设置为大于 `0` 的值，例如 `100`。这会启动一个额外定时器，按照指定频率和不断推进的时间戳重复发布 `odom -> imu` 与 `imu -> lidar` 变换，使 tf2 始终能找到覆盖点云时间戳的变换。

```yaml
# config/control_command.yaml
tf_extra_publish_rate: 100  # 0：关闭；>0：发布频率（Hz），推荐用于 ROS 2 Jazzy / Ubuntu 24.04
```

Ubuntu 22.04（Humble）或 ROS 1 通常不需要该配置，可保持 `0`。

## 6. 技术支持

如需技术支持，请联系：support@manifoldtech.cn

为便于现场应用工程师（FAE）诊断问题，请提供以下信息：

1. 当前固件版本：

```text
[device_version_capture]: ros_driver_version: [Version Number]
```

2. 当前使用的电源适配器和转接线照片。
3. 问题是偶发还是稳定复现。
4. 问题发生场景的图片。
5. 第 5 节中的故障排查方法是否解决了问题。
6. 期望的问题解决时间。
