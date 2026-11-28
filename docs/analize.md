# ODIN1 设备采图源码笔记

本文记录设备驱动、SLAM 点云拼接和 Qt 保存流程。全场运行时的 VGICP 定位见[导航代码原理](NAVIGATION_MATHEMATICS_AND_PRINCIPLES.md#glim-runtime)。

## 设备配置与驱动

#### 1. 留形雷达的驱动本质
留形（Odin1）不同于传统雷达只发原始点云。**它是一台计算前置的设备**：
* 传感器硬件内部内置了计算单元，在端侧运行紧耦合激光/视觉/惯导 LIO。
* 主机（电脑）通过 USB 3.0 发送控制指令；传感器向主机回传解算完毕的 **400Hz 高频里程计（`/odin1/odometry_highfreq`）** 和 **已对齐到全局坐标系（`odom`）的稠密 SLAM 点云（`/odin1/cloud_slam`）**。

#### 2. 核心配置文件：[`config/control_command.yaml`](../src/odin_ros_driver/config/control_command.yaml)
驱动启动时由 [`YamlParser`](../src/odin_ros_driver/src/yaml_parser.cpp) 解析该文件：
* `strict_usb3.0_check: 1`：强制要求 USB 3.0 连接，避免 USB 2.0 带宽不足丢帧。
* `use_host_ros_time: 2`：将传感器内部时间轴与主机 ROS 时间精密对齐。
* `sendcloudslam: 1` / `sendodom: 1` / `sendimu: 1`：保留导航与建图必需的 3 个核心流。
* `sendrgb: 0` / `senddtof: 0` / `senddepth: 0`：关闭 RGB 图像与密集深度补全算法，将主机 CPU 与 USB 占用降到最低。

---

## 驱动、点云与界面

#### 模块 1：驱动生命周期与数据分发 —— [`host_sdk_sample.cpp`](../src/odin_ros_driver/src/host_sdk_sample.cpp)

留形官方 SDK 为 C 接口（`lidar_api.h`），驱动通过状态机和回调方式与底层硬件交互。

##### 1.1 主入口与系统初始化 (`main` 函数，约 2240~2430 行)
```cpp
// 1. 读取 YAML 配置文件
g_parser = std::make_shared<odin_ros_driver::YamlParser>(config_file);
if (!g_parser->loadConfig()) {
    RCLCPP_ERROR(node->get_logger(), "Failed to load config file: %s", config_file.c_str());
    return -1;
}

// 2. 将配置键值赋给全局开关变量
g_sendrgb       = get_key_value("sendrgb", 1);
g_sendimu       = get_key_value("sendimu", 1);
g_sendodom      = get_key_value("sendodom", 1);
g_sendcloudslam = get_key_value("sendcloudslam", 0);

// 3. 注册 SDK 系统的设备热插拔回调，底层开始监控 USB
if (lidar_system_init(lidar_device_callback)) {
    RCLCPP_ERROR(node->get_logger(), "Lidar system init failed");
    return -1;
}
```
* **逐句解析**：
  * `g_parser->loadConfig()`：解析 YAML，将开关参数存入内存。
  * `get_key_value(...)`：读取控制项，决定后续是否向 ROS 话题发布 RGB、IMU 或点云。
  * `lidar_system_init(lidar_device_callback)`：**留形 SDK 入口**。传入函数指针 `lidar_device_callback`，启动后台 USB 事件监听线程。

---

##### 1.2 设备连接与工作流建立 (`lidar_device_callback` 函数，约 1270~1880 行)
当硬件插入 USB 时，SDK 后台线程触发此函数：
```cpp
static void lidar_device_callback(const lidar_device_info_t* device, bool attach) {
    int type = LIDAR_MODE_SLAM;  // 设置目标工作模式为 SLAM

    if (attach == true) {        // 硬件插入事件
        // 1. 严格检查是否插入了 USB 3.0 接口
        if (!isUsb3OrHigher(TARGET_VENDOR, TARGET_PRODUCT)) {
            RCLCPP_FATAL(..., "Device connected to USB 2.0 port. Requires USB 3.0 or higher.");
            exit(1);
        }

        // 2. 根据检测到的硬件信息，在 SDK 内部创建设备句柄
        if (lidar_create_device(const_cast<lidar_device_info_t*>(device), &odinDevice)) {
            RCLCPP_ERROR(..., "Create device failed");
            return;
        }

        // 3. 建立软件层连接
        if (lidar_open_device(odinDevice)) {
            RCLCPP_ERROR(..., "Open device failed");
            lidar_destory_device(odinDevice);
            return;
        }

        // 4. 从设备端固件自动下载外参/相机标定文件到本地 ~/.ros/odin_ros_driver/calib.yaml
        if (lidar_get_calib_file(odinDevice, calib_runtime_dir.c_str())) {
            RCLCPP_ERROR(..., "Failed to get calibration file");
            return;
        }

        // 5. 将传感器设为 SLAM 空间定位模式
        if (lidar_set_mode(odinDevice, type)) {
            RCLCPP_ERROR(..., "Set mode failed");
            return;
        }

        // 6. 注册数据到达回调函数
        lidar_data_callback_info_t data_callback_info;
        data_callback_info.data_callback = lidar_data_callback; // 核心数据回调
        data_callback_info.user_data = &odinDevice;
        lidar_register_stream_callback(odinDevice, data_callback_info);

        // 7. 正式向雷达下发启动数据流指令
        if (lidar_start_stream(odinDevice, type, dtof_subframe_odr)) {
            RCLCPP_ERROR(..., "Start stream failed");
            return;
        }
    }
}
```
* **逐句解析**：
  * `isUsb3OrHigher`：读取系统 sysfs 节点确认端口速率。USB 2.0 带宽不足以支撑点云传输，直接报错拦截。
  * `lidar_create_device` / `lidar_open_device`：初始化设备通讯协议栈。
  * `lidar_get_calib_file`：模组出厂时相机和雷达内参写在自身 Flash 中，驱动启动时自动下载同步，无需手动配置。
  * `lidar_set_mode(..., LIDAR_MODE_SLAM)`：通知留形模组固件启用内部 LIO 算法引擎。
  * `lidar_register_stream_callback`：注册数据接收函数。
  * `lidar_start_stream`：完成握手，传感器开始源源不断向电脑推送数据包。

---

##### 1.3 数据到达与 ROS 2 发布 (`lidar_data_callback` 函数，约 970~1045 行)
```cpp
static void lidar_data_callback(const lidar_data_t *data, void *user_data) {
    if (!deviceConnected || !g_ros_object) return;

    switch(data->type) {
        case LIDAR_DT_RAW_IMU:  // 400Hz 原始 IMU 数据
            if (g_sendimu) {
                imudata = (imu_convert_data_t *)data->stream.imageList[0].pAddr;
                // 放入专用线程安全队列，独立平滑发布，防止阻塞数据回调线程
                std::lock_guard<std::mutex> lock(g_imu_queue_mutex);
                g_imu_queue.push(*imudata);
                g_imu_queue_cv.notify_one();
            }
            break;

        case LIDAR_DT_SLAM_CLOUD:  // 稠密 SLAM 点云 (已位姿对齐)
            if (g_sendcloudslam) {
                // 转换为 sensor_msgs::msg::PointCloud2 发布至 /odin1/cloud_slam
                g_ros_object->publishPC2XYZRGBA((capture_Image_List_t *)&data->stream, 0);
            }
            break;

        case LIDAR_DT_SLAM_ODOMETRY:  // 高频里程计
            if (g_sendodom) {
                // 转换为 nav_msgs::msg::Odometry 发布至 /odin1/odometry_highfreq
                g_ros_object->publishOdometry((capture_Image_List_t *)&data->stream, ...);
            }
            break;
    }
}
```
* **逐句解析**：
  * SDK 会在独立内部线程中触发 `lidar_data_callback`。
  * 根据 `data->type` 分流：IMU 数据入队；稠密点云由 `publishPC2XYZRGBA` 转换为 ROS 2 `PointCloud2` 发布；里程计转换为 `Odometry` 发布。

---

#### 模块 2：轻量建图与拼接 —— [`odin_slam_node.cpp`](../src/odin_slam/src/odin_slam_node.cpp)

建图节点负责订阅留形模组发布的已对齐点云，完成累加拼图、降采样与落盘保存。

##### 2.1 构造函数与订阅配置 (约 11~34 行)
```cpp
OdinSlamNode::OdinSlamNode(const std::string &node) : rclcpp::Node(node) {
    // 订阅留形输出的 SLAM 点云，采用 SensorDataQoS（Best Effort 避免队列堆积）
    m_cloud_sub = create_subscription<sensor_msgs::msg::PointCloud2>(
        "/odin1/cloud_slam", rclcpp::SensorDataQoS(),
        std::bind(&OdinSlamNode::on_receive_cloud, this, std::placeholders::_1)
    );

    // 声明建图参数：保存路径、空间裁剪范围、体素降采样大小、离群点滤波阈值
    this->declare_parameter("save_pcd_path", std::string("/tmp/odin_slam_result.pcd"));
    this->declare_parameter("cropbox.min_x", -10.0);
    // ... cropbox 各轴参数
    this->declare_parameter("voxel.leaf_size", 0.1);
    this->declare_parameter("sor.mean_k", 50);
    this->declare_parameter("sor.stddev_mul_thresh", 1.0);

    // 初始化全局累加点云容器
    m_acc_cloud = std::make_shared<PointCloud>();
    m_acc_cloud->width = 0;
    m_acc_cloud->height = 1;
    m_acc_cloud->is_dense = true;
}
```
* **为什么不需要在这里做 TF 坐标转换？**
  留形模组发出的 `/odin1/cloud_slam` 的 `header.frame_id` 已经是 `odom`。传感器内部已将当前帧根据瞬时位姿投射到了世界坐标系中，因此节点只需在全局系中做点云累加即可。

---

##### 2.2 实时点云累加回调 (`on_receive_cloud`，约 121~142 行)
```cpp
void OdinSlamNode::on_receive_cloud(const sensor_msgs::msg::PointCloud2::ConstSharedPtr &msg) {
    if (m_acc_flag) { // 用户在界面点击【运行 SLAM】后置为 true
        // 1. 将 ROS 2 点云消息转换为 PCL 点云格式
        PointCloudPtr frame = std::make_shared<PointCloud>();
        pcl::fromROSMsg(*msg, *frame);

        // 2. 剔除无效点（NaN / Inf 异常距离测量点）
        PointCloudPtr no_nan_cloud = std::make_shared<PointCloud>();
        pcl::Indices indices;
        pcl::removeNaNFromPointCloud(*frame, *no_nan_cloud, indices);

        // 3. 对当前单帧进行体素滤波（降采样，避免数据量过大导致内存暴增）
        double leaf_size = this->get_parameter("voxel.leaf_size").as_double();
        PointCloudPtr filtered_frame = std::make_shared<PointCloud>();
        pcl::VoxelGrid<pcl::PointXYZ> voxel_grid;
        voxel_grid.setLeafSize(leaf_size, leaf_size, leaf_size);
        voxel_grid.setInputCloud(no_nan_cloud);
        voxel_grid.filter(*filtered_frame);

        // 4. 将处理后的单帧点云直接累加进全局地图点云
        if (!filtered_frame->empty())
            *m_acc_cloud += *filtered_frame;
    }
}
```
* **逐句解析**：
  * `pcl::fromROSMsg`：从 ROS 字节流反序列化为 PCL 点云对象。
  * `removeNaNFromPointCloud`：防止盲区近距离未测出距离的点破坏后续滤波。
  * `VoxelGrid`：以小体素栅格（如 2cm）合并密集点，极大减少累积点数。
  * `*m_acc_cloud += *filtered_frame`：PCL 重载了 `+=` 运算符，将点追加到全局地图点云中。

---

##### 2.3 最终地图后处理与保存 (`save_result`，约 50~119 行)
用户点击【保存结果】后触发：
```cpp
void OdinSlamNode::save_result() {
    if (m_acc_cloud->empty()) return;

    // 步骤 1：空间边界裁剪 (CropBox 滤波)
    // 限制地图的最大 XYZ 范围，剔除远端不可靠的噪点与天空点
    PointCloudPtr cloud_crop = std::make_shared<PointCloud>();
    pcl::CropBox<pcl::PointXYZ> crop_box;
    crop_box.setMin(Eigen::Vector4f(min_x, min_y, min_z, 1.0));
    crop_box.setMax(Eigen::Vector4f(max_x, max_y, max_z, 1.0));
    crop_box.setInputCloud(m_acc_cloud);
    crop_box.filter(*cloud_crop);

    // 步骤 2：全局体素网格滤波 (VoxelGrid)
    // 对拼接重叠部位的点做二次空间均匀化，保证地图各区域点云密度一致
    PointCloudPtr cloud_voxel = std::make_shared<PointCloud>();
    pcl::VoxelGrid<pcl::PointXYZ> voxel_grid;
    voxel_grid.setLeafSize(leaf_size, leaf_size, leaf_size);
    voxel_grid.setInputCloud(cloud_crop);
    voxel_grid.filter(*cloud_voxel);

    // 步骤 3：统计学离群点滤波 (StatisticalOutlierRemoval)
    // 计算每个点到邻域 K 个点的平均距离，剔除漂浮在空中的离群噪点
    PointCloudPtr cloud_sor = std::make_shared<PointCloud>();
    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setMeanK(mean_k);                       // 考察 50 个邻居
    sor.setStddevMulThresh(stddev);              // 偏离标准差倍数阈值
    sor.setInputCloud(cloud_voxel);
    sor.filter(*cloud_sor);

    // 步骤 4：落盘保存为 PCD 文件
    std::filesystem::path pcd_path = this->get_parameter("save_pcd_path").as_string();
    std::filesystem::create_directories(pcd_path); // 确保目录存在
    pcd_path /= std::to_string(m_save_counter) + ".pcd";
    
    if (pcl::io::savePCDFileBinary(pcd_path, *cloud_sor) == 0) {
        RCLCPP_INFO(..., "Successfully saved filtered point cloud to: %s", pcd_path.c_str());
        m_save_counter++;
        m_acc_cloud->clear(); // 清空，便于下一次建图
    }
}
```
* **逐句解析**：
  * 保存流程经过了 **CropBox 裁剪 -> VoxelGrid 网格均匀化 -> SOR 统计学去噪** 三重滤波。
  * `savePCDFileBinary`：使用二进制压缩格式存储 PCD，读取速度快且体积更小。

---

#### 模块 3：Qt 界面与 ROS 2 多线程交互 —— [`ros_worker.cpp`](../src/odin_slam/src/ros_worker.cpp)

很多 ROS 2 初学者在写 GUI 节点时容易把 `rclcpp::spin()` 放在主线程，导致界面卡死。
这里的写法非常标准：
```cpp
void RosWorker::run() {
    rclcpp::init(m_argc, m_argv, ...);
    m_record_client = std::make_shared<OdinSlamNode>();

    // 创建单线程执行器
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(m_record_client);

    QEventLoop loop;
    while (m_run_flag) {
        executor.spin_some(); // 处理 ROS 2 回调
        loop.processEvents(); // 让出 CPU，处理 Qt 线程内事件
    }
}
```
* `ControlDialog`（Qt 界面）运行在主线程。
* `RosWorker` 继承自 `QThread`，独立运行在后台处理 ROS 消息。
* 界面按钮的点击通过 **Qt 信号/槽机制（Signal & Slot）** 通知 `OdinSlamNode`，确保了界面交互流畅且点云处理不丢帧。

---

## 采图流程小结

1. **核心驱动链**：`USB 3.0 -> lydHostApi 固件通信 -> control_command.yaml 裁剪 -> host_sdk_sample -> 发布 /odin1/cloud_slam & /odin1/odometry_highfreq`。
2. **建图链**：`Qt 界面按键 -> 触发 OdinSlamNode 累加 -> PCL 降采样与去噪 -> 导出 project_slam/maps/0.pcd`。
3. **环境已就绪**：现在你可以直接插入留形模组，在 VS Code 中点击代码符号跳转阅读，并按照上一轮的运行命令进行实机测试！