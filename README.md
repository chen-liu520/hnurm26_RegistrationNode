# 解释
## void RelocaliztionNode::initial_pose_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)；
```
┌─────────────────┐
│ 启动时 Quatro++  │◄──── 自动粗配准
│ 计算初始位姿     │
└────────┬────────┘
         │ get_first_tf_from_quatro_ = true
         ▼
┌─────────────────┐
│ 用户在 RViz 点击 │◄──── 手动修正
│ "2D Pose Estimate"│
└────────┬────────┘
         ▼
┌─────────────────┐
│ 本回调函数接收   │
│ 并更新 initial_guess_
└─────────────────┘
```
- bug:
```cpp
else
{
    RCLCPP_INFO(get_logger(), "waiting for quatro++ calculation");
    getInitialPose_ = true; // 有bug
}
```

## std::vector<Eigen::Isometry3d> RelocaliztionNode::generate_initial_guesses(const Eigen::Isometry3d &initial_pose, double trans_noise, double rot_noise)
- 功能：根据初始位姿生成多个初始猜测位姿，用于 GICP 精配准。
- 参数
    - initial_pose	Eigen::Isometry3d	基准位姿（如 Quatro 粗配准结果）
    - trans_noise	double	位置扰动范围（米）
    - rot_noise	    double	旋转扰动范围（弧度）
- 输出：10个随机扰动的位姿
- 调用时机：Quatro 粗配准后、GICP 精配准前
```
单一起点：                  多起点：
    ↓                        ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓ ↓
   GICP                     GICP×10
    ↓                        ↓
 可能陷入局部最优           选择误差最小的收敛结果
    ↓                        ↓
  定位失败                  提高成功率
```

## void RelocaliztionNode::reset()
- 重置配准节点的状态，让系统重新开始定位流程。
  
| 变量 | 重置值 | 含义 |
| --- | --- | --- |
| pre_result_ | Eigen::Isometry3d::Identity()【单位矩阵】| 清空上一次配准结果，不再有"前一帧"的位姿参考 |
| getInitialPose_ | false | 标记为未获取初始位姿，需要重新确定起点 |
| doFirstRegistration_ | false | 标记为未完成首次配准，需要重新执行第一次配准流程 |

- 系统回到"刚启动"状态 --> 等待新的初始位姿输入 → 重新进行 Quatro 粗配准 → GICP 精配准

## void RelocaliztionNode::load_pcd_map(const std::string &map_path)；
- 功能：加载全局地图的 PCD 文件。
    - 如果需要计算下采样（参数：generate_downsampled_pcd），会自动进行下采样并保存到 `downsampled_pcd_file_`。
    - 不需要的话：直接加载预下采样地图 → 协方差计算 → KD-Tree → 发布降采样点云 → 准备配准
- 带协方差的点云是降采样后计算得到的
  

## void RelocaliztionNode::timer_pub_tf_callback()
- 定时发布降采样点云地图，用于 RViz 可视化。
- transform位姿？


##  void RelocaliztionNode::timer_callback()
- 功能：定时执行配准流程
- 具体：更新下采样点云地图的时间戳，判断用不用quatro，获取当前点云，下采样➕协方差，**执行核心配准流程（relocalization()）**,发布transform


## vGICPFactor	使用 GICP（Generalized ICP）误差函数，考虑点云协方差oid RelocaliztionNode::pointcloud_sub_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
- 当前点云命名错误：
    - accumulate_cloud_：临时接收当前 msg 转换的点云，用完即弃
    - source_cloud_	真正累积多帧的点云容器，用于后续配准
- 用fixed的话，把quatro++的结果作为initial_guess_
- 这个initial_guess_作为gicp十个初始猜测位姿的输入，是多起点GICP优化


## void RelocaliztionNode::publishTransform(const Eigen::Matrix4d &transform_matrix)
- TF发布方向：
    - map → [source_cloud_->header.frame_id]：通常是：map → base_link 或 map → livox/lidar
  

## void RelocaliztionNode::relocalization()；
1. 检查是否有初始位姿
2.  GICP 配准器配置
```cpp
// GICPFactor	使用 GICP（Generalized ICP）误差函数，考虑点云协方差
// ParallelReductionOMP	OpenMP 并行加速
// max_dist_sq	距离超过此值的对应点被剔除（过滤离群点）
Registration<GICPFactor, ParallelReductionOMP> registration;
registration.reduction.num_threads = num_threads_;  // OpenMP 并行线程数
registration.rejector.max_dist_sq = max_dist_sq_;   // 对应点最大距离阈值
```
3. 是否完成首次配准
   - 是：用上一帧的配准结果作为初始猜测位姿
   - 不是（第一次配准）：用quatro++的结果作为初始猜测位姿
  

## 点云话题：/cloud_registered
- cloud_registered来自fast_lio2，订阅话题为/cloud_registered，是第一帧雷达数据的imu的位姿，他是固定不变的，所以在TF中他就是odom坐标系
- 内容：
    - header.frame_id: 代码中是“camera_init”，定义是第一帧雷达数据的imu的位姿，他是固定不变的，**所以在TF中他就是odom坐标系**
    - 点云数据：经过紧耦合 LiDAR-IMU 里程计配准 + 运动畸变校正后的点云，具备两个关键特性：
        - 无运动畸变：雷达转一圈需要100ms，这100ms里如果机器人运动了，点云的坐标就会发生变化，而fastlio2会对点云进行运动畸变校正，所以点云的坐标是没有运动畸变的
        - 坐标系：计算点云的xyz是相对于camera_init的，也就是第一帧雷达数据的imu的位姿，也就是odom
- 总结：
    - 用/cloud_registered做为配准源是非常正确的
    - 重定位节点中，所有要拿到source_cloud_的frame_id的地方，拿到的都是odom，实现的变换也就是map → odom

## 重定位主流程
```
首次定位：Quatro粗配准 → initial_guess_ → GICP精配准
                                            ↓
                ↓
正常跟踪：   pre_result_ ──→ GICP（快速收敛）
                ↑_____________│
                （结果反馈，形成闭环）
连续失败：  → reset() → 回到 initial_guess_ 模式
```


## 函数调用
1. 回调

| 函数名 | 触发条件 | 频率 | 功能 |
| --- | --- | --- | --- |
| pointcloud_sub_callback |	/cloud_registered新消息	| 10-20Hz | 接收点云，累积3帧，Quatro粗配准 |
| initial_pose_callback	|	/initialpose 新消息	| 单次	| RViz设置初始位姿，替换视觉回调 |
| recv_sub_callback | 	/vision_recv_data	| - | 根据己方颜色选择初始位姿并完成初始化。 |
| timer_callback |	2000ms | 定时0.5Hz |	GICP精配准 |
| timer_pub_tf_callback	|	10ms |定时100Hz	| 发布TF和地图 |

2. 调用关系

| 函数名 | 调用者 | 调用条件 | 功能 |
| --- | --- | --- | --- |
| load_pcd_map() |	构造函数 |	初始化时 |	加载PCD，计算协方差，构建KD-Tree |
| relocalization() | timer_callback() |	source_cloud_PointCovariance_ 非空 | GICP配准核心算法 |
| publishTransform() | pointcloud_sub_callback()	| Quatro配准成功时	| 发布map→lidar的TF |
| reset()	| relocalization() | 	配准失败2次时	| 重置定位状态 |
| generate_initial_guesses() |	未被调用 | 	被注释掉 |	生成10个扰动初始猜测 |
| parsePose() |	构造函数 |	初始化时	| 解析红蓝方初始位姿参数 |


https://github.com/chen-liu520/hnurm26_RegistrationNode.git