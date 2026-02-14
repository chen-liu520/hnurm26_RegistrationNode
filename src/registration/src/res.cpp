#include "res.hpp"

#include <iostream>

int a = 0;

namespace hnurm
{
    RelocaliztionNode::RelocaliztionNode(const rclcpp::NodeOptions &options)
        : Node("RelocaliztionNode", options)
    {
        RCLCPP_INFO(get_logger(), "RelocaliztionNode is running");
        /***************************************参数声明和获取 start******************************************/
        pointcloud_sub_topic_ = this->declare_parameter("pointcloud_sub_topic", "/cloud_registered");
        // cloud_registered来自fast_lio2，订阅话题为/cloud_registered，是第一帧雷达数据的imu的位姿，他是固定不变的，所以在TF中他就是odom坐标系
        // 内容：
        vision_topic_ = this->declare_parameter("vision_topic", "/vision_recv_data");

        pcd_file_ = this->declare_parameter("pcd_file", "/home/rm/nav/src/hnunavigation_-ros2/hnurm_perception/PCD/all_raw_points.pcd");
        generate_downsampled_pcd_ = this->declare_parameter("generate_downsampled_pcd", false);
        downsampled_pcd_file_ = this->declare_parameter("downsampled_pcd_file", "/home/rm/nav/src/hnunavigation_-ros2/hnurm_perception/PCD/all_raw_points_downsampled.pcd");

        // small_gicp 参数
        num_threads_ = this->declare_parameter("num_threads", 4);
        num_neighbors_ = this->declare_parameter("num_neighbors", 20);
        max_dist_sq_ = this->declare_parameter("max_dist_sq", 1.0);
        source_voxel_size_ = this->declare_parameter("source_voxel_size", 0.25);
        map_voxel_size_ = this->declare_parameter("map_voxel_size_", 0.25);

        // initial pose
        this->declare_parameter("self_blue.initial_pose", std::vector<double>{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0});
        this->declare_parameter("self_red.initial_pose", std::vector<double>{0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 1.0});
        auto blue_pose = get_parameter("self_blue.initial_pose").as_double_array();
        auto red_pose = get_parameter("self_red.initial_pose").as_double_array();
        [[maybe_unused]] geometry_msgs::msg::Transform self_blue_trans_ = parsePose(blue_pose); // convert to Transform
        [[maybe_unused]] geometry_msgs::msg::Transform self_red_trans_ = parsePose(red_pose);
        // [[maybe_unused]] 是 C++17 引入的属性（attribute），用于告诉编译器：这个变量可能不会被使用，不要报警告。

        // method selector
        use_fixed_ = this->declare_parameter("use_fixed", false);
        use_quatro_ = this->declare_parameter("use_quatro", false);

        // quatro++ 参数
        m_rotation_max_iter_ = this->declare_parameter("m_rotation_max_iter", 100);
        m_num_max_corres_ = this->declare_parameter("m_num_max_corres", 200);
        m_normal_radius_ = this->declare_parameter("m_normal_radius", 0.02);
        m_fpfh_radius_ = this->declare_parameter("m_fpfh_radius", 0.04);
        m_distance_threshold_ = this->declare_parameter("m_distance_threshold", 30.0);
        m_noise_bound_ = this->declare_parameter("m_noise_bound", 0.25);
        m_rotation_gnc_factor_ = this->declare_parameter("m_rotation_gnc_factor", 1.39);
        m_rotation_cost_thr_ = this->declare_parameter("m_rotation_cost_thr", 0.0001);
        m_estimate_scale_ = this->declare_parameter("m_estimate_scale", false);
        m_use_optimized_matching_ = this->declare_parameter("m_use_optimized_matching", true);
        /***************************************参数声明和获取 end******************************************/

        if (!use_quatro_)
        {
            // 如果不用 quatro++，则使用 small_gicp，显示gicp参数
            RCLCPP_INFO(get_logger(), "use small gicp mode,reading params......");
            RCLCPP_INFO(get_logger(), "get params: pointcloud_sub_topic =%s", pointcloud_sub_topic_.c_str());
            RCLCPP_INFO(get_logger(), "get params: pcd_file =%s", pcd_file_.c_str());
            RCLCPP_INFO(get_logger(), "get params: num_threads =%d", num_threads_);
            RCLCPP_INFO(get_logger(), "get params: num_neighbors =%d", num_neighbors_);
            RCLCPP_INFO(get_logger(), "get params: max_dist_sq =%f", max_dist_sq_);
            RCLCPP_INFO(get_logger(), "get params: source_voxel_size =%f", source_voxel_size_);
            RCLCPP_INFO(get_logger(), "get params: map_voxel_size =%f", map_voxel_size_);
        }
        else
        {
            // 如果用 quatro++，则显示 quatro++ 参数
            RCLCPP_INFO(get_logger(), "use quatro mode,reading params......");
            RCLCPP_INFO(get_logger(), "get params: m_rotation_max_iter =%d", m_rotation_max_iter_);
            RCLCPP_INFO(get_logger(), "get params: m_num_max_corres =%d", m_num_max_corres_);
            RCLCPP_INFO(get_logger(), "get params: m_normal_radius =%f", m_normal_radius_);
            RCLCPP_INFO(get_logger(), "get params: m_fpfh_radius =%f", m_fpfh_radius_);
            // RCLCPP_INFO(get_logger(), "get params: m_distance_threshold =%f", m_distance_threshold_);
            RCLCPP_INFO(get_logger(), "get params: m_noise_bound =%f", m_noise_bound_);
            RCLCPP_INFO(get_logger(), "get params: m_rotation_gnc_factor =%f", m_rotation_gnc_factor_);
            RCLCPP_INFO(get_logger(), "get params: m_rotation_cost_thr =%f", m_rotation_cost_thr_);
            if (m_estimate_scale_)
                RCLCPP_INFO(get_logger(), "get params: m_estimate_scale = true");
            else
                RCLCPP_INFO(get_logger(), "get params: m_estimate_scale = false");
            if (m_use_optimized_matching_)
                RCLCPP_INFO(get_logger(), "get params: m_use_optimized_matching = true");
            else
                RCLCPP_INFO(get_logger(), "get params: m_use_optimized_matching = false");
        }
        // quatro++ 处理模块初始化，传入quatro++ 参数
        /*
        m_quatro_handler = std::make_shared<quatro<QuatroPointType>>(m_normal_radius_,
                                                                     m_fpfh_radius_, m_noise_bound_, m_rotation_gnc_factor_, m_rotation_cost_thr_,
                                                                     m_rotation_max_iter_, m_estimate_scale_, m_use_optimized_matching_,
                                                                     m_distance_threshold_, m_num_max_corres_);*/

        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
        tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(*this);

        // init pointcloud pointer，.reset（）等价于td::make_shared
        // reset:.reset() 会先释放旧对象（引用计数-1，如果为0则销毁）,然后接管新创建的对象。循环复用（重置状态），之后会清空上一帧点云
        source_cloud_.reset(new pcl::PointCloud<pcl::PointXYZ>);
        accumulate_cloud_.reset(new pcl::PointCloud<pcl::PointXYZ>);
        source_cloud_downsampled_.reset(new pcl::PointCloud<pcl::PointXYZ>);
        global_map_downsampled_.reset(new pcl::PointCloud<pcl::PointXYZ>);
        global_map_.reset(new pcl::PointCloud<pcl::PointXYZ>);
        global_map_PointCovariance_.reset(new pcl::PointCloud<pcl::PointCovariance>);

        // load pcd
        load_pcd_map(pcd_file_);
        // 配置 ROS2 的 QoS（Quality of Service，服务质量）策略
        // 队列保留 最新的 10 条消息，旧消息会被丢弃（类似于缓冲区大小）
        rclcpp::QoS qos_profile(rclcpp::KeepLast(10));
        // 设置 可靠传输模式。确保消息必达，类似 TCP（会重传丢失的消息）
        qos_profile.reliable();
        // 适用于：点云数据 — 不能丢帧，否则地图不完整。定位结果 — 位姿信息必须可靠传输

        // subscribers
        pointcloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            pointcloud_sub_topic_,
            rclcpp::SensorDataQoS(),
            std::bind(&RelocaliztionNode::pointcloud_sub_callback, this, std::placeholders::_1));
        /*
        recv_sub_ = this->create_subscription<hnurm_interfaces::msg::VisionRecvData>(
            vision_topic_,
            rclcpp::SensorDataQoS(),
            std::bind(&RelocaliztionNode::recv_sub_callback, this, std::placeholders::_1));*/

        init_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "/initialpose",
            rclcpp::SensorDataQoS(),
            std::bind(&RelocaliztionNode::initial_pose_callback, this, std::placeholders::_1));

        // publishers
        // 发布配准点云
        pointcloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/global_pcd_map", 10);
        timer_ = this->create_wall_timer(std::chrono::milliseconds(2000), std::bind(&RelocaliztionNode::timer_callback, this));
        timer_pub_tf_ = this->create_wall_timer(std::chrono::milliseconds(10), std::bind(&RelocaliztionNode::timer_pub_tf_callback, this));
    }

    // void RelocaliztionNode::recv_sub_callback(const hnurm_interfaces::msg::VisionRecvData::SharedPtr msg)；
    //  ROS2 回调函数，处理视觉模块发来的数据，主要功能是 根据己方颜色选择初始位姿并完成初始化。
    /*
    void RelocaliztionNode::recv_sub_callback(const hnurm_interfaces::msg::VisionRecvData::SharedPtr msg)
    {
        
        if (!is_initialized_)
        {
            std::string self_color;
            geometry_msgs::msg::Transform initial_pose_;
            if (msg->self_color.data == 1)
            {
                self_color = "RED";
                initial_guess_ = tf2::transformToEigen(self_red_trans_);
                initial_pose_ = self_red_trans_;
            }
            else
            {
                self_color = "BLUE";
                initial_guess_ = tf2::transformToEigen(self_blue_trans_);
                initial_pose_ = self_blue_trans_;
            }
            if (!getInitialPose_)
            {
                RCLCPP_INFO(get_logger(), "Current Self Color: %s ,Received initial pose:", self_color.c_str());
                RCLCPP_INFO(get_logger(), "  Position: [%.2f, %.2f, %.2f]",
                            initial_pose_.translation.x,
                            initial_pose_.translation.y,
                            initial_pose_.translation.z);
                RCLCPP_INFO(get_logger(), "  Orientation: [%.2f, %.2f, %.2f, %.2f]",
                            initial_pose_.rotation.x,
                            initial_pose_.rotation.y,
                            initial_pose_.rotation.z,
                            initial_pose_.rotation.w);
            }
            getInitialPose_ = true;
            is_initialized_ = true;
        }
    }
    */

    // 接受rviz鼠标点击初始位姿估计
    void RelocaliztionNode::initial_pose_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
    {
        if (use_fixed_ && get_first_tf_from_quatro_) // enable reset initial pose // 是否得到quatro++的初始配准
        // 允许通过 RViz 修正位姿
        // 只有 Quatro 完成后才能手动修正
        {
            // 提取位姿信息
            geometry_msgs::msg::Transform trans_;
            trans_.translation.x = msg->pose.pose.position.x;
            trans_.translation.y = msg->pose.pose.position.y;
            trans_.translation.z = msg->pose.pose.position.z;
            trans_.rotation.w = msg->pose.pose.orientation.w;
            trans_.rotation.x = msg->pose.pose.orientation.x;
            trans_.rotation.y = msg->pose.pose.orientation.y;
            trans_.rotation.z = msg->pose.pose.orientation.z;
            // 转换为 Eigen 格式
            initial_guess_ = tf2::transformToEigen(trans_);
            // guesses_ = generate_initial_guesses(initial_guess_,0.5,M_PI/6);

            getInitialPose_ = true;
            RCLCPP_INFO(get_logger(), "Received initial pose:");
            RCLCPP_INFO(get_logger(), "  Position: [%.2f, %.2f, %.2f]",
                        msg->pose.pose.position.x,
                        msg->pose.pose.position.y,
                        msg->pose.pose.position.z);
            RCLCPP_INFO(get_logger(), "  Orientation: [%.2f, %.2f, %.2f, %.2f]",
                        msg->pose.pose.orientation.x,
                        msg->pose.pose.orientation.y,
                        msg->pose.pose.orientation.z,
                        msg->pose.pose.orientation.w);
        }
        else  // 提示等待 Quatro 计算
        {
            RCLCPP_INFO(get_logger(), "waiting for quatro++ calculation");
            getInitialPose_ = true; // 有bug
        }
    }

    // 这个函数用于生成多个扰动的初始位姿猜测，实现多起点优化（Multi-start Optimization），提高配准成功率。
    /*
    params：基准位姿（如 Quatro 粗配准结果）、 位置扰动范围（米）、位置扰动范围（米）
    调用时机：Quatro 粗配准后、GICP 精配准前
    */
    std::vector<Eigen::Isometry3d> RelocaliztionNode::generate_initial_guesses(const Eigen::Isometry3d &initial_pose, double trans_noise, double rot_noise)
    {
        std::vector<Eigen::Isometry3d> guesses;
        for (int i = 0; i < 10; ++i)
        { // generate 10 guess
            // 随机平移扰动，[-trans_noise, +trans_noise]
            Eigen::Vector3d trans = Eigen::Vector3d::Random() * trans_noise; // Eigen::Vector3d::Random() 生成 [-1, 1] 范围内均匀分布的随机数
            //  随机旋转扰动。rot_axis：随机单位向量作为旋转轴
            Eigen::Vector3d rot_axis = Eigen::Vector3d::Random().normalized();              // 随机旋转轴
            Eigen::AngleAxisd rot(rot_noise * Eigen::internal::random(0.0, 1.0), rot_axis); // 绕该轴旋转 [0, rot_noise] 范围内的随机角度

            // 合成扰动位姿
            Eigen::Isometry3d perturbed = initial_pose * Eigen::Translation3d(trans) * rot;
            guesses.emplace_back(perturbed);
        }
        return guesses;
    }

    void RelocaliztionNode::reset()
    {
        pre_result_ = Eigen::Isometry3d::Identity();
        getInitialPose_ = false;
        doFirstRegistration_ = false;
        // RCLCPP_INFO(get_logger(), "receive new initial pose ,reset");
    }

    void RelocaliztionNode::load_pcd_map(const std::string &map_path)
    {
        if (generate_downsampled_pcd_)
        {
            // 加载原始点云地图到 global_map_
            if (pcl::io::loadPCDFile<pcl::PointXYZ>(map_path, *global_map_) == -1)
            {
                RCLCPP_ERROR(get_logger(), "Failed to load PCD map: %s", map_path.c_str());
                return;
            }
            RCLCPP_INFO(get_logger(), "Loaded PCD map with %ld points", global_map_->size());

            // downsampling：体素下采样（减少点数）
            global_map_downsampled_ = voxelgrid_sampling_omp(*global_map_, map_voxel_size_);
            // save downsampled pcd：保存下采样后的点云地图到文件
            if (pcl::io::savePCDFileASCII(downsampled_pcd_file_, *global_map_downsampled_) == -1)
            {
                RCLCPP_ERROR(
                    get_logger(),
                    "Failed to save downsampled PCD map: %s",
                    downsampled_pcd_file_.c_str());
            }
            else
            {
                RCLCPP_INFO(
                    get_logger(),
                    "Successfully saved downsampled map to: %s",
                    downsampled_pcd_file_.c_str());
            }
            return;
        }
        // global_map_.reset(new pcl::PointCloud<pcl::PointXYZ>);
        // 1. 加载下采样点云
        if (pcl::io::loadPCDFile<pcl::PointXYZ>(downsampled_pcd_file_, *global_map_downsampled_) == -1)
        {
            RCLCPP_ERROR(get_logger(), "Failed to load PCD map: %s", downsampled_pcd_file_.c_str());
            return;
        }
        RCLCPP_INFO(get_logger(), "Loaded PCD map with %ld points", global_map_downsampled_->size());
        // //downsampling

        // global_map_downsampled_ = voxelgrid_sampling_omp(*global_map_,map_voxel_size_);

        auto t_start = std::chrono::steady_clock::now();

        // 2. 计算点云协方差（GICP 配准需要）
        global_map_PointCovariance_ = voxelgrid_sampling_omp<pcl::PointCloud<pcl::PointXYZ>, pcl::PointCloud<pcl::PointCovariance>>(*global_map_downsampled_, map_voxel_size_); // do not do downsample
        estimate_covariances_omp(*global_map_PointCovariance_, num_neighbors_, num_threads_);

        // build target kd_tree
        // 3. 构建 KD-Tree（加速最近邻搜索）
        target_tree_ = std::make_shared<KdTree<pcl::PointCloud<pcl::PointCovariance>>>(global_map_PointCovariance_, KdTreeBuilderOMP(num_threads_));

        // 4. 计算下采样运行时间差
        auto t_end = std::chrono::steady_clock::now();
        auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(t_end - t_start).count();
        RCLCPP_INFO(get_logger(), "VoxelGrid downsampling took %ld ms", elapsed_ms);

        RCLCPP_INFO(this->get_logger(), "Downsampled PCD map to %ld points", global_map_PointCovariance_->size());

        // 5. 可视化输出
        sensor_msgs::msg::PointCloud2 output_cloud;
        pcl::toROSMsg(*global_map_downsampled_, output_cloud);
        output_cloud.header.frame_id = "map";
        output_cloud.header.stamp = this->now();
        cloud_ = std::make_shared<sensor_msgs::msg::PointCloud2>(output_cloud);
    }

    void RelocaliztionNode::timer_pub_tf_callback()
    {
        if (cloud_) // test reading pcd
        {
            cloud_->header.stamp = this->now();
            pointcloud_pub_->publish(*cloud_); // publish downsampled pcd map，给rviz显示用的
            if (use_fixed_) // use tf guess form quatro , small_gicp do global relocalization
            {
                // RCLCPP_INFO(this->get_logger(),"test use fixed method");
                if (get_first_tf_from_quatro_)
                {
                    // source_cloud_PointCovariance_ = voxelgrid_sampling_omp<pcl::PointCloud<pcl::PointXYZ>, pcl::PointCloud<pcl::PointCovariance>>(*source_cloud_, source_voxel_size_);
                    // RCLCPP_INFO(this->get_logger(),"publish pcd map");
                    if (source_cloud_PointCovariance_)
                    {
                        // relocalization();
                        transform.header.frame_id = "map";
                        transform.header.stamp = this->now();
                        tf_broadcaster_->sendTransform(transform);
                    }
                }
            }
            else if (!use_quatro_)
            {
                // source_cloud_PointCovariance_ = voxelgrid_sampling_omp<pcl::PointCloud<pcl::PointXYZ>, pcl::PointCloud<pcl::PointCovariance>>(*source_cloud_, source_voxel_size_);
                // RCLCPP_INFO(this->get_logger(),"publish pcd map");
                if (source_cloud_PointCovariance_)
                {
                    // relocalization();
                    transform.header.frame_id = "map";
                    transform.header.stamp = this->now();
                    tf_broadcaster_->sendTransform(transform);
                }
            }
        }
    }

    void RelocaliztionNode::timer_callback()
    {
        if (cloud_) // test reading pcd
        {
            cloud_->header.stamp = this->now();
            // pointcloud_pub_->publish(*cloud_);
            if (use_fixed_) // use tf guess form quatro , small_gicp do global relocalization
            {
                // RCLCPP_INFO(this->get_logger(),"test use fixed method");
                if (get_first_tf_from_quatro_)
                {

                    source_cloud_PointCovariance_ = voxelgrid_sampling_omp<pcl::PointCloud<pcl::PointXYZ>, pcl::PointCloud<pcl::PointCovariance>>(*source_cloud_, source_voxel_size_);
                    // RCLCPP_INFO(this->get_logger(),"publish pcd map");
                    if (source_cloud_PointCovariance_)
                    {
                        relocalization();
                        transform.header.frame_id = "map";
                        transform.header.stamp = this->now();
                        tf_broadcaster_->sendTransform(transform);
                    }
                }
            }
            else if (!use_quatro_)
            {
                source_cloud_PointCovariance_ = voxelgrid_sampling_omp<pcl::PointCloud<pcl::PointXYZ>, pcl::PointCloud<pcl::PointCovariance>>(*source_cloud_, source_voxel_size_);
                // RCLCPP_INFO(this->get_logger(),"publish pcd map");
                if (source_cloud_PointCovariance_)
                {
                    relocalization();
                    transform.header.frame_id = "map";
                    transform.header.stamp = this->now();
                    tf_broadcaster_->sendTransform(transform);
                }
            }
        }
    }

    void RelocaliztionNode::pointcloud_sub_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        // return;
        if (!getInitialPose_)
        {
            RCLCPP_WARN_THROTTLE(
                get_logger(),
                *get_clock(),
                1000, // 1 second
                "Waiting for initial pose...");
            return;
        }
        if (use_fixed_) // 只要开启固定位姿
        {
            // RCLCPP_INFO(this->get_logger(),"test use fixed method");
            // 第一次积累三帧
            if (accumulation_counter_ <= 2 && !get_first_tf_from_quatro_)
            {
                pcl::fromROSMsg(*msg, *accumulate_cloud_);

                *source_cloud_ += *accumulate_cloud_;

                accumulation_counter_++;
                if (accumulation_counter_ > 2)
                {
                    // 下采样
                    source_cloud_downsampled_ = voxelgrid_sampling_omp(*source_cloud_, source_voxel_size_);
                    bool if_valid_;
                    // Quatro 配准
                    Eigen::Matrix4d output_tf_ = m_quatro_handler->align(*source_cloud_downsampled_, *global_map_downsampled_, if_valid_);
                    if (if_valid_ && !get_first_tf_from_quatro_)
                    {
                        publishTransform(output_tf_);                    // 发布 TF
                        getInitialPose_ = true;
                        initial_guess_ = Eigen::Isometry3d(output_tf_);  // 保存为初始猜测
                        RCLCPP_INFO(this->get_logger(), "publish tf");  
                        get_first_tf_from_quatro_ = true;                // 标记已完成粗配准
                    }
                    else
                        accumulation_counter_ = 0; // 失败则重新累积
                }
            }
            pcl::fromROSMsg(*msg, *source_cloud_);

            // 正常接收单帧，下采样
            source_cloud_downsampled_ = voxelgrid_sampling_omp(*source_cloud_, source_voxel_size_);

            // bool if_valid_;
            // Eigen::Matrix4d output_tf_ = m_quatro_handler->align(*source_cloud_downsampled_, *global_map_downsampled_,if_valid_);
            // if (if_valid_&&!get_first_tf_from_quatro_)
            // {
            //   publishTransform(output_tf_);
            //   getInitialPose_ = true;
            //   initial_guess_ = Eigen::Isometry3d(output_tf_);
            //   RCLCPP_INFO(this->get_logger(),"publish tf");
            //   get_first_tf_from_quatro_ = true;
            // }
        }
        else if (!use_quatro_) // 无确定位姿、不使用quatro：纯GICP
        {
            // 正常接收单帧，写入source_cloud_
            pcl::fromROSMsg(*msg, *source_cloud_);
            // source_tree_ = std::make_shared<KdTree<pcl::PointCloud<pcl::PointCovariance>>>(source_cloud_PointCovariance_, KdTreeBuilderOMP(num_threads_));
            // relocalization(source_cloud_);
            // relocalization();
        }
        else // 纯用quatro，但是不用固定坐标
        {

            pcl::fromROSMsg(*msg, *source_cloud_);
            source_cloud_downsampled_ = voxelgrid_sampling_omp(*source_cloud_, source_voxel_size_);
            bool if_valid_;
            Eigen::Matrix4d output_tf_ = m_quatro_handler->align(*source_cloud_downsampled_, *global_map_downsampled_, if_valid_);
            if (if_valid_)
            {
                publishTransform(output_tf_);
                RCLCPP_INFO(this->get_logger(), "publish tf");
            }
            else
                RCLCPP_INFO(this->get_logger(), "invalid");
        }
    }
    

    void RelocaliztionNode::publishTransform(const Eigen::Matrix4d &transform_matrix)
    {
        geometry_msgs::msg::TransformStamped transform;

        transform.header.stamp = this->now();
        transform.header.frame_id = "map";
        transform.child_frame_id = source_cloud_->header.frame_id;

        // 提取平移部分 (4x4矩阵的最后一列)
        transform.transform.translation.x = transform_matrix(0, 3);
        transform.transform.translation.y = transform_matrix(1, 3);
        transform.transform.translation.z = transform_matrix(2, 3);

        // 提取旋转部分 (3x3左上子矩阵)
        Eigen::Matrix3d rotation = transform_matrix.block<3, 3>(0, 0);
        Eigen::Quaterniond quat(rotation);
        transform.transform.rotation.x = quat.x();
        transform.transform.rotation.y = quat.y();
        transform.transform.rotation.z = quat.z();
        transform.transform.rotation.w = quat.w();

        tf_broadcaster_->sendTransform(transform);
    }

    void RelocaliztionNode::relocalization()
    {
        if (!getInitialPose_)
        {
            RCLCPP_WARN(get_logger(), "No initial pose received. Skipping relocalization.");
            return;
        }


        // Downsample points and convert them into pcl::PointCloud<pcl::PointCovariance>.
        // pcl::PointCloud<pcl::PointCovariance>::Ptr target = voxelgrid_sampling_omp<pcl::PointCloud<pcl::PointXYZ>, pcl::PointCloud<pcl::PointCovariance>>(*raw_target, 0.25);
        // pcl::PointCloud<pcl::PointCovariance>::Ptr source = voxelgrid_sampling_omp<pcl::PointCloud<pcl::PointXYZ>, pcl::PointCloud<pcl::PointCovariance>>(*raw_source, 0.25);

        // Estimate covariances of points.
        // const int num_threads = num_threads_;
        // const int num_neighbors = num_neighbors_;
        // estimate_covariances_omp(*global_map_PointCovariance_, num_neighbors, num_threads);
        // estimate_covariances_omp(*source_cloud_PointCovariance_, num_neighbors, num_threads);

        // Create KdTree for target and source.
        // auto target_tree = std::make_shared<KdTree<pcl::PointCloud<pcl::PointCovariance>>>(global_map_PointCovariance_, KdTreeBuilderOMP(num_threads));
        // auto source_tree = std::make_shared<KdTree<pcl::PointCloud<pcl::PointCovariance>>>(source, KdTreeBuilderOMP(num_threads));

        Registration<GICPFactor, ParallelReductionOMP> registration;
        registration.reduction.num_threads = num_threads_;
        registration.rejector.max_dist_sq = max_dist_sq_;

        if (doFirstRegistration_)
        {
            result = registration.align(*global_map_PointCovariance_, *source_cloud_PointCovariance_, *target_tree_, pre_result_);
        }
        // else
        // {
        //   std::vector<RegistrationResult> results;
        //   #pragma omp parallel for num_threads(4)
        //   for(const auto& guess_:guesses_)
        //   {
        //     result = registration.align(*global_map_PointCovariance_, *source_cloud_PointCovariance_, *target_tree_, guess_);
        //     RegistrationResult rs_;
        //     rs_.converged = result.converged;
        //     rs_.error = result.error;
        //     rs_.transformation = result.T_target_source;
        //     #pragma omp critical
        //     results.emplace_back(rs_);
        //   }
        //   auto best_result = std::min_element(results.begin(), results.end(),
        //   [](const auto& a, const auto& b) { return a.error < b.error; });
        //   pre_result_=best_result->transformation;
        //   // if(best_result->error>20.0&&!doFirstRegistration_)
        //   // {
        //   //   RCLCPP_INFO(get_logger(), "cannot do first registration,reset,result error:%f",best_result->error);
        //   //   reset();
        //   //   return;
        //   // }
        //   // else pre_result_=best_result->transformation;
        // }
        else
        {
            result = registration.align(*global_map_PointCovariance_, *source_cloud_PointCovariance_, *target_tree_, initial_guess_);
            if (!result.converged && !doFirstRegistration_)
            {
                RCLCPP_INFO(get_logger(), "cannot do first registration,reset,result error:%f", result.error);
                reset();
                transform.header.frame_id = "map";
                transform.child_frame_id = source_cloud_->header.frame_id;
                transform.transform = tf2::eigenToTransform(initial_guess_).transform;
                return;
            }
        }

        // publish map->odom tf
        if (result.converged)
        {
            // pre_result_ = result.T_target_source.inverse();
            pre_result_ = result.T_target_source;
            doFirstRegistration_ = true;
            Eigen::Isometry3d T_map_odom = pre_result_;

            // publish transform
            transform.header.stamp = this->now();
            transform.header.frame_id = "map";
            transform.child_frame_id = source_cloud_->header.frame_id;
            transform.transform = tf2::eigenToTransform(T_map_odom).transform;
            RCLCPP_INFO(get_logger(), "Published map->odom transform,result error:%f", result.error);
        }
        else
        {
            fail_counter++;
            if (fail_counter > 1)
            {
                fail_counter = 0;
                reset();
                RCLCPP_ERROR(get_logger(), "fail upto the threshold,reset,please set initalpose again");
            }

            RCLCPP_ERROR(get_logger(), "Relocalization failed to converge,result error:%f", result.error);
        }

        // 第一次结果的逆矩阵（map->odom）
        // std::cout << "--- T_map_odom ---" << std::endl << result.T_target_source.inverse().matrix() << std::endl;

        // std::cout << "--- T_target_source ---" << std::endl << result.T_target_source.matrix() << std::endl;
        // std::cout << "converged:" << result.converged << std::endl;
        // std::cout << "error:" << result.error << std::endl;
        // std::cout << "iterations:" << result.iterations << std::endl;
        // std::cout << "num_inliers:" << result.num_inliers << std::endl;
        // std::cout << "--- H ---" << std::endl << result.H << std::endl;
        // std::cout << "--- b ---" << std::endl << result.b.transpose() << std::endl;

        // // Because this usage exposes all preprocessed data, you can easily re-use them to obtain the best efficiency.
        // auto result2 = registration.align(*source, *target, *source_tree, Eigen::Isometry3d::Identity());

        // std::cout << "--- T_target_source ---" << std::endl << result2.T_target_source.inverse().matrix() << std::endl;
    }
}
