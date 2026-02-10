#include "registration_node.hpp"

namespace hnurm
{
    RelocationNode::RelocationNode(const rclcpp::NodeOptions &options)
        : Node("relocation_node", options)
    {

        RCLCPP_INFO(get_logger(), "RelocationNode created");

        /***********1. 参数读取***************/
        pointcloud_sub_topic_ = this->declare_parameter("pointcloud_sub_topic", "/cloud_registered");

        pcd_file_ = this->declare_parameter("pcd_file", "/home/rm/nav/src/hnunavigation_-ros2/hnurm_perception/PCD/all_raw_points.pcd");
        generate_downsampled_pcd_ = this->declare_parameter("generate_downsampled_pcd", false);
        downsampled_pcd_file_ = this->declare_parameter("downsampled_pcd_file", "/home/rm/nav/src/hnunavigation_-ros2/hnurm_perception/PCD/all_raw_points_downsampled.pcd");

        num_threads_ = this->declare_parameter("num_threads", 4);
        num_neighbors_ = this->declare_parameter("num_neighbors", 20);
        max_dist_sq_ = this->declare_parameter("max_dist_sq", 1.0);
        gicp_max_iterations_ = this->declare_parameter("gicp_max_iterations", 50);             // 最大迭代次数（默认通常是30-50）
        gicp_convergence_tolerance_ = this->declare_parameter("gicp_convergence_tolerance", 1e-4); // 收敛阈值（默认1e-4）
        tracking_frequency_divisor_ = this->declare_parameter("tracking_frequency_divisor", 1);

        gicp_voxel_size_ = this->declare_parameter("gicp_voxel_size", 0.25);
        quatro_voxel_size_ = this->declare_parameter("quatro_voxel_size", 0.25);
        map_voxel_size_ = this->declare_parameter("map_voxel_size", 0.25);
        init_accumulation_counter_ = this->declare_parameter("init_accumulation_counter", 10);  // 新添加
        track_accumulation_counter_ = this->declare_parameter("track_accumulation_counter", 3); // 新添加
        reset_accumulation_counter_ = this->declare_parameter("reset_accumulation_counter", 5); // 新添加

        // method selector
        test_cloud_registered_ = this->declare_parameter("test_cloud_registered", false);
        use_rviz_revise_ = this->declare_parameter("use_rviz_revise", false);
        use_timer_ = this->declare_parameter("use_timer", false); // 综合判断点云回调和定时器哪个在精准度和资源消耗上更优

        // use_fixed_ = this->declare_parameter("use_fixed", false);
        // use_quatro_ = this->declare_parameter("use_quatro", false);

        // quatro params
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

        /***************2. 参数打印 *************/

        RCLCPP_INFO(get_logger(), "get params: pointcloud_sub_topic =%s", pointcloud_sub_topic_.c_str());
        RCLCPP_INFO(get_logger(), "get params: pcd_file =%s", pcd_file_.c_str());
        RCLCPP_INFO(get_logger(), "get params: num_threads =%d", num_threads_);
        RCLCPP_INFO(get_logger(), "get params: num_neighbors =%d", num_neighbors_);
        RCLCPP_INFO(get_logger(), "get params: max_dist_sq =%f", max_dist_sq_);
        RCLCPP_INFO(get_logger(), "get params: gicp_voxel_size =%f", gicp_voxel_size_);
        RCLCPP_INFO(get_logger(), "get params: map_voxel_size =%f", map_voxel_size_);

        RCLCPP_INFO(get_logger(), "use quatro mode,reading params......");
        RCLCPP_INFO(get_logger(), "get params: m_rotation_max_iter =%d", m_rotation_max_iter_);
        RCLCPP_INFO(get_logger(), "get params: m_num_max_corres =%d", m_num_max_corres_);
        RCLCPP_INFO(get_logger(), "get params: m_normal_radius =%f", m_normal_radius_);
        RCLCPP_INFO(get_logger(), "get params: m_fpfh_radius =%f", m_fpfh_radius_);
        RCLCPP_INFO(get_logger(), "get params: m_distance_threshold =%f", m_distance_threshold_);
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

        // set quatro params && initialize handler
        m_quatro_handler = std::make_shared<quatro<QuatroPointType>>(m_normal_radius_,
                                                                     m_fpfh_radius_, m_noise_bound_, m_rotation_gnc_factor_, m_rotation_cost_thr_,
                                                                     m_rotation_max_iter_, m_estimate_scale_, m_use_optimized_matching_,
                                                                     m_distance_threshold_, m_num_max_corres_);

        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(get_clock());
        tf_listener_ = std::make_unique<tf2_ros::TransformListener>(*tf_buffer_);
        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(*this);

        current_accumulated_cloud_.reset(new pcl::PointCloud<pcl::PointXYZ>);
        current_cloud_.reset(new pcl::PointCloud<pcl::PointXYZ>);
        current_cloud_downsampled_.reset(new pcl::PointCloud<pcl::PointXYZ>);
        summary_downsampled_cloud_.reset(new pcl::PointCloud<pcl::PointXYZ>);

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
            std::bind(&RelocationNode::pointcloud_sub_callback, this, std::placeholders::_1));

        init_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            "/initialpose",
            rclcpp::SensorDataQoS(),
            std::bind(&RelocationNode::initial_pose_callback, this, std::placeholders::_1));

        // publishers
        // 发布降采样的全局点云
        pointcloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/global_pcd_map", 10);
        // 发布配准后的点云
        pointcloud_registered_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("/pointcloud_registered", 10);
        // 发布状态，可能会给决策节点用，想法是reset状态急停，等待配准
        status_pub_ = this->create_publisher<std_msgs::msg::String>("/registration_status", 10);
        timer_ = this->create_wall_timer(std::chrono::milliseconds(200), std::bind(&RelocationNode::timer_callback, this));

        init_current_clouds_vector.reserve(init_accumulation_counter_ + 10); // 只预留空间，不分配内存，不创建对象
    }

    RelocationNode::~RelocationNode(){
        if (quatro_future_.valid())
        {
            quatro_future_.wait(); // 等待完成
        }
    }
    void RelocationNode::initial_pose_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
    {
        if (use_rviz_revise_ && !is_QUAandGICP_running_.load()) // enable reset initial pose // 是否得到quatro++的初始配准
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
        else // 提示等待 Quatro 计算
        {
            RCLCPP_ERROR(get_logger(), "正在初始化配准或者不允许使用rviz设定初始位姿，请修改参数或等待");
        }
    }

    void RelocationNode::pointcloud_sub_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        if (test_cloud_registered_)
        {
            if (is_first_callback_)
            {
                last_callback_time_ = std::chrono::steady_clock::now();
                is_first_callback_ = false;
            }
            auto t_now = std::chrono::steady_clock::now();
            auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(t_now - last_callback_time_).count(); // 调试信息
            RCLCPP_WARN(this->get_logger(), "点云发布间隔时间：%ld ms", elapsed_ms);
        }
        else if (!is_QUAandGICP_running_.load())
        {
            if (state_.load() == State::INIT && !use_rviz_revise_)
            {
                accumulate_cloud_then_QUAandGICP_with_debug(msg, init_accumulation_counter_);
            }
            else if (state_.load() == State::TRACKING)
            {
                // small_gicp 连续配准,滑动窗口
                GICP_tracking(msg);
            }
            else if (state_.load() == State::RESET)
            {
                accumulate_cloud_then_QUAandGICP(msg, reset_accumulation_counter_);
            }
            else
            {
                // use_rviz_revise_
                if (!getInitialPose_)
                {
                    RCLCPP_INFO(get_logger(), "当前模式为使用rviz给出初始位姿，等待初始位姿中......");
                    return;
                }
                GICP_tracking(msg);
            }
        }else{
            RCLCPP_ERROR(get_logger(), "正在初始化配准中，当前点云被丢弃");
        }
    }

    void RelocationNode::timer_callback()
    {
        if (cloud_) // test reading pcd
        {
            cloud_->header.stamp = this->now();
            pointcloud_pub_->publish(*cloud_);
        }
        // 发布状态
        std_msgs::msg::String status_msg;
        status_msg.data = state_.load() == State::INIT ? "INIT" : (state_.load() == State::TRACKING ? "TRACKING" : "RESET");
        status_pub_->publish(status_msg);
        
    }

    void RelocationNode::relocalization(pcl::PointCloud<pcl::PointXYZ>::Ptr current_sum_cloud_)
    {
        if (!getInitialPose_)
        {
            RCLCPP_WARN(get_logger(), "No initial pose received. Skipping relocalization.");
            return;
        }

        // smallgicp配准器
        small_gicp::Registration<small_gicp::GICPFactor, small_gicp::ParallelReductionOMP> registration;
        // 设置参数
        registration.reduction.num_threads = num_threads_; // 线程数
        registration.rejector.max_dist_sq = max_dist_sq_;  // 最大对应点距离
        // 收敛条件
        registration.optimizer.max_iterations = gicp_max_iterations_;          // 最大迭代次数（默认通常是30-50）
        registration.criteria.translation_eps = gicp_convergence_tolerance_;   // 收敛阈值（默认1e-4）

        // 只进行了下采样
        source_cloud_PointCovariance_ = small_gicp::voxelgrid_sampling_omp<pcl::PointCloud<pcl::PointXYZ>,      // 输入类型
                                                              pcl::PointCloud<pcl::PointCovariance>// 输出类型
                                                              >(*current_sum_cloud_, gicp_voxel_size_);
        // 计算协方差
        small_gicp::estimate_covariances_omp(
            *source_cloud_PointCovariance_,
            num_neighbors_,
            num_threads_);

        if (state_.load() == State::TRACKING)
        {
            RCLCPP_INFO(this->get_logger(), "函数relocalization：：small_gicp的tracking模式配准");
            result = registration.align(*global_map_PointCovariance_, *source_cloud_PointCovariance_, *target_tree_, pre_result_);
        }
        // RESET + INIT是一个逻辑：使用quatro++进行初始积累，成功后切换到small_gicp
        else /*if(state_ == State::INIT)*/
        {
            RCLCPP_INFO(this->get_logger(), "函数relocalization：：small_gicp的INIT/RESET模式配准");
            result = registration.align(*global_map_PointCovariance_, *source_cloud_PointCovariance_, *target_tree_, initial_guess_);

            if (!result.converged /*&& !doFirstRegistration_*/)
            {
                RCLCPP_ERROR(get_logger(), "cannot do first registration,reset,result error:%f", result.error);
                reset();
                transform.header.frame_id = "map";
                transform.child_frame_id = current_sum_cloud_->header.frame_id;
                transform.transform = tf2::eigenToTransform(initial_guess_).transform;
                transform.header.stamp = this->now();      // 添加时间戳
                tf_broadcaster_->sendTransform(transform); // 发布
                RCLCPP_ERROR(get_logger(), "函数relocalization：：small_gicp配准失败，发布quatro结果作为fallback");
                return;
            }
        }

        // publish map->odom tf
        if (result.converged)
        {
            // pre_result_ = result.T_target_source.inverse();
            pre_result_ = result.T_target_source;
            // doFirstRegistration_ = true;
            Eigen::Isometry3d T_map_odom = pre_result_;

            // publish transform
            transform.header.stamp = this->now();
            transform.header.frame_id = "map";
            transform.child_frame_id = current_sum_cloud_->header.frame_id;
            transform.transform = tf2::eigenToTransform(T_map_odom).transform;
            RCLCPP_WARN(get_logger(), "函数relocalization：：!!!!!完成gicp配准，Published map->odom transform,result error:%f", result.error);
            transform.header.stamp = this->now();
            tf_broadcaster_->sendTransform(transform);

            /***********************发布配准后的点云 start************************/
            sensor_msgs::msg::PointCloud2 current_cloud_pub_msg;
            pcl::toROSMsg(*current_sum_cloud_, current_cloud_pub_msg);
            current_cloud_pub_msg.header.frame_id = current_sum_cloud_->header.frame_id; 
            current_cloud_pub_msg.header.stamp = this->now();
            pointcloud_registered_pub_->publish(current_cloud_pub_msg);


            /************************更新状态*****************************/

            if (state_.load() == State::INIT || state_.load() == State::RESET)
            {
                state_.store(State::TRACKING);
            }
        }
        else
        {
            gicp_failed_counter_++;
            if (gicp_failed_counter_ > 1)
            {
                gicp_failed_counter_ = 0;
                reset();
                RCLCPP_ERROR(get_logger(), "GICP两次配准失败，重置系统，从quatro开始");
            }

            RCLCPP_ERROR(get_logger(), "Relocalization failed to converge,result error:%f", result.error);
        }
    }

    void RelocationNode::reset()
    {
        pre_result_ = Eigen::Isometry3d::Identity();
        getInitialPose_ = false;
        // doFirstRegistration_ = false;
        state_.store(State::RESET);
        // 状态重置
        track_slide_window_clouds_queue.clear();
        is_queue_full_ = false;
        gicp_run_counter_ = 0;

    }

    void RelocationNode::load_pcd_map(const std::string &map_path)
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
            global_map_downsampled_ = small_gicp::voxelgrid_sampling_omp(*global_map_, map_voxel_size_);
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
        }
        // global_map_.reset(new pcl::PointCloud<pcl::PointXYZ>);
        // 1. 加载下采样点云
        if (pcl::io::loadPCDFile<pcl::PointXYZ>(downsampled_pcd_file_, *global_map_downsampled_) == -1)
        {
            RCLCPP_ERROR(get_logger(), "Failed to load PCD map: %s", downsampled_pcd_file_.c_str());
            return;
        }
        RCLCPP_INFO(get_logger(), "Loaded PCD map with %ld points", global_map_downsampled_->size());

        auto t_start = std::chrono::steady_clock::now();

        // 2. 计算点云协方差（GICP 配准需要）
        // 传入降采样，体素大小相同，那么不会再降采样，主要的是【类型转换（XYZ → Covariance）】
        global_map_PointCovariance_ = small_gicp::voxelgrid_sampling_omp<pcl::PointCloud<pcl::PointXYZ>, pcl::PointCloud<pcl::PointCovariance>>(*global_map_downsampled_, map_voxel_size_); 
        small_gicp::estimate_covariances_omp(*global_map_PointCovariance_, num_neighbors_, num_threads_);

        // build target kd_tree
        // 3. 构建 KD-Tree（加速最近邻搜索）
        target_tree_ = std::make_shared<small_gicp::KdTree<pcl::PointCloud<pcl::PointCovariance>>>(
            global_map_PointCovariance_, // 不要解引用，直接传智能指针
            small_gicp::KdTreeBuilderOMP(num_threads_));

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

    void RelocationNode::QUA_GICP_init_and_reset(std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> protecting_vector, int count_num)
    {

        bool if_valid_;
        Eigen::Matrix4d output_tf_ = m_quatro_handler->align(*summary_downsampled_cloud_, *global_map_downsampled_, if_valid_);

        if (if_valid_)
        {
            initial_guess_ = Eigen::Isometry3d(output_tf_);
            getInitialPose_ = true;
            RCLCPP_INFO(this->get_logger(), "已经完成%d帧积累降采样点云的Quatro++配准，initial_guess_已填充，将交给small_gicp进行精配准", count_num);

            // small_gicp 精配准
            pcl::PointCloud<pcl::PointXYZ>::Ptr current_sum_cloud_for_gicp(new pcl::PointCloud<pcl::PointXYZ>);
            for (size_t i = 0; i < static_cast<size_t>(track_accumulation_counter_); ++i)
            {
                *current_sum_cloud_for_gicp += *protecting_vector[protecting_vector.size() - i - 1];
            }
            relocalization(current_sum_cloud_for_gicp);
            std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>().swap(protecting_vector); // 清空并释放内存
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Quatro 配准失败，将重新积累点云");
            std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>().swap(protecting_vector); // 清空并释放内存
        }
        return;
    }

    void RelocationNode::QUA_GICP_init_and_reset_with_debug(std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> protecting_vector, int count_num)
    {
        RCLCPP_INFO(this->get_logger(), "开始quatro++和GICP算法运行");
        bool if_valid_;
        // Quatro 配准
        /**/ auto t_start = std::chrono::steady_clock::now(); // 调试信息
        Eigen::Matrix4d output_tf_ = m_quatro_handler->align(*summary_downsampled_cloud_, *global_map_downsampled_, if_valid_);
        /**/ auto t_end = std::chrono::steady_clock::now();                                                    // 调试信息
        /**/ auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(t_end - t_start).count(); // 调试信息
        /**/ RCLCPP_ERROR(this->get_logger(), "Quatro++ 配准所用时间：%ld ms", elapsed_ms);                       // 调试信息
        if (if_valid_)
        {
            initial_guess_ = Eigen::Isometry3d(output_tf_);
            getInitialPose_ = true;
            RCLCPP_INFO(this->get_logger(), "已经完成%d帧积累降采样点云的Quatro++配准，initial_guess_已填充，将交给small_gicp进行精配准", count_num);

            // small_gicp 精配准
            /**/ t_start = std::chrono::steady_clock::now(); // 调试信息
            pcl::PointCloud<pcl::PointXYZ>::Ptr current_sum_cloud_for_gicp(new pcl::PointCloud<pcl::PointXYZ>);
            for (size_t i = 0; i < static_cast<size_t>(track_accumulation_counter_); ++i)
            {
                *current_sum_cloud_for_gicp += *protecting_vector[protecting_vector.size() - i - 1];
            }
            relocalization(current_sum_cloud_for_gicp);
            std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>().swap(protecting_vector);
            /**/ t_end = std::chrono::steady_clock::now();
            /**/ elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(t_end - t_start).count(); // 调试信息
            /**/ RCLCPP_WARN(this->get_logger(), "small_gicp 精配准所用时间：%ld ms", elapsed_ms);            // 调试信息
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "Quatro 配准失败，将重新积累点云");
            std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr>().swap(protecting_vector); // 清空并释放内存
        }
        return;
    }

    void RelocationNode::GICP_tracking(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        RCLCPP_INFO(this->get_logger(), "GICP_tracking 开始运行");
        auto current_cloud = std::make_shared<pcl::PointCloud<pcl::PointXYZ>>();
        pcl::fromROSMsg(*msg, *current_cloud);
        if (!is_queue_full_)
        {
            track_slide_window_clouds_queue.push_back(current_cloud);
            if (track_slide_window_clouds_queue.size() >= static_cast<size_t>(track_accumulation_counter_))
            {
                // 划定窗口大小
                is_queue_full_ = true;
            }
            return;
        }
        else
        {
            // 滑动窗口核心：每接到新的一帧，剔除最开始的一帧，补充新的一帧，保持窗口大小不变后加和
            track_slide_window_clouds_queue.pop_front();
            track_slide_window_clouds_queue.push_back(current_cloud);
            RCLCPP_INFO(this->get_logger(), "GICP_tracking 滑动窗口已更新，当前窗口大小：%ld", track_slide_window_clouds_queue.size());
        }

        /****************************控制gicp频率*************************/
        gicp_run_counter_++;
        if (gicp_run_counter_ % tracking_frequency_divisor_ != 0)
        {
            return; // 如果运行次数没有到阈值整数倍，直接结束这个函数，不进行gicp
        }

        pcl::PointCloud<pcl::PointXYZ>::Ptr current_sum_cloud_for_gicp(new pcl::PointCloud<pcl::PointXYZ>);

        for (const auto &cloud : track_slide_window_clouds_queue)
        {
            *current_sum_cloud_for_gicp += *cloud;
        }
        relocalization(current_sum_cloud_for_gicp);
    }

    void RelocationNode::accumulate_cloud_then_QUAandGICP(const sensor_msgs::msg::PointCloud2::SharedPtr msg, int count_num)
    {
        if (!is_QUAandGICP_running_.load()) // 正在跑配准时，新来的点云直接丢弃，不添加到vector里，保证点云实时性
        {
            // 1. 新创建智能指针，存储当前帧点云
            pcl::PointCloud<pcl::PointXYZ>::Ptr current_cloud(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::fromROSMsg(*msg, *current_cloud);

            // 2. 加入vector，加锁
            std::lock_guard<std::mutex> lock(accumulation_mutex_);
            init_current_clouds_vector.push_back(current_cloud);

            // 3. 到达阈值执行配准流程，先对阈值里的点云加和+降采样
            if (init_current_clouds_vector.size() >= static_cast<size_t>(count_num))
            {
                // 使用前清空
                current_accumulated_cloud_->clear();
                for (const auto &cloud : init_current_clouds_vector)
                {
                    *current_accumulated_cloud_ += *cloud;
                }
                summary_downsampled_cloud_ = small_gicp::voxelgrid_sampling_omp(*current_accumulated_cloud_, quatro_voxel_size_);

                // 4. 多线程执行配准，避免quatro长时间阻塞，同时避免这段时间新的点云进来
                is_QUAandGICP_running_.store(true);

                // 5. 拷贝，防止多线程竞争
                auto protecting_vector = init_current_clouds_vector;
                init_current_clouds_vector.clear();

                // 6. 主线程视角里，这个lambda被“跳过“，但是【新线程开始执行】，与主线程并行
                quatro_future_ = std::async(std::launch::async, [this, protecting_vector, count_num]()
                { 
                    QUA_GICP_init_and_reset(protecting_vector, count_num);

                    is_QUAandGICP_running_.store(false); 
                });

                // 7. 【主线程视角】：到lambda直接跳到这里，可以继续接收处理timer和点云回调
            }
        }
    }
    void RelocationNode::accumulate_cloud_then_QUAandGICP_with_debug(const sensor_msgs::msg::PointCloud2::SharedPtr msg, int count_num)
    {
        RCLCPP_INFO(this->get_logger(), "开始积累点云，后quatro++和GICP【初始位姿】配准，当前为accumulate_cloud_then_QUAandGICP_with_debug函数，如果正常接下来会打印【10帧积累【降采样】所用时间】");
        if (!is_QUAandGICP_running_.load())
        {
            // 新创建智能指针，存储当前帧点云
            pcl::PointCloud<pcl::PointXYZ>::Ptr current_cloud(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::fromROSMsg(*msg, *current_cloud);

            std::lock_guard<std::mutex> lock(accumulation_mutex_);
            init_current_clouds_vector.push_back(current_cloud);

            if (init_current_clouds_vector.size() >= static_cast<size_t>(count_num))
            {
                /**/ auto t_start = std::chrono::steady_clock::now(); // 调试信息
                current_accumulated_cloud_->clear();
                for (const auto &cloud : init_current_clouds_vector)
                {
                    *current_accumulated_cloud_ += *cloud;
                }
                summary_downsampled_cloud_ = small_gicp::voxelgrid_sampling_omp(*current_accumulated_cloud_, quatro_voxel_size_);
                /**/ auto t_end = std::chrono::steady_clock::now();                                                    // 调试信息
                /**/ auto elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(t_end - t_start).count(); // 调试信息
                /**/ RCLCPP_WARN(this->get_logger(), "10帧积累【降采样】所用时间：%ld ms", elapsed_ms);                // 调试信息

                is_QUAandGICP_running_.store(true);

                auto protecting_vector = init_current_clouds_vector;
                init_current_clouds_vector.clear();

                RCLCPP_INFO(this->get_logger(), "开始多线程执行和GICP精确配准，将验证是否开启多线程");
                /**/ t_start = std::chrono::steady_clock::now();                                                      // 调试信息
                quatro_future_ = std::async(std::launch::async, [this, protecting_vector, count_num]()
                { 
                    QUA_GICP_init_and_reset_with_debug(protecting_vector, count_num);

                    is_QUAandGICP_running_.store(false); 
                });
                /**/ t_end = std::chrono::steady_clock::now();
                /**/ elapsed_ms = std::chrono::duration_cast<std::chrono::milliseconds>(t_end - t_start).count();            // 调试信息
                /**/ RCLCPP_WARN(this->get_logger(), "如果正常多线程，这里时间应该非常小，小于1ms，【所用时间】：%ld ms", elapsed_ms);   // 调试信息
            }
        }
    }
}
