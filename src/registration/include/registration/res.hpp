#include "rclcpp/rclcpp.hpp"

#pragma once
#include <rclcpp/rclcpp.hpp>
#include <chrono>
#include <iostream>
#include <random>

#include <tf2_eigen/tf2_eigen.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>
#include <sensor_msgs/msg/point_field.hpp>
#include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>

// eigen libs
#include <Eigen/Geometry>
#include <Eigen/Core>

// small_gicp libs
#include <small_gicp/util/downsampling_omp.hpp>
#include <small_gicp/pcl/pcl_registration.hpp>
#include <small_gicp/pcl/pcl_point.hpp>
#include <small_gicp/pcl/pcl_point_traits.hpp>
#include <small_gicp/benchmark/read_points.hpp>

// pcl libs
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/approximate_voxel_grid.h>

// teaserpp libs
#include <teaser/ply_io.h>
#include <teaser/registration.h>

#include <mutex>
namespace hnurm
{

    class RelocaliztionNode : public rclcpp::Node
    {
    public:
        explicit RelocaliztionNode(const rclcpp::NodeOptions &options);
        ~RelocaliztionNode()
        {
            RCLCPP_INFO(get_logger(), "RelocaliztionNode destroyed");
        }
        RelocaliztionNode(const RelocaliztionNode &) = delete;
        RelocaliztionNode &operator=(const RelocaliztionNode &) = delete;
        RelocaliztionNode(RelocaliztionNode &&) = delete;
        RelocaliztionNode &operator=(RelocaliztionNode &&) = delete;

    private:
        /*@ pointcloud_sub_callback
         * @brief 点云数据订阅回调函数
         * @param msg 接收到的PointCloud2类型点云消息
         * @订阅话题：/pointcloud
         */
        void pointcloud_sub_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg);
        //void recv_sub_callback(const hnurm_interfaces::msg::VisionRecvData::SharedPtr msg);

        /*@ initial_pose_callback
         * @brief 初始位姿订阅回调函数
         * @param msg 接收到的带协方差的位姿消息(PoseWithCovarianceStamped)
         * @订阅话题：/initial_pose
         */
        void initial_pose_callback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);

        /*@ load_pcd_map
         * @brief 加载PCD格式的地图点云文件
         * @param map_path PCD地图文件的路径
         */
        void load_pcd_map(const std::string &map_path);

        /*@ timer_callback
         * @brief 定时器回调函数，用于执行重定位主逻辑
         */
        void timer_callback();

        /*@ timer_pub_tf_callback
         * @brief 定时器回调函数，用于发布TF变换
         */
        void timer_pub_tf_callback();


        /*@ relocalization
         * @brief 执行点云配准重定位的核心算法
         */
        void relocalization();
        /*@ reset
         * @brief 重置重定位节点的内部状态
         */
        void reset();
        /*@ publishTransform
         * @brief 将变换矩阵发布到TF树
         * @param transform_matrix 4x4的变换矩阵
         */
        void publishTransform(const Eigen::Matrix4d &transform_matrix);

        /*@ generate_initial_guesses
         * @brief 根据初始位姿生成多个带噪声的初始猜测位姿，用于多初值配准
         * @param initial_pose 初始位姿（Isometry3d格式）
         * @param trans_noise 平移噪声（单位：米），默认0.5
         * @param rot_noise 旋转噪声（单位：弧度），默认π/6
         * @return 包含多个初始猜测位姿的向量
         */
        std::vector<Eigen::Isometry3d> generate_initial_guesses(const Eigen::Isometry3d &initial_pose, double trans_noise = 0.5, double rot_noise = M_PI / 6);

        rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_sub_;
        //rclcpp::Subscription<hnurm_interfaces::msg::VisionRecvData>::SharedPtr recv_sub_;
        rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr init_pose_sub_;

        // 点云发布器，用于降采样全局的点云（调试用）
        rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_pub_;
        // 重定位主定时器，周期性执行配准算法
        rclcpp::TimerBase::SharedPtr timer_;
        // TF发布定时器，周期性发布map到base_link的变换
        rclcpp::TimerBase::SharedPtr timer_pub_tf_;

        // TF监听器，用于查询坐标系之间的变换关系
        std::unique_ptr<tf2_ros::TransformListener> tf_listener_;
        // TF缓冲区，存储TF树的历史变换数据
        std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
        // TF广播器，用于发布配准结果（map→base_link）
        std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;





        /*********************************点云地图数据*****************************************/
        pcl::PointCloud<pcl::PointXYZ>::Ptr global_map_;             // 原始全局地图点云，从PCD文件加载
        pcl::PointCloud<pcl::PointXYZ>::Ptr global_map_downsampled_; // 降采样后的全局地图点云，用于加速配准
        pcl::PointCloud<pcl::PointCovariance>::Ptr global_map_PointCovariance_; // 带协方差信息的全局地图点云，用于small_gicp




        /*********************************实时点云数据*****************************************/
        int accumulation_counter_ = 0;
        pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud_;             // 积累后的点云，或者当前帧接收到的原始点云，【来自fast_lio2的odom系下的当前帧点云的坐标】
        pcl::PointCloud<pcl::PointXYZ>::Ptr accumulate_cloud_;         // 当前帧点云，会被积累，用于提高配准稳定性
        pcl::PointCloud<pcl::PointXYZ>::Ptr source_cloud_downsampled_; // 降采样后的当前帧点云

        pcl::PointCloud<pcl::PointCovariance>::Ptr source_cloud_PointCovariance_; // 带协方差信息的当前帧点云，用于small_gicp

        geometry_msgs::msg::Transform self_blue_trans_; // 	蓝方机器人的初始位姿（比赛场景）
        geometry_msgs::msg::Transform self_red_trans_;  // 	红方机器人的初始位姿（比赛场景）


        /*@ parsePose
         * @brief 从向量解析位姿数据转换为Transform消息
         * @param pose 包含7个元素的向量[x, y, z, qx, qy, qz, qw]
         * @return geometry_msgs::msg::Transform类型的变换消息
         * @throws std::runtime_error 当输入向量大小不等于7时抛出异常
         */
        geometry_msgs::msg::Transform parsePose(const std::vector<double> &pose)
        {
            if (pose.size() != 7)
            {
                RCLCPP_ERROR(get_logger(), "Invalid initial_pose size!");
                throw std::runtime_error("Invalid pose");
            }

            geometry_msgs::msg::Transform transform;
            transform.translation.x = pose[0];
            transform.translation.y = pose[1];
            transform.translation.z = pose[2];
            transform.rotation.x = pose[3];
            transform.rotation.y = pose[4];
            transform.rotation.z = pose[5];
            transform.rotation.w = pose[6];
            return transform;
        }

        bool is_initialized_ = false; // 节点是否完成初始化
        std::vector<Eigen::Isometry3d> guesses_; // 多个初始位姿猜测（用于多初始值优化）
        Eigen::Isometry3d initial_guess_ = Eigen::Isometry3d::Identity(); // 单个初始位姿猜测，默认为单位矩阵

        geometry_msgs::msg::TransformStamped transform; // ROS2 格式变换消息，用于发布 TF

        // 自定义结构体，用于存储配准结果
        struct RegistrationResult
        {
            Eigen::Isometry3d transformation; // 变换矩阵，从源点云到目标点云的变换
            double error; // 配准误差（通常是点云对的均方误差）
            bool converged; // 是否收敛（是否满足停止条件）
        };

        // kd_tree
        std::shared_ptr<small_gicp::KdTree<pcl::PointCloud<pcl::PointCovariance>>> target_tree_; // 目标点云（地图）的 KD-Tree，加速最近邻搜索
        std::shared_ptr<small_gicp::KdTree<pcl::PointCloud<pcl::PointCovariance>>> source_tree_; // 源点云（当前帧）的 KD-Tree

        sensor_msgs::msg::PointCloud2::SharedPtr cloud_; // 下采样点云消息的智能指针
        bool getInitialPose_ = false; // 是否已经获得初始位姿
        bool doFirstRegistration_ = false; // 是否正在进行第一次配准
        Eigen::Isometry3d pre_result_ = Eigen::Isometry3d::Identity(); // 上一次配准的结果（用于连续性约束）
        small_gicp::RegistrationResult result; // 当前配准结果

        bool get_first_tf_from_quatro_ = false; // 	是否使用 Quatro 算法获取初始变换

        teaser::RobustRegistrationSolver::Params params; // TEASER 鲁棒配准算法的参数配置
        teaser::RobustRegistrationSolver solver(teaser::RobustRegistrationSolver::Params params); // TEASER/Quatro++ 求解器实例 - 一种基于图优化的鲁棒点云配准方法，对噪声和离群点具有强鲁棒性
        // quatro++ 处理模块
        shared_ptr<quatro<QuatroPointType>> m_quatro_handler = nullptr;

        // params
        std::string pointcloud_sub_topic_;         // 输入点云的订阅话题
        std::string vision_topic_;                 // 下位机发送的视觉相关数据话题
        std::string pcd_file_;                     // 地图点云文件路径
        std::string downsampled_pcd_file_;         // 下采样后的地图保存路径
        bool generate_downsampled_pcd_ = false;    // 是否生成下采样地图
        int num_threads_;                          // 并行计算的线程数
        int num_neighbors_;                        // KD-Tree 搜索的近邻数量
        float max_dist_sq_;                        // 最大对应点距离的平方（过滤远距离匹配）
        float source_voxel_size_;                  // 源点云（当前帧）下采样体素大小
        float map_voxel_size_;                     // 地图点云下采样体素大小

        bool use_quatro_ = false; // 是否启用 Quatro 算法
        bool use_fixed_ = false;  // 是否使用固定初始位姿

        // quatro++ params

        // 旋转估计的最大 GNC 迭代次数，最大对应点数量
        int m_rotation_max_iter_ = 100, m_num_max_corres_ = 200;
        // 法向量估计的搜索半径（米）、FPFH 特征计算的搜索半径、对应点距离阈值
        double m_normal_radius_ = 0.02, m_fpfh_radius_ = 0.04, m_distance_threshold_ = 30.0;
        // 噪声边界（用于鲁棒估计）、GNC（Graduated Non-Convexity）迭代因子、旋转优化的收敛阈值
        double m_noise_bound_ = 0.25, m_rotation_gnc_factor_ = 1.39, m_rotation_cost_thr_ = 0.0001;
        // 是否估计尺度（通常用于不同坐标系之间的配准）、是否使用优化的匹配策略
        bool m_estimate_scale_ = false, m_use_optimized_matching_ = true;

    protected:
        std::shared_ptr<RelocaliztionNode> shared_from_this()
        {
            return std::static_pointer_cast<RelocaliztionNode>(rclcpp::Node::shared_from_this());
        }
    };
}