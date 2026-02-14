# registration — 小GICP点云重定位节点

## 功能概述
- 订阅激光点云（`pointcloud_sub_topic`，默认 `/cloud_registered`），将当前帧点云与全局地图进行配准，估计当前位姿。
- 支持从离线 PCD 地图加载（`pcd_file`），可按需自动生成/缓存降采样地图（`generate_downsampled_pcd` 与 `downsampled_pcd_file`）。
- 可选初始位姿估计策略：
  - TEASER++/QUATRO 粗配准获取初始位姿（`use_quatro`）。
  - 使用固定初始值（`use_fixed`）。
- 使用 small_gicp 实现并行化配准，支持 OpenMP 多线程（`num_threads`），可设置配准拒绝阈值（`max_dist_sq`）及体素降采样分辨率（`source_voxel_size`、`map_voxel_size`）。
- 订阅视觉/状态话题（`vision_topic`，默认 `/vision_recv_data`）以辅助初始化或逻辑判断（具体逻辑见源码）。

核心实现请参考：
- 类定义：[`hnurm::RelocaliztionNode`](hnurm_perception/registration/include/registration/registration_node.hpp)
- 节点实现：[hnurm_perception/registration/src/registration_node.cpp](hnurm_perception/registration/src/registration_node.cpp)

## 目录结构
- 构建配置
  - [hnurm_perception/registration/CMakeLists.txt](hnurm_perception/registration/CMakeLists.txt)
  - [hnurm_perception/registration/package.xml](hnurm_perception/registration/package.xml)
- 源码
  - 头文件：[hnurm_perception/registration/include/registration/registration_node.hpp](hnurm_perception/registration/include/registration/registration_node.hpp)
  - 源文件：[hnurm_perception/registration/src/registration_node.cpp](hnurm_perception/registration/src/registration_node.cpp)
  - 入口：[hnurm_perception/registration/src/main.cpp](hnurm_perception/registration/src/main.cpp)
- 启动文件
  - [hnurm_perception/registration/launch/registration.launch.py](hnurm_perception/registration/launch/registration.launch.py)
- 参数
  - 默认参数：[hnurm_perception/registration/params/default.yaml](hnurm_perception/registration/params/default.yaml)

## 启动方式
- 使用默认参数启动：
  ```bash
  ros2 launch registration registration.launch.py
  ```
- 使用自定义参数文件启动：
  ```bash
  ros2 launch registration registration.launch.py params_file:=/path/to/your.yaml
  ```

## 参数说明（可在 YAML 中设置）
- 订阅与地图
  - `pointcloud_sub_topic` (string, 默认 `/cloud_registered`)：输入点云话题。
  - `vision_topic` (string, 默认 `/vision_recv_data`)：视觉/状态话题。
  - `pcd_file` (string)：全局地图 PCD 文件路径。
  - `generate_downsampled_pcd` (bool, 默认 `false`)：是否生成降采样地图并保存。
  - `downsampled_pcd_file` (string)：降采样地图保存路径。
- 配准与性能
  - `num_threads` (int, 默认 `4`)：配准并行线程数（需启用 OpenMP）。
  - `max_dist_sq` (float)：匹配点对的最大距离平方（用于离群点剔除）。
  - `source_voxel_size` (float)：当前帧点云体素下采样分辨率。
  - `map_voxel_size` (float)：地图点云体素下采样分辨率。
- 初值策略
  - `use_quatro` (bool, 默认 `false`)：启用 QUATRO/TEASER++ 进行初始位姿估计。
  - `use_fixed` (bool, 默认 `false`)：使用固定初始位姿（固定值的设置见源码或扩展参数）。

实际使用的参数集合与默认值以源码为准：
- 头文件字段：[`hnurm::RelocaliztionNode` 成员与参数声明](hnurm_perception/registration/include/registration/registration_node.hpp)
- 参数声明与使用位置：[hnurm_perception/registration/src/registration_node.cpp](hnurm_perception/registration/src/registration_node.cpp)

## 示例参数（自定义 YAML）
```yaml
registration:
  ros__parameters:
    # 输入/辅助话题
    pointcloud_sub_topic: /cloud_registered
    vision_topic: /vision_recv_data

    # 地图
    pcd_file: /path/to/all_raw_points.pcd
    generate_downsampled_pcd: true
    downsampled_pcd_file: /path/to/all_raw_points_downsampled.pcd

    # 配准与性能
    num_threads: 4
    max_dist_sq: 0.25          # 距离阈值平方: 0.5m -> 0.25
    source_voxel_size: 0.1
    map_voxel_size: 0.1

    # 初值
    use_quatro: true
    use_fixed: false
```

## 依赖
构建/运行依赖见 [CMakeLists.txt](hnurm_perception/registration/CMakeLists.txt)：
- ROS 2: rclcpp、sensor_msgs、tf2、tf2_ros、tf2_eigen、pcl_ros、pcl_conversions、geometry_msgs
- 第三方: PCL、OpenMP、Eigen3、teaserpp、quatro
