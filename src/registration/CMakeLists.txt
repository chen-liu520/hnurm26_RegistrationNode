cmake_minimum_required(VERSION 3.8)
project(registration)

set(CMAKE_CXX_STANDARD 17)   # 设置 C++ 标准为 C++17，且强制要求支持
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_definitions(-Wall -Werror) # 启用所有警告，并将警告视为错误

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies(REQUIRED
  rclcpp 
  sensor_msgs 
  tf2 
  tf2_ros 
  tf2_eigen
  pcl_conversions
  geometry_msgs
  pcl_ros
  quatro
  hnurm_interfaces
)
find_package(PCL REQUIRED)
find_package(OpenMP REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(teaserpp REQUIRED)
find_package(small_gicp REQUIRED)

if(OpenMP_CXX_FOUND)
  add_definitions(-DOPENMP_AVAILABLE)  # 定义宏告诉small_gicp已启用OpenMP
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}") # 添加编译标志
endif()

ament_auto_add_executable(registration_node 
                src/main.cpp 
                src/registration_node.cpp) # 创建可执行文件

target_include_directories(registration_node PRIVATE
  ${PCL_INCLUDE_DIRS}
)

target_link_libraries(registration_node
  ${PCL_LIBRARIES}
  Eigen3::Eigen
  OpenMP::OpenMP_CXX
  teaserpp::teaser_registration 
  teaserpp::teaser_io
  small_gicp::small_gicp
  #pcl_ros::pcl_ros
)

ament_auto_package(
        INSTALL_TO_SHARE
        launch   # 安装 launch 文件夹
        params   # 安装 params 文件夹
        rviz     # 安装 rviz 配置文件夹
)


