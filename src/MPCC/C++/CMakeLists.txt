cmake_minimum_required(VERSION 3.5)
project(mpcc_control)

# 使用 C++14
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 查找依赖包
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(Python COMPONENTS Development REQUIRED)
find_package(eufs_msgs REQUIRED)
find_package(ackermann_msgs REQUIRED)

# 包含外部库和依赖包的路径
include_directories(
  ${CMAKE_SOURCE_DIR}
  include
  ${rclcpp_INCLUDE_DIRS}
  ${std_msgs_INCLUDE_DIRS}
  ${geometry_msgs_INCLUDE_DIRS}
  ${eufs_msgs_INCLUDE_DIRS}
  External/blasfeo/lib/include
  External/hpipm/lib/include
  External/matplotlib
  External/Eigen
  External/Json/include
)

# 添加可执行文件
add_executable(${PROJECT_NAME}_node
  mpcc_controller.cpp    # 修改为你实际的主程序文件
  main.cpp
  types.cpp
  types.h
  config.h  # 添加配置文件
  Params/params.cpp
  Spline/cubic_spline.cpp
  Spline/arc_length_spline.cpp
  Interfaces/hpipm_interface.cpp
  Interfaces/solver_interface.cpp
  Constraints/constraints.cpp
  Constraints/bounds.cpp
  Cost/cost.cpp
  Model/model.cpp
  Model/integrator.cpp
  MPC/mpc.cpp
  Params/track.cpp
  Params/track.h
  Plotting/plotting.cpp
  Plotting/plotting.h
)

# 链接依赖库
target_link_libraries(${PROJECT_NAME}_node
  ${rclcpp_LIBRARIES}
  ${std_msgs_LIBRARIES}
  ${geometry_msgs_LIBRARIES}
  ${Python_LIBRARIES}
  ${eufs_msgs_LIBRARIES}  # 添加 eufs_msgs 库
  ${ackermann_msgs_LIBRARIES}  # 添加这个
  ${CMAKE_SOURCE_DIR}/External/hpipm/lib/lib/libhpipm.a
  ${CMAKE_SOURCE_DIR}/External/blasfeo/lib/lib/libblasfeo.a
)

# 包含目录
target_include_directories(${PROJECT_NAME}_node PRIVATE
  ${rclcpp_INCLUDE_DIRS}
  ${Python_INCLUDE_DIRS}
  ${eufs_msgs_INCLUDE_DIRS}  # 添加 eufs_msgs 的 include 路径
)

# 安装目标和配置环境
install(TARGETS
  ${PROJECT_NAME}_node
  DESTINATION lib/${PROJECT_NAME}
 
)
# 安装整个 Params 目录
install(DIRECTORY Params
  DESTINATION share/${PROJECT_NAME}
)
# 导出依赖
ament_package()
